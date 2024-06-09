#include "Worker.h"

#include <iostream>
#include <chrono>
#include <random>

#include <sys/time.h>
#include <ctime>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include "../helper/definitions.h"
#include "../platerecognizer/recognizertool.h"
#include "../webapi/webapi.h"
#include "../helper/Helper.h"
#include "../thirdparty/json/json/json.h"
#include "../platerecognizercom/PlateRecognizerComAPI.h"

#include "PlateWorker.h"

Worker::WorkerList Worker::workers;
std::vector<WorkerState> Worker::workerTasks;

WorkerTaskData::WorkerTaskData(std::string _uuid, cv::Mat _frame, int camera_id) : uuid(_uuid), frame(_frame), camera_id(camera_id) {}

WorkerState::WorkerState(WorkerProcessStatus status) : m_status(status) {}
WorkerState::WorkerState(WorkerProcessStatus status, WorkerTaskData taskData) : m_data(taskData), m_status(status) {}

Worker::Worker(int index) : m_index(index)
{
}

Worker::~Worker()
{
    stopWorking();
}

void Worker::createAndStartWorkers(int size)
{
    for (int i = workers.size(); i < size; i++)
    {
        workerTasks.push_back(WorkerState(WaitingNewTask));
        std::unique_ptr<Worker> ptr(new Worker(i));
        ptr->start();
        workers.push_back(std::move(ptr));
    }
}

void Worker::startWorking()
{
    if (!m_thread)
    {
        m_shouldStop = false;
        m_thread.reset(new std::thread(&Worker::process, this));
    }
}

void Worker::stopWorking()
{
    if (m_thread)
    {
        m_shouldStop = true;
        m_thread->join();
        m_thread.reset();
    }
}

void Worker::process()
{
    while (!m_shouldStop)
    {
        std::unique_lock<std::mutex> lock(notify_mutex);

        /// Wait until the task is created
        if (m_index >= workerTasks.size() || workerTasks.at(m_index).m_status == WaitingNewTask)
        {
            notify_convariable.wait(lock);
            continue;
        }

        workerTasks[m_index].m_status = WorkingOnTask;

        dowork2();

        workerTasks[m_index].m_status = WaitingNewTask;
    }
}

void Worker::start()
{
    startWorking();
}

void Worker::stop()
{
    stopWorking();
}

void Worker::addTask(WorkerTaskData data)
{

    bool isAssigned = false;
    while (!isAssigned)
    {
        for (int i = 0; i < workerTasks.size(); i++)
        {
            if (workerTasks[i].m_status != WaitingNewTask)
            {
                continue;
            }

            std::unique_lock<std::mutex> lock(workers[i]->notify_mutex);

            workerTasks[i] = WorkerState(AssignedNewTask, data);
            isAssigned = true;

            workers[i]->notify_convariable.notify_all();

            break;
        }
    }
}

void Worker::stopAll()
{
    for (int i = 0; i < workers.size(); i++)
    {
        // mark the worker as should stop
        workers[i]->m_shouldStop = true;

        // unlock the notify mutex
        // When unlock the notify mutex, the worker will check the m_shouldStop flag
        // and if m_shouldStop is true, it's will be ends dowork funtion
        std::unique_lock<std::mutex> lock(workers[i]->notify_mutex);
        workers[i]->notify_convariable.notify_all();
    }

    for (int i = 0; i < workers.size(); i++)
    {
        // wait until the worker stop (join)
        // and reset the thread for the next start
        workers[i]->stop();
    }
}

void Worker::imageWrite(const cv::Mat &image, const std::string path)
{
    // Support for writing JPG
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);

    cv::imwrite(path, image, compression_params);
}

void Worker::dowork()
{

    WorkerTaskData data = Worker::workerTasks[m_index].m_data;

    try
    {
        if (EXPORT_VEHICLE_IMAGE)
        {
            std::string path = APP_RUN_PATH + "car_img_plate/";
            path += Worker::workerTasks[m_index].m_data.uuid;
            path += ".jpg";

            Worker::imageWrite(data.frame, path);
        }
        cv::Mat orgImage = data.frame.clone();
        std::vector<RecognizerTool::DetectedPlate> detectedPlates;
        RecognizerTool::DetectStatus detechStatus = RecognizerTool::detect_plate_on_image(data.frame, detectedPlates);
        if (detechStatus == RecognizerTool::DetectStatus::PlateWasFound)
        {
            int i = 0;
            for (auto &&dPlate : detectedPlates)
            {
                i++;
                std::cout << "UUID: " << data.uuid << ", Plate: " << dPlate.plateStr << std::endl;

                if (EXPORT_VEHICLE_IMAGE)
                {
                    std::string path = APP_RUN_PATH + "car_img_plate_detected/";
                    path += Worker::workerTasks[m_index].m_data.uuid;
                    path += "_" + std::to_string(i);
                    path += ".jpg";

                    std::cout << path << std::endl;

                    Worker::imageWrite(dPlate.plateImg, path);
                }

                std::string auth_token;

                if (WebAPI::getAuthToken(auth_token))
                {
                    std::string vehicleImageBase64, numberPlateImageBase64;
                    /*BEGIN UPLOAD TO WEB API PROCESS*/
                    {
                        std::vector<uchar> buf;
                        cv::imencode(".jpg", orgImage, buf);
                        auto base64_png = reinterpret_cast<const unsigned char *>(buf.data());
                        vehicleImageBase64 = std::move("image/jpeg;base64," + Helper::base64_encode(base64_png, buf.size()));
                    }

                    {
                        std::vector<uchar> buf;
                        cv::imencode(".jpg", dPlate.plateImg, buf);
                        auto base64_png = reinterpret_cast<const unsigned char *>(buf.data());
                        numberPlateImageBase64 = std::move("image/jpeg;base64," + Helper::base64_encode(base64_png, buf.size()));
                    }
                    
                    auto dt = std::chrono::time_point_cast<std::chrono::milliseconds>(
                    	std::chrono::system_clock::now()
                    );

                    std::string station_id = "";
					uint16_t logtype = 2;
                    
                    if (data.camera_id == 1) {
						logtype = 1;
                        station_id = "28038"; // giris id
                    } else if (data.camera_id == 2) {
                        station_id = "28039"; // cikis id
						logtype = 2;
                    }else logtype = 2; //defult
                    
                    Json::Value jReq;
                    jReq["carAccessDateTime"] = std::to_string((int)(dt.time_since_epoch().count() / 1000));
                    jReq["systemTime"] = std::to_string((int)(dt.time_since_epoch().count() / 1000));
                    jReq["alprsLogType"] = unsigned(logtype);
                    jReq["vehicleImageFileName"] = "img7.jpeg";
                    jReq["numberPlateImageFileName"] = "img7-plate.jpeg";
                    jReq["carRegistrationNumber"] = dPlate.plateStr;
                    jReq["numberPlateCharacters"] = dPlate.plateStr;
                    jReq["carDetailId"] = unsigned(1);
                    jReq["alprsSystemId"] = "";
                    jReq["alprsStationId"] = station_id;
                    jReq["alprsSystemSerialNumber"] = "";
                    jReq["username"] = "alpdade";
                    jReq["password"] = "alpdade";
                    jReq["geographicalAreaNameCode"] = "AU";
                    jReq["vehicleMake"] = ""; //"Porsche"
               		jReq["vehicleModel"] = ""; //"cayenne";
                    jReq["alprsContrastValue"] = "60";
                    jReq["numberPlateRecognitionScore"] = "90";
                    jReq["vehicleImageFile"] = vehicleImageBase64;
                    jReq["numberPlateImageFile"] = numberPlateImageBase64;
                    jReq["alprsFK"] = "string";
                    jReq["bookingId"] = unsigned(1);

                    // std::cout << "JSON stringify: " << jReq.toStyledString() << std::endl;
                    bool success = WebAPI::uploadPlateLog(auth_token, std::move(jReq.toStyledString()));

                    if (success)
                    {
                        std::cout << std::endl
                                  << "\033[33m  ** The plate has been sent to the server. **    \033[0m" << std::endl;
                    }
                    else
                    {
                        std::cerr << std::endl
                                  << "\033[33m  ** Error: The plate hasn't been sent to the server! **    \033[0m" << std::endl;
                    }
                    /*END UPLOAD TO WEB API PROCESS*/
                }
                else
                {
                    std::cerr << "Authentication failed.\n";
                    std::cerr << "WebAPI: The Auth Token is hasn't been gotten from server.\n";
                }
            }
        }
        else
        {
            switch (detechStatus)
            {
            case RecognizerTool::DetectStatus::SettingsNotYetApplied:
                std::cerr << "RecognizerTool settings not yet applied!\n";
                break;
            case RecognizerTool::DetectStatus::FrameIsEmpty:
                std::cerr << "Camera frame is empty!\n";
                break;
                /*case RecognizerTool::DetectStatus::PlateWasNotFound:
                    std::cout << "Plate was not found!\n";
                    break;*/
            }
        }
    }
    catch (std::exception e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

void Worker::dowork2()
{
    WorkerTaskData data = Worker::workerTasks[m_index].m_data;

    try
    {
        std::string vehicleImageBase64;

        {
            std::vector<uchar> buf;
            cv::imencode(".jpg", data.frame, buf);
            auto base64_png = reinterpret_cast<const unsigned char *>(buf.data());
            vehicleImageBase64 = std::move("image/jpeg;base64," + Helper::base64_encode(base64_png, buf.size()));
        }

        Json::Value jValue;
        bool ok = PlateRecognizerComAPI::sendRequest(jValue, "d7b64e1b832ce81609fe3048f35f74cc74d92c20", vehicleImageBase64);

        if (!ok)
        {
            std::cout << "No plate found" << std::endl;
            return;
        }

        const Json::Value results = jValue["results"];

        for (int index = 0; index < results.size(); ++index)
        {
            std::string plateStr = results[index]["plate"].asString();

            int x, y, width, height;

            x = results[index]["box"]["xmin"].asInt();
            y = results[index]["box"]["ymin"].asInt();
            width = results[index]["box"]["xmax"].asInt() - results[index]["box"]["xmin"].asInt();
            height = results[index]["box"]["ymax"].asInt() - results[index]["box"]["ymin"].asInt();

            cv::Mat imgPlate = data.frame(cv::Rect(x, y, width, height)).clone();

            std::string auth_token;

            if (WebAPI::getAuthToken(auth_token))
            {
                std::string numberPlateImageBase64;
                /*BEGIN UPLOAD TO WEB API PROCESS*/

                {
                    std::vector<uchar> buf;
                    cv::imencode(".jpg", imgPlate, buf);
                    auto base64_png = reinterpret_cast<const unsigned char *>(buf.data());
                    numberPlateImageBase64 = std::move("image/jpeg;base64," + Helper::base64_encode(base64_png, buf.size()));
                }

                auto dt = std::chrono::time_point_cast<std::chrono::milliseconds>(
                    	std::chrono::system_clock::now()
                    );

                std::string station_id = "";

                uint16_t logtype = 2;
                    
                    if (data.camera_id == 1) {
						logtype = 1;
                        station_id = "28038"; // giris id
                    } else if (data.camera_id == 2) {
                        station_id = "28039"; // cikis id
						logtype = 2;
                    }else logtype = 2; //defult
                    
                 Json::Value jReq;
                jReq["carAccessDateTime"] = std::to_string((int)(dt.time_since_epoch().count() / 1000) - 3600); //bir saat geri almak için 3600 değeri alındı
                jReq["systemTime"] = std::to_string((int)(dt.time_since_epoch().count() / 1000)-3600);
                jReq["alprsLogType"] = unsigned(logtype);
                jReq["vehicleImageFileName"] = "img7.jpeg";
                jReq["numberPlateImageFileName"] = "img7-plate.jpeg";
                jReq["carRegistrationNumber"] = plateStr;
                jReq["numberPlateCharacters"] = plateStr;
                jReq["carDetailId"] = unsigned(1);
                jReq["alprsSystemId"] = "";
                jReq["alprsStationId"] = station_id;
                jReq["alprsSystemSerialNumber"] = "";
                jReq["username"] = "alpdade";
                jReq["password"] = "alpdade";
                jReq["geographicalAreaNameCode"] = "AU";
                jReq["vehicleMake"] = ""; //"Porsche"
                jReq["vehicleModel"] = ""; //"cayenne";
                jReq["alprsContrastValue"] = "60";
                jReq["numberPlateRecognitionScore"] = "90";
                jReq["vehicleImageFile"] = vehicleImageBase64;
                jReq["numberPlateImageFile"] = numberPlateImageBase64;
                jReq["alprsFK"] = "string";
                jReq["bookingId"] = unsigned(1);

                // std::cout << "JSON stringify: " << jReq.toStyledString() << std::endl;
                bool success = WebAPI::uploadPlateLog(auth_token, std::move(jReq.toStyledString()));

                if (success)
                {
                    PlateWorkerTask task {};
                    task.founded_plate = plateStr;
                    task.gate_id = data.camera_id;
                    PlateWorker::addTask(task);
                    
                    std::cout << std::endl
                              << "\033[33m  ** The plate has been sent to the server. **    \033[0m" << std::endl;
                }
                else
                {
                    std::cerr << std::endl
                              << "\033[33m  ** Error: The plate hasn't been sent to the server! **    \033[0m" << std::endl;
                }
                /*END UPLOAD TO WEB API PROCESS*/
            }
            else
            {
                std::cerr << "Authentication failed.\n";
                std::cerr << "WebAPI: The Auth Token is hasn't been gotten from server.\n";
            }
        }
    }
    catch (std::exception e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}
