#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <unistd.h>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <chrono>
#include "helper/definitions.h"
#include "helper/Helper.h"
#include "platerecognizer/recognizertool.h"
#include "core/Worker.h"
#include "core/PlateWorker.h"
#include "cardetection/cardetection.h"

using namespace cv;
using namespace std;

std::string APP_RUN_PATH = "/";
bool USE_VIDEO_FILE = false;
std::string SOURCE_VIDEO_PATH = "";
bool EXPORT_VEHICLE_IMAGE = true;
std::string camera_1_url = "";
std::string camera_2_url = "";
bool should_exit_threads = false;
std::vector<std::string> plate_list;

bool loadSettings() {
    // Load plate list
    std::string plate_list_path = APP_RUN_PATH + "alpr_config/plate_list.txt";
    if (!std::filesystem::is_regular_file(plate_list_path)) {
        std::cerr << "plate_list.txt was not found!" << std::endl;
        return false;
    }
    ifstream file(plate_list_path);
    std::string line;
    while (getline(file, line)) {
        plate_list.push_back(line);
    }
    file.close();
    if (plate_list.empty()) {
        std::cerr << "Plate list is empty" << std::endl;
        return false;
    }

    // Load camera settings and search areas
    if (!loadCameraSettings(1) || !loadCameraSettings(2)) {
        return false;
    }

    return true;
}

bool loadCameraSettings(int camera_id) {
    std::vector<cv::Point> searchAreaPolygon;
    std::string search_area_file_path = APP_RUN_PATH + "alpr_config/camera_" + std::to_string(camera_id) + "_search_area.txt";
    if (!std::filesystem::is_regular_file(search_area_file_path)) {
        std::cerr << "search_area.txt was not found!" << std::endl;
        return false;
    }
    ifstream file(search_area_file_path);
    std::string line;
    while (getline(file, line)) {
        std::vector<std::string> tmp = Helper::split(line, ',');
        if (tmp.size() == 2) {
            searchAreaPolygon.push_back(cv::Point(std::stoi(tmp[0]), std::stoi(tmp[1])));
        }
    }
    file.close();
    if (searchAreaPolygon.size() < 4) {
        std::cerr << "Search Area is not valid!" << std::endl;
        return false;
    }
    if (camera_id == 1) {
        CarDetection::CAMERA_1_SEARCH_AREA_POLYGON = searchAreaPolygon;
    } else {
        CarDetection::CAMERA_2_SEARCH_AREA_POLYGON = searchAreaPolygon;
    }

    std::string settings_file_path = APP_RUN_PATH + "alpr_config/camera_" + std::to_string(camera_id) + "_settings.txt";
    if (!std::filesystem::is_regular_file(settings_file_path)) {
        std::cerr << "settings.txt was not found!" << std::endl;
        return false;
    }
    file.open(settings_file_path);
    while (getline(file, line)) {
        std::vector<std::string> tmp = Helper::split(line, '=');
        if (tmp.size() == 2) {
            if (tmp[0] == "horizontal_line_position") {
                if (camera_id == 1) {
                    CarDetection::CAMERA_1_HORIZONTAL_LINE_POSITION = std::stod(tmp[1]);
                } else {
                    CarDetection::CAMERA_2_HORIZONTAL_LINE_POSITION = std::stod(tmp[1]);
                }
            } else if (tmp[0] == "vertical_motion_direction") {
                if (camera_id == 1) {
                    CarDetection::CAMERA_1_VERTICAL_MOTION_DIRECTON = CarDetection::VerticalMotionDirection(std::stoi(tmp[1]));
                } else {
                    CarDetection::CAMERA_2_VERTICAL_MOTION_DIRECTON = CarDetection::VerticalMotionDirection(std::stoi(tmp[1]));
                }
            } else if (tmp[0] == "min_car_area") {
                if (camera_id == 1) {
                    CarDetection::CAMERA_1_MIN_CAR_AREA = std::stoi(tmp[1]);
                } else {
                    CarDetection::CAMERA_2_MIN_CAR_AREA = std::stoi(tmp[1]);
                }
            } else if (tmp[0] == "export_vehicle_image") {
                EXPORT_VEHICLE_IMAGE = !!std::stoi(tmp[1]);
            } else if (tmp[0] == "camera_url") {
                if (camera_id == 1) {
                    camera_1_url = tmp[1];
                } else {
                    camera_2_url = tmp[1];
                }
            }
        }
    }
    file.close();
    return true;
}

void cameraProcess(int camera_id, const std::string &camera_url, const std::vector<cv::Point> &search_area_polygon) {
    if (!RecognizerTool::applySettings()) {
        std::cerr << "RecognizerTool settings could not be applied!" << std::endl;
        return;
    }

    cv::VideoCapture cameraCapture;
    cameraCapture.open(camera_url);

    if (!cameraCapture.isOpened()) {
        std::cerr << "Error reading video stream" << std::endl;
        return;
    }

    cv::Mat imgFrame1, imgFrame2;
    cameraCapture.read(imgFrame1);
    cameraCapture.read(imgFrame2);

    int intHorizontalLinePosition = (int)std::round((double)imgFrame1.rows * (camera_id == 1 ? CarDetection::CAMERA_1_HORIZONTAL_LINE_POSITION : CarDetection::CAMERA_2_HORIZONTAL_LINE_POSITION));
    cv::Point crossingLine[2] = {cv::Point(0, intHorizontalLinePosition), cv::Point(imgFrame1.cols - 1, intHorizontalLinePosition)};
    std::vector<Blob> blobs;

    bool blnFirstFrame = true;
    int frameCount = 2;
    char chCheckForEscKey = 0;

    while (cameraCapture.isOpened() && chCheckForEscKey != 27 && !should_exit_threads) {
        std::vector<Blob> currentFrameBlobs;
        cv::Mat imgFrame1Copy = imgFrame1.clone();
        cv::Mat imgFrame2Copy = imgFrame2.clone();

        cv::Mat imgDifference, imgThresh;
        cv::cvtColor(imgFrame1Copy, imgFrame1Copy, COLOR_BGR2GRAY);
        cv::cvtColor(imgFrame2Copy, imgFrame2Copy, COLOR_BGR2GRAY);
        cv::GaussianBlur(imgFrame1Copy, imgFrame1Copy, cv::Size(5, 5), 0);
        cv::GaussianBlur(imgFrame2Copy, imgFrame2Copy, cv::Size(5, 5), 0);
        cv::absdiff(imgFrame1Copy, imgFrame2Copy, imgDifference);
        cv::threshold(imgDifference, imgThresh, 30, 255.0, CV_THRESH_BINARY);

        for (int i = 0; i < 2; i++) {
            cv::dilate(imgThresh, imgThresh, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
            cv::erode(imgThresh, imgThresh, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(imgThresh.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (auto &contour : contours) {
            cv::convexHull(contour, contour);
            Blob possibleBlob(contour);
            if (CarDetection::checkIfBlobInsideSearchPolygon(possibleBlob.currentBoundingRect, search_area_polygon)) {
                currentFrameBlobs.push_back(possibleBlob);
            }
        }

        if (blnFirstFrame) {
            blobs = currentFrameBlobs;
        } else {
            CarDetection::matchCurrentFrameBlobsToExistingBlobs(blobs, currentFrameBlobs);
        }

        for (auto &blob : blobs) {
            if (blob.blnStillBeingTracked && blob.isAboveTheLine && !blob.isProcessStarted) {
                Worker::addTask(WorkerTaskData(blob.uuid, imgFrame1(blob.currentBoundingRect).clone(), camera_id));
                blob.isProcessStarted = true;
            }
        }

        imgFrame1 = imgFrame2.clone();
        cameraCapture.read(imgFrame2);
        blnFirstFrame = false;
        frameCount++;
        chCheckForEscKey = cv::waitKey(1);
    }

    should_exit_threads = true;
    cv::destroyAllWindows();
}

void signalHandler(int signum) {
    std::cerr << "Interrupt signal (" << signum << ") received.\n";
    should_exit_threads = true;
    Worker::stopAll();
    exit(signum);
}

void applySettings() {
    if (!std::filesystem::is_directory(APP_RUN_PATH + "car_img_plate")) {
        if (!std::filesystem::create_directories(APP_RUN_PATH + "car_img_plate")) {
            std::cerr << "Failed to create car_img_plate directory\n";
        }
    }
    if (!std::filesystem::is_directory(APP_RUN_PATH + "car_img_plate_detected")) {
        if (!std::filesystem::create_directories(APP_RUN_PATH + "car_img_plate_detected")) {
            std::cerr << "Failed to create car_img_plate_detected directory\n";
        }
    }
}

void multipleCameraProcess() {
    std::thread cam_1_thread(cameraProcess, 1, camera_1_url, CarDetection::CAMERA_1_SEARCH_AREA_POLYGON);
    std::thread cam_2_thread(cameraProcess, 2, camera_2_url, CarDetection::CAMERA_2_SEARCH_AREA_POLYGON);
    cam_1_thread.join();
    cam_2_thread.join();
}

int main(int argc, char **argv) {
    std::string full_app_path = argv[0];
    std::vector<std::string> pathItems = Helper::split(argv[0], '/');
    for (size_t i = 0; i < pathItems.size() - 1; i++) {
        APP_RUN_PATH += pathItems[i] + "/";
    }

    if (argc >= 2 && std::string(argv[1]) == "--video" && argc >= 3) {
        USE_VIDEO_FILE = true;
        SOURCE_VIDEO_PATH = argv[2];
    }

    applySettings();
    signal(SIGINT, signalHandler);

    if (!loadSettings()) {
        std::cerr << "Failed to load settings!" << std::endl;
        return 2;
    }

    PlateWorker::setRef(new PlateWorker("/dev/ttyUSB0", plate_list));
    multipleCameraProcess();
    return 0;
}
