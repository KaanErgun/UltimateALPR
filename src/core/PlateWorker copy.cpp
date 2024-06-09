#include "PlateWorker.h"
#include <iostream>

#include <stdio.h>
#include <fcntl.h>     /* File Control Definitions           */
#include <termios.h>   /* POSIX Terminal Control Definitions */
#include <unistd.h>    /* UNIX Standard Definitions          */
#include <errno.h>     /* ERROR Number Definitions           */
#include <sys/ioctl.h> /* ioctl()*/
#include <chrono>
#include <algorithm>
#include <fstream>
#include <ctime>


PlateWorker *PlateWorker::m_workerRef = nullptr;

PlateWorker::PlateWorker(std::string gate_serialport, std::vector<std::string> plates)
{
    gate_serialport_name = gate_serialport;
    plate_list = plates;
    m_shouldStop = false;
    m_thread.reset(new std::thread(&PlateWorker::process, this));

    write_log("Starting to log!");
}

PlateWorker::~PlateWorker()
{
    m_shouldStop = true;
}

void PlateWorker::stop()
{
    m_workerRef->m_shouldStop = true;
}

void PlateWorker::setRef(PlateWorker *worker)
{
    m_workerRef = worker;
}

/*PlateWorker* PlateWorker::getRef()
{
    return m_workerRef;
}*/

void PlateWorker::addTask(const PlateWorkerTask task)
{
    std::unique_lock<std::mutex> lock(m_workerRef->notify_mutex);
    m_workerRef->m_tasks.push_back(task);
    m_workerRef->notify_convariable.notify_all();
}

void PlateWorker::process()
{
    while (!m_shouldStop)
    {

        std::unique_lock<std::mutex> lock(notify_mutex);
        notify_convariable.wait(lock);

        for (int i = 0; i < m_tasks.size(); i++)
        {
            dowork(m_tasks[i]);
        }

        m_tasks.erase(m_tasks.begin());
        // PROCESS FIRST TASK
    }
}

void PlateWorker::write_log(std::string msg)
{
    try {
        
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer,sizeof(buffer),"%Y-%m-%d %H:%M:%S",timeinfo);
        std::string current(buffer);

        std::ofstream outfile;

        outfile.open("car_gate_status_log.txt", std::ios_base::app); // append instead of overwrite

        outfile << current;
        outfile << "--> ";
        outfile << msg;
        outfile << std::endl;
        outfile.close();
    } catch(std::exception e) {
        std::cout << "File log write error!" << std::endl;
    }
}

void PlateWorker::dowork(const PlateWorkerTask &task)
{

    bool isFound = false;

    for (int i = 0; i < plate_list.size(); i++)
    {
        if (task.founded_plate.find(plate_list.at(i)) != std::string::npos)
        {
            isFound = true;
            break;
        }
    }

    if (!isFound && std::find(plate_list.begin(), plate_list.end(), task.founded_plate) == plate_list.end())
    {
        std::cout << "The plate \"" << task.founded_plate << "\" doesn't find in plate list. This is the unkown number plate" << std::endl;
        write_log("The plate \"" + task.founded_plate + "\" doesn't find in plate list. This is the unkown number plate");
        return;
    }

    /// OPEN GATE
    std::cout << "Gate 1 is opening for the number plate -> \"" << task.founded_plate << "\"" << std::endl;
    write_log("Gate 1 is opening for the number plate -> \"" + task.founded_plate + "\"");

    int fd, sercmd, serstat;

    sercmd = TIOCM_RTS;
    fd = open(this->gate_serialport_name.c_str(), O_RDONLY); // Open the serial port.

    printf("Setting the RTS pin.\n");
    ioctl(fd, TIOCMBIS, &sercmd); // Set the RTS pin.

    // Read the RTS pin status.
    ioctl(fd, TIOCMGET, &serstat);
    if (serstat & TIOCM_RTS)
        printf("RTS pin is set.\n");
    else
        printf("RTS pin is reset.\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    printf("Resetting the RTS pin.\n");
    ioctl(fd, TIOCMBIC, &sercmd); // Reset the RTS pin.

    // Read the RTS pin status.
    ioctl(fd, TIOCMGET, &serstat);
    if (serstat & TIOCM_RTS)
        printf("RTS pin is set.\n");
    else
        printf("RTS pin is reset.\n");

    close(fd);
}