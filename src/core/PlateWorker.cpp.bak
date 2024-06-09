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


#ifdef __cplusplus
extern "C"
{
#endif

#include <libusbrelay.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/*
Renk kodları aşağıdaki gibidir:


31 - Kırmızı
32 - Yeşil
33 - Sarı
34 - Mor
35 - Pembe
36 - Mavi
37 - Gri

echo -e "\033[31mMakdos Blog\033[0m"
Yukarıdaki komutun açıklamaları:


\033[       :octal \033   ,  ESC karakterine karşılık gelir.
31           :kırmızı renk kodu
m            :graphical modu uygulama
\033[0m  :graphical modu resetleme
*/
void gate_open()
{
	enumerate_relay_boards(getenv("USBID"), 1, 1);
	relay_board *board = find_board("BITFT", 1);
	if (board) {
	//	printf("HID Serial: %s ", board->serial);
		printf("\033[32mThe door is opening!\033[0m\n");
		operate_relay((const char *)"BITFT", 1, CMD_ON, 1);
		//std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		sleep(5);
		operate_relay((const char *)"BITFT", 1, CMD_OFF, 1);
		printf("\033[31mThe door is closed!\033[0m\n");
	}
}

#else

void gate_open()
{
	printf("Only C++\n");
}

#endif

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

char cmd_set_relay_high[3] = {0xFF, 0x01, 0x01};
char cmd_set_relay_low[3] = {0xFF, 0x01, 0x00};

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
    
    gate_open();
/*
    int fd, sercmd, serstat;

    struct termios options;

    sercmd = TIOCM_RTS;
    fd = open(this->gate_serialport_name.c_str(), O_WRONLY); // Open the serial port.

    tcgetattr(fd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    tcsetattr(fd, TCSANOW,&options);
    
    write(fd, cmd_set_relay_high, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write(fd, cmd_set_relay_high, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write(fd, cmd_set_relay_high, 3);

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    write(fd, cmd_set_relay_low, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write(fd, cmd_set_relay_low, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    write(fd, cmd_set_relay_low, 3);

    close(fd);
*/
}
