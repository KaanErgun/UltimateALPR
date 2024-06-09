#ifndef PLATEWORKER_H
#define PLATEWORKER_H

#include <atomic>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>

struct PlateWorkerTask {
    std::string founded_plate;
};

class PlateWorker {
public:
    PlateWorker(std::string gate_serialport, std::vector<std::string> plates);
    ~PlateWorker();

    static void setRef(PlateWorker *worker);
    static void addTask(const PlateWorkerTask task);

    void stop();

    // Kopyalama işlemlerini sil
    PlateWorker(const PlateWorker&) = delete;
    PlateWorker& operator=(const PlateWorker&) = delete;

private:
    static PlateWorker *m_workerRef;
    std::string gate_serialport_name;
    std::vector<std::string> plate_list;
    std::atomic<bool> m_shouldStop;
    std::unique_ptr<std::thread> m_thread;
    std::mutex notify_mutex;
    std::condition_variable notify_convariable;
    std::vector<PlateWorkerTask> m_tasks;

    void process();
    void write_log(std::string msg);
    void dowork(const PlateWorkerTask &task);
    void gate_open();
};

#endif // PLATEWORKER_H
