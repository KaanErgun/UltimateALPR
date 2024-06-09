#ifndef PLATE_WORKER_H
#define PLATE_WORKER_H

#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <string>

class PlateWorkerTask {
public:
    PlateWorkerTask() {}

    std::string founded_plate;
    int gate_id;
};

class PlateWorker
{

public:
    explicit PlateWorker(std::string gate_serialport, std::vector<std::string> plates);
    ~PlateWorker();

    /// @brief Stop the thread of the worker.
    static void stop();

    static void addTask(const PlateWorkerTask task);
    static void setRef(PlateWorker *worker);
    // static PlateWorker* getRef();

    std::mutex notify_mutex;
    std::condition_variable notify_convariable;

    std::string gate_serialport_name;
    std::vector<std::string> plate_list;

private:
    std::unique_ptr<std::thread> m_thread;
    std::atomic<bool> m_shouldStop = false;
    static PlateWorker *m_workerRef;

    std::mutex m_taskListMutex;
    std::vector<PlateWorkerTask> m_tasks;

    void process(); // thread loop is here
    void dowork(const PlateWorkerTask &task);

    void write_log(std::string msg);
};

#endif // PLATE_WORKER
