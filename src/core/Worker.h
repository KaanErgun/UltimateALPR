#ifndef BASE_WORKER_H
#define BASE_WORKER_H

#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <string>
#include <opencv2/opencv.hpp>

enum WorkerProcessStatus
{
    WaitingNewTask = 1,
    AssignedNewTask,
    WorkingOnTask,
};

class WorkerTaskData {
public:
    WorkerTaskData() {}
    WorkerTaskData(std::string _uuid, cv::Mat _frame, int camera_id);
    std::string uuid;
    cv::Mat frame;
    int camera_id;
};

class WorkerState
{
public:
    explicit WorkerState(WorkerProcessStatus status);
    explicit WorkerState(WorkerProcessStatus status, WorkerTaskData taskData);

    WorkerProcessStatus status() const { return m_status; }
    WorkerTaskData taskData() const { return m_data; }

private:
    WorkerProcessStatus m_status;
    WorkerTaskData m_data;

    friend class Worker;
};

class Worker
{
    typedef std::vector<std::unique_ptr<Worker>> WorkerList;
public:
    explicit Worker(int index);
    ~Worker();
    
    static void createAndStartWorkers(int size);
    
    /// @brief Stop all worker threads 
    static void stopAll();

    /// @brief Add a task for first thread who are in the wait mode.
    /// Try until assign a thread.
    static void addTask(WorkerTaskData data);

    static void imageWrite(const cv::Mat &image, const std::string path);
    

    /// @brief Start the thread of the worker.
    /// When starts a thread, If it's have not any task, waits a new task. 
    void start();

    /// @brief Stop the thread of the worker.
    void stop();

    /// @brief Get index of worker
    /// @return int
    int index() const { return m_index; }

    /// m_mutext and m_convariable defined for notifiy tasks to the workers;
    std::mutex notify_mutex;
    std::condition_variable notify_convariable;

private:
    /// @brief The thread of the worker.
    /// When you add a task to worker, the process will be run in this thread.
    std::unique_ptr<std::thread> m_thread;
    
    /// @brief When set to true, the thread of worker will be stopped.
    /// On the normally, the thread waits a new task.
    /// That means the thread alive everytime after start the worker.
    std::atomic<bool> m_shouldStop = false;
    
    /// @brief The index of thread.
    /// All threads must be have an index of different the other thread.
    /// Thread uses the m_index for access and modify the own WorkerTask.
    std::atomic<int> m_index;

    /// @brief The `workers` is list of Workers
    /// When the createWorkers called, a new worker will be created in `workers`
    static Worker::WorkerList workers;
    
    /// @brief workerTasks stores the list of WorkerTask.
    /// The Worker thread will be access the own task with own index.
    /// For example, you want to run the last worker on the workers list,
    ///  you have to set task to last item of the `WorkerTask` list.
    static std::vector<WorkerState> workerTasks;


    void startWorking();
    void stopWorking();
    void process();
    void dowork();
    void dowork2();

};

#endif
