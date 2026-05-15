#pragma once
#include <vector>
#include <functional>
#include <mutex>
#include <omp.h>

namespace symx 
{
    class DeferredParallelTasks 
    {
    private:
        std::vector<std::function<void()>> tasks;
        std::mutex mtx;

    public:
        void add(std::function<void()> task)
        {
            std::lock_guard<std::mutex> lock(mtx);
            tasks.push_back(task);
        }

        void run(int n_threads)
        {
            const int n_tasks = (int)tasks.size();
            #pragma omp parallel for num_threads(n_threads) schedule(dynamic)
            for (int i = 0; i < n_tasks; ++i) {
                tasks[i]();
            }
            tasks.clear();
        }
    };
}
