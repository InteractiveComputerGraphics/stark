#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <omp.h>
#include <fmt/format.h>

namespace symx 
{
    class Log 
    {
    public:
        struct Timer {
            double start_time = 0.0;
            double total_time = 0.0;
            int count = 0;
        };

        void start_timing(const std::string& label) {
            if (timers.find(label) == timers.end()) {
                timers[label] = Timer();
                insertion_order.push_back(label);
            }
            timers[label].start_time = omp_get_wtime();
        }

        void end_timing(const std::string& label) {
            auto it = timers.find(label);
            if (it != timers.end()) {
                double end = omp_get_wtime();
                it->second.total_time += end - it->second.start_time;
                it->second.count++;
            }
        }

        struct ScopedTimer {
            Log& log;
            std::string label;
            ScopedTimer(Log& log, std::string label) : log(log), label(std::move(label)) {
                log.start_timing(this->label);
            }
            ~ScopedTimer() {
                log.end_timing(this->label);
            }
        };

        ScopedTimer scope(const std::string& label) {
            return ScopedTimer(*this, label);
        }

        void print() const {
            std::cout << "\n=== Timing Report ===\n";
            std::cout << fmt::format("{:<35} | {:<12} | {:<8}\n", "Label", "Time (s)", "Count");
            std::cout << std::string(60, '-') << "\n";
            for (const auto& label : insertion_order) {
                const auto& timer = timers.at(label);
                std::cout << fmt::format("{:<35} | {:<12.6f} | {:<8}\n", label, timer.total_time, timer.count);
            }
            std::cout << std::string(60, '-') << "\n";
        }

        const std::unordered_map<std::string, Timer>& get_timers() const {
            return timers;
        }

    private:
        std::unordered_map<std::string, Timer> timers;
        std::vector<std::string> insertion_order;
    };
}
