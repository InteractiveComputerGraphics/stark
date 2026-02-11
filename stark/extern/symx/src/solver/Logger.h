#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <limits>
#include <memory>

namespace symx
{
    class Logger
    {
    public:
        // =============================================================
        // Scoped Timer — RAII timing API
        // =============================================================
        // Usage: { auto t = logger.time("CG"); ... } // Accumulated automatically
        struct ScopedTimer {
            Logger& logger;
            std::string label;
            double start;

            ScopedTimer(Logger& l, const std::string& label);
            ~ScopedTimer();
            ScopedTimer(ScopedTimer&& other) noexcept;
            ScopedTimer(const ScopedTimer&) = delete;
            ScopedTimer& operator=(const ScopedTimer&) = delete;
            ScopedTimer& operator=(ScopedTimer&&) = delete;
        };
        [[nodiscard]] ScopedTimer time(const std::string& label);

        // Manual start/stop timing
        void start_timing(const std::string& label);
        void stop_timing(const std::string& label);

        // =============================================================
        // Typed Series
        // =============================================================
        void append(const std::string& label, double v);
        void append(const std::string& label, int v);
        void append(const std::string& label, const std::string& v);

        // =============================================================
        // Accumulators
        // =============================================================
        void set(const std::string& label, double v);
        void set(const std::string& label, int v);
        void add(const std::string& label, double v);
        void add(const std::string& label, int v);

        // =============================================================
        // Add and Append
        // =============================================================
        void add_and_append(const std::string& label, double v);
        void add_and_append(const std::string& label, int v);

        // =============================================================
        // Getters
        // =============================================================
        double get_double(const std::string& label) const;
        int get_int(const std::string& label) const;
        const std::vector<double>& get_double_series(const std::string& label) const;
        const std::vector<int>& get_int_series(const std::string& label) const;
        const std::vector<std::string>& get_string_series(const std::string& label) const;

        // =============================================================
        // Timer Queries
        // =============================================================
        double get_timer_total(const std::string& label) const;
        int get_timer_count(const std::string& label) const;
        double get_timer_avg(const std::string& label) const;
        double get_timer_min(const std::string& label) const;
        double get_timer_max(const std::string& label) const;
        const std::vector<std::string>& get_timer_labels() const;

        // =============================================================
        // Statistics — "TOTAL | AVG | [MIN, MAX]" for double and int series
        // =============================================================
        std::string get_statistics(const std::string& label) const;

        // =============================================================
        // Persistence
        // =============================================================
        void set_path(const std::string& path);
        std::string get_path() const;
        void save_to_disk(const std::string& path) const;
        void save_to_disk() const;

        // =============================================================
        // Control
        // =============================================================
        void clear();

    private:

        // Timing
        struct Timer {
            double start = 0.0;
            double total = 0.0;
            int count = 0;
            double min = std::numeric_limits<double>::max();
            double max = 0.0;
        };
        std::unordered_map<std::string, Timer> timers_;
        std::vector<std::string> timer_insertion_order_;

        // Series
        std::unordered_map<std::string, std::vector<double>> series_double_;
        std::unordered_map<std::string, std::vector<int>> series_int_;
        std::unordered_map<std::string, std::vector<std::string>> series_string_;

        // Accumulators
        std::unordered_map<std::string, double> acc_double_;
        std::unordered_map<std::string, int> acc_int_;

        // Persistence path
        std::string path_;

        // Empty vectors for safe reference returns
        static const std::vector<double> empty_double_series_;
        static const std::vector<int> empty_int_series_;
        static const std::vector<std::string> empty_string_series_;

        // Internal
        Timer& _get_or_create_timer(const std::string& label);
        void _record_elapsed(const std::string& label, double elapsed);
    };
    using spLogger = std::shared_ptr<Logger>;
}
