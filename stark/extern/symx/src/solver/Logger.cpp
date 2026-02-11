#include "Logger.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <omp.h>
#include <fmt/format.h>

using namespace symx;

// Static empty vectors
const std::vector<double> Logger::empty_double_series_;
const std::vector<int> Logger::empty_int_series_;
const std::vector<std::string> Logger::empty_string_series_;

// =============================================================
// ScopedTimer
// =============================================================
Logger::ScopedTimer::ScopedTimer(Logger& l, const std::string& label)
    : logger(l), label(label), start(omp_get_wtime())
{
    // Ensure the timer entry exists (for insertion order tracking)
    l._get_or_create_timer(label);
}

Logger::ScopedTimer::~ScopedTimer()
{
    if (!label.empty()) {
        double elapsed = omp_get_wtime() - start;
        logger._record_elapsed(label, elapsed);
    }
}

Logger::ScopedTimer::ScopedTimer(ScopedTimer&& other) noexcept
    : logger(other.logger), label(std::move(other.label)), start(other.start)
{
    other.label.clear();  // Prevent dtor from recording
}

Logger::ScopedTimer Logger::time(const std::string& label)
{
    return ScopedTimer(*this, label);
}

// =============================================================
// Manual start/stop timing
// =============================================================
void Logger::start_timing(const std::string& label)
{
    if (!enabled_) return;
    auto& timer = _get_or_create_timer(label);
    timer.start = omp_get_wtime();
}

void Logger::stop_timing(const std::string& label)
{
    if (!enabled_) return;
    auto it = timers_.find(label);
    if (it == timers_.end()) {
        std::cout << "symx::Logger::stop_timing error: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    double elapsed = omp_get_wtime() - it->second.start;
    _record_elapsed(label, elapsed);
}

// =============================================================
// Typed Series
// =============================================================
void Logger::append(const std::string& label, double v)
{
    if (!enabled_) return;
    series_double_[label].push_back(v);
}

void Logger::append(const std::string& label, int v)
{
    if (!enabled_) return;
    series_int_[label].push_back(v);
}

void Logger::append(const std::string& label, const std::string& v)
{
    if (!enabled_) return;
    series_string_[label].push_back(v);
}

// =============================================================
// Accumulators
// =============================================================
void Logger::set(const std::string& label, double v)
{
    if (!enabled_) return;
    acc_double_[label] = v;
}

void Logger::set(const std::string& label, int v)
{
    if (!enabled_) return;
    acc_int_[label] = v;
}

void Logger::add(const std::string& label, double v)
{
    if (!enabled_) return;
    acc_double_[label] += v;
}

void Logger::add(const std::string& label, int v)
{
    if (!enabled_) return;
    acc_int_[label] += v;
}

void Logger::add_and_append(const std::string &label, double v)
{
    if (!enabled_) return;
    add(label, v);
    append(label, v);
}

void Logger::add_and_append(const std::string &label, int v)
{
    if (!enabled_) return;
    add(label, v);
    append(label, v);
}

// =============================================================
// Getters
// =============================================================
double Logger::get_double(const std::string& label) const
{
    auto it = acc_double_.find(label);
    if (it == acc_double_.end()) {
        std::cout << "symx::Logger::get_double warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    return it->second;
}

int Logger::get_int(const std::string& label) const
{
    auto it = acc_int_.find(label);
    if (it == acc_int_.end()) {
        std::cout << "symx::Logger::get_int warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    return it->second;
}

const std::vector<double>& Logger::get_double_series(const std::string& label) const
{
    auto it = series_double_.find(label);
    if (it == series_double_.end()) {
        std::cout << "symx::Logger::get_double_series warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    return it->second;
}

const std::vector<int>& Logger::get_int_series(const std::string& label) const
{
    auto it = series_int_.find(label);
    if (it == series_int_.end()) {
        std::cout << "symx::Logger::get_int_series warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    return it->second;
}

const std::vector<std::string>& Logger::get_string_series(const std::string& label) const
{
    auto it = series_string_.find(label);
    if (it == series_string_.find(label)) {
        std::cout << "symx::Logger::get_string_series warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    return it->second;
}

// =============================================================
// Timer Queries
// =============================================================
double Logger::get_timer_total(const std::string& label) const
{
    auto it = timers_.find(label);
    if (it == timers_.end()) {
        std::cout << "symx::Logger::get_timer_total warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    return it->second.total;
}

int Logger::get_timer_count(const std::string& label) const
{
    auto it = timers_.find(label);
    if (it == timers_.end()) {
        std::cout << "symx::Logger::get_timer_count warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    return it->second.count;
}

double Logger::get_timer_avg(const std::string& label) const
{
    auto it = timers_.find(label);
    if (it == timers_.end()) {
        std::cout << "symx::Logger::get_timer_avg warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    if (it->second.count == 0) return 0.0;
    return it->second.total / it->second.count;
}

double Logger::get_timer_min(const std::string& label) const
{
    auto it = timers_.find(label);
    if (it == timers_.end()) {
        std::cout << "symx::Logger::get_timer_min warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    if (it->second.count == 0) return 0.0;
    return it->second.min;
}

double Logger::get_timer_max(const std::string& label) const
{
    auto it = timers_.find(label);
    if (it == timers_.end()) {
        std::cout << "symx::Logger::get_timer_max warning: Label '" << label << "' not found." << std::endl;
        exit(-1);
    }
    if (it->second.count == 0) return 0.0;
    return it->second.max;
}

const std::vector<std::string>& Logger::get_timer_labels() const
{
    return timer_insertion_order_;
}

// =============================================================
// Statistics
// =============================================================
std::string Logger::get_statistics(const std::string& label) const
{
    // Check double series first
    {
        auto it = series_double_.find(label);
        if (it != series_double_.end() && !it->second.empty()) {
            const auto& data = it->second;
            double sum = 0.0, mn = data[0], mx = data[0];
            for (double v : data) {
                sum += v;
                mn = std::min(mn, v);
                mx = std::max(mx, v);
            }
            double avg = sum / data.size();
            return fmt::format("{:.6f} total | {:.6f} avg | [{:.6f}, {:.6f}]", sum, avg, mn, mx);
        }
    }

    // Check int series
    {
        auto it = series_int_.find(label);
        if (it != series_int_.end() && !it->second.empty()) {
            const auto& data = it->second;
            long long sum = 0;
            int mn = data[0], mx = data[0];
            for (int v : data) {
                sum += v;
                mn = std::min(mn, v);
                mx = std::max(mx, v);
            }
            double avg = (double)sum / data.size();
            return fmt::format("{} total | {:.1f} avg | [{}, {}]", sum, avg, mn, mx);
        }
    }

    // Check timer
    {
        auto it = timers_.find(label);
        if (it != timers_.end() && it->second.count > 0) {
            const auto& t = it->second;
            double avg = t.total / t.count;
            return fmt::format("{:.6f} total | {:.6f} avg | [{:.6f}, {:.6f}]", t.total, avg, t.min, t.max);
        }
    }

    return "(no data)";
}

// =============================================================
// Persistence
// =============================================================
void Logger::set_path(const std::string& path)
{
    path_ = path;
}

std::string Logger::get_path() const
{
    return path_;
}

void Logger::save_to_disk(const std::string& path) const
{
    std::ofstream out(path);
    if (!out) {
        std::cout << "symx::Logger::save_to_disk error: Cannot open file " << path << std::endl;
        exit(-1);
    }

    // Helper: sort keys from an unordered_map
    auto sorted_keys = [](const auto& map) {
        std::vector<std::string> keys;
        keys.reserve(map.size());
        for (const auto& [k, v] : map) keys.push_back(k);
        std::sort(keys.begin(), keys.end());
        return keys;
    };

    out << "# Simulation Log (YAML)\n\n";

    // ── Accumulators ──
    if (!acc_double_.empty() || !acc_int_.empty()) {
        out << "accumulators:\n";
        for (const auto& k : sorted_keys(acc_double_)) {
            out << fmt::format("  \"{}\": {:.6e}\n", k, acc_double_.at(k));
        }
        for (const auto& k : sorted_keys(acc_int_)) {
            out << fmt::format("  \"{}\": {}\n", k, acc_int_.at(k));
        }
        out << "\n";
    }

    // ── Timers ──
    if (!timer_insertion_order_.empty()) {
        out << "timers:\n";
        for (const auto& label : timer_insertion_order_) {
            auto it = timers_.find(label);
            if (it == timers_.end() || it->second.count == 0) continue;
            const auto& t = it->second;
            double avg = t.total / t.count;
            out << fmt::format("  \"{}\":\n", label);
            out << fmt::format("    total: {:.6f}\n", t.total);
            out << fmt::format("    count: {}\n", t.count);
            out << fmt::format("    avg: {:.6f}\n", avg);
            out << fmt::format("    min: {:.6f}\n", t.min);
            out << fmt::format("    max: {:.6f}\n", t.max);
        }
        out << "\n";
    }

    // ── Statistics (series summaries) ──
    bool has_series = !series_double_.empty() || !series_int_.empty();
    if (has_series) {
        out << "statistics:\n";

        for (const auto& k : sorted_keys(series_double_)) {
            const auto& data = series_double_.at(k);
            if (data.empty()) continue;
            double sum = 0.0, mn = data[0], mx = data[0];
            for (double v : data) { sum += v; mn = std::min(mn, v); mx = std::max(mx, v); }
            double avg = sum / data.size();
            out << fmt::format("  \"{}\": {{total: {:.6e}, avg: {:.6e}, min: {:.6e}, max: {:.6e}, count: {}}}\n",
                k, sum, avg, mn, mx, data.size());
        }

        for (const auto& k : sorted_keys(series_int_)) {
            const auto& data = series_int_.at(k);
            if (data.empty()) continue;
            long long sum = 0; int mn = data[0], mx = data[0];
            for (int v : data) { sum += v; mn = std::min(mn, v); mx = std::max(mx, v); }
            double avg = (double)sum / data.size();
            out << fmt::format("  \"{}\": {{total: {}, avg: {:.6f}, min: {}, max: {}, count: {}}}\n",
                k, sum, avg, mn, mx, data.size());
        }
        out << "\n";
    }

    // ── Raw Series ──
    if (has_series || !series_string_.empty()) {
        out << "series:\n";

        for (const auto& k : sorted_keys(series_double_)) {
            const auto& data = series_double_.at(k);
            out << "  \"" << k << "\": [";
            for (size_t i = 0; i < data.size(); i++) {
                if (i > 0) out << ", ";
                out << fmt::format("{:.6e}", data[i]);
            }
            out << "]\n";
        }

        for (const auto& k : sorted_keys(series_int_)) {
            const auto& data = series_int_.at(k);
            out << "  \"" << k << "\": [";
            for (size_t i = 0; i < data.size(); i++) {
                if (i > 0) out << ", ";
                out << data[i];
            }
            out << "]\n";
        }

        for (const auto& k : sorted_keys(series_string_)) {
            const auto& data = series_string_.at(k);
            out << "  \"" << k << "\": [";
            for (size_t i = 0; i < data.size(); i++) {
                if (i > 0) out << ", ";
                out << "\"" << data[i] << "\"";
            }
            out << "]\n";
        }
    }

    out.close();
}

void Logger::save_to_disk() const
{
    if (path_.empty()) {
        std::cout << "symx::Logger::save_to_disk error: no path set." << std::endl;
        exit(-1);
    }
    save_to_disk(path_);
}

// =============================================================
// Control
// =============================================================
void Logger::clear()
{
    timers_.clear();
    timer_insertion_order_.clear();
    series_double_.clear();
    series_int_.clear();
    series_string_.clear();
    acc_double_.clear();
    acc_int_.clear();
}

void Logger::set_enabled(bool enabled)
{
    enabled_ = enabled;
}

bool Logger::is_enabled() const
{
    return enabled_;
}

// =============================================================
// Internal
// =============================================================
Logger::Timer& Logger::_get_or_create_timer(const std::string& label)
{
    auto it = timers_.find(label);
    if (it == timers_.end()) {
        timers_[label] = Timer();
        timer_insertion_order_.push_back(label);
        return timers_[label];
    }
    return it->second;
}

void Logger::_record_elapsed(const std::string& label, double elapsed)
{
    auto& timer = timers_[label];
    timer.total += elapsed;
    timer.count++;
    timer.min = std::min(timer.min, elapsed);
    timer.max = std::max(timer.max, elapsed);
}
