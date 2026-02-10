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

// =============================================================
// Getters
// =============================================================
double Logger::get_double(const std::string& label) const
{
    auto it = acc_double_.find(label);
    return (it != acc_double_.end()) ? it->second : 0.0;
}

int Logger::get_int(const std::string& label) const
{
    auto it = acc_int_.find(label);
    return (it != acc_int_.end()) ? it->second : 0;
}

const std::vector<double>& Logger::get_double_series(const std::string& label) const
{
    auto it = series_double_.find(label);
    return (it != series_double_.end()) ? it->second : empty_double_series_;
}

const std::vector<int>& Logger::get_int_series(const std::string& label) const
{
    auto it = series_int_.find(label);
    return (it != series_int_.end()) ? it->second : empty_int_series_;
}

const std::vector<std::string>& Logger::get_string_series(const std::string& label) const
{
    auto it = series_string_.find(label);
    return (it != series_string_.end()) ? it->second : empty_string_series_;
}

// =============================================================
// Timer Queries
// =============================================================
double Logger::get_timer_total(const std::string& label) const
{
    auto it = timers_.find(label);
    return (it != timers_.end()) ? it->second.total : 0.0;
}

int Logger::get_timer_count(const std::string& label) const
{
    auto it = timers_.find(label);
    return (it != timers_.end()) ? it->second.count : 0;
}

double Logger::get_timer_avg(const std::string& label) const
{
    auto it = timers_.find(label);
    if (it == timers_.end() || it->second.count == 0) return 0.0;
    return it->second.total / it->second.count;
}

double Logger::get_timer_min(const std::string& label) const
{
    auto it = timers_.find(label);
    if (it == timers_.end() || it->second.count == 0) return 0.0;
    return it->second.min;
}

double Logger::get_timer_max(const std::string& label) const
{
    auto it = timers_.find(label);
    if (it == timers_.end() || it->second.count == 0) return 0.0;
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

    // Timers
    if (!timer_insertion_order_.empty()) {
        out << "[Timers]\n";
        for (const auto& label : timer_insertion_order_) {
            const auto& t = timers_.at(label);
            if (t.count > 0) {
                double avg = t.total / t.count;
                out << fmt::format("  {}: total={:.6f} s | count={} | avg={:.6f} s | min={:.6f} s | max={:.6f} s\n",
                    label, t.total, t.count, avg, t.min, t.max);
            }
        }
        out << "\n";
    }

    // Accumulators (doubles)
    if (!acc_double_.empty()) {
        out << "[Doubles]\n";
        for (const auto& [label, val] : acc_double_) {
            out << fmt::format("  {}: {}\n", label, val);
        }
        out << "\n";
    }

    // Accumulators (ints)
    if (!acc_int_.empty()) {
        out << "[Ints]\n";
        for (const auto& [label, val] : acc_int_) {
            out << fmt::format("  {}: {}\n", label, val);
        }
        out << "\n";
    }

    // Double series
    for (const auto& [label, data] : series_double_) {
        out << label << ": ";
        for (size_t i = 0; i < data.size(); i++) {
            if (i > 0) out << ", ";
            out << data[i];
        }
        out << "\n";
    }

    // Int series
    for (const auto& [label, data] : series_int_) {
        out << label << ": ";
        for (size_t i = 0; i < data.size(); i++) {
            if (i > 0) out << ", ";
            out << data[i];
        }
        out << "\n";
    }

    // String series
    for (const auto& [label, data] : series_string_) {
        out << label << ": ";
        for (size_t i = 0; i < data.size(); i++) {
            if (i > 0) out << ", ";
            out << data[i];
        }
        out << "\n";
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
