#pragma once
#include <string>
#include <iostream>
#include <fstream>

namespace symx
{
    enum class Verbosity
    {
        Silent = 0,
        Summary = 1,
        Step = 2,
        Detail = 3,

        // Backward compatibility aliases
        None = Silent,
        StepIterations = Step,
        LineSearchIteration = Detail,
    };

    enum class OutputMode
    {
        PrintOnly,      // stdout only
        FileOnly,       // file only
        PrintAndFile,   // both
    };

    class OutputSink
    {
    public:
        // --- Verbosity ---
        void set_verbosity(Verbosity v) { verbosity_ = v; }
        Verbosity get_verbosity() const { return verbosity_; }

        // --- Indentation ---
        void set_root_tab(int level) { root_tab_ = level; }
        void set_tab_size(int spaces) { tab_size_ = spaces; }
        int get_root_tab() const { return root_tab_; }
        int get_tab_size() const { return tab_size_; }

        // --- Output mode & file ---
        void set_mode(OutputMode m) { mode_ = m; }
        OutputMode get_mode() const { return mode_; }

        void open_file(const std::string& path) {
            file_.open(path);
        }
        void close_file() {
            if (file_.is_open()) {
                file_.close();
            }
        }
        bool is_file_open() const { return file_.is_open(); }

        // --- Console enable ---
        void set_console_enabled(bool v) { console_enabled_ = v; }
        bool is_console_enabled() const { return console_enabled_; }

        // --- Core API ---

        // Print with verbosity gate.
        // If indent=true (default), indentation = (root_tab + level) * tab_size spaces, prepended automatically.
        void print(const std::string& msg, Verbosity level, bool indent = true) const {
            if (level > verbosity_) return;
            _emit(indent ? _indent(level) + msg : msg);
        }

        // Ungated print — no verbosity check, no auto-indent.
        void print(const std::string& msg) const {
            _emit(msg);
        }

        void flush_file() {
            if (file_.is_open()) file_.flush();
        }

    private:
        void _emit(const std::string& msg) const {
            if (console_enabled_ && mode_ != OutputMode::FileOnly) {
                std::cout << msg;
            }
            if (file_.is_open() && mode_ != OutputMode::PrintOnly) {
                file_ << msg;
            }
        }

        std::string _indent(Verbosity level) const {
            int depth = root_tab_ + static_cast<int>(level);
            return std::string(depth * tab_size_, ' ');
        }

        Verbosity verbosity_ = Verbosity::Step;
        OutputMode mode_ = OutputMode::PrintOnly;
        bool console_enabled_ = true;
        int root_tab_ = 0;
        int tab_size_ = 2;
        mutable std::ofstream file_;
    };
}
