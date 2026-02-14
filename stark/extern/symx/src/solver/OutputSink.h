#pragma once
#include <string>
#include <iostream>
#include <fstream>

namespace symx
{
    enum class Verbosity
    {
        Minimal = 0,
        Summary = 1,
        Medium = 2,
        Full = 3,
    };
    inline std::string to_string(Verbosity v)
    {
        switch (v) {
            case Verbosity::Minimal:  return "Minimal";
            case Verbosity::Summary: return "Summary";
            case Verbosity::Medium:    return "Medium";
            case Verbosity::Full:  return "Full";
            default:           
                std::cout << "symx::Verbosity " << (int)v << " does not have a name. Exiting." << std::endl;
                exit(-1);
        }
    }

    class OutputSink
    {
    public:
        // --- Verbosity ---
        void set_console_verbosity(Verbosity v) { console_verbosity_ = v; }
        void set_file_verbosity(Verbosity v) { file_verbosity_ = v; }
        Verbosity get_console_verbosity() const { return console_verbosity_; }
        Verbosity get_file_verbosity() const { return file_verbosity_; }

        // --- Indentation ---
        void set_root_tab(int level) { root_tab_ = level; }
        void set_tab_size(int spaces) { tab_size_ = spaces; }
        int get_root_tab() const { return root_tab_; }
        int get_tab_size() const { return tab_size_; }

        // --- File ---
        void open_file(const std::string& path) {
            file_.open(path);
            if (!file_.is_open()) {
                std::cout << "symx::OutputSink error: Failed to open file at path: " << path << std::endl;
                exit(-1);
            }
        }
        void close_file() {
            if (file_.is_open()) {
                file_.close();
            }
        }
        bool is_file_open() const { return file_.is_open(); }

        // --- Global enable ---
        void set_enabled(bool v) { enabled_ = v; }
        bool is_enabled() const { return enabled_; }

        // --- Core API ---

        // Print with verbosity gate.
        void print(const std::string& msg, Verbosity level) const {
            _emit(msg, level);
        }

        // Ungated print — no verbosity check, no auto-indent.
        void print(const std::string& msg) const {
            _emit(msg, Verbosity::Minimal);
        }

        void print_new_line(int indent = 0) const {
            _emit(indent ? "\n" + _indent(indent) : "\n", Verbosity::Minimal);
        }

        void print_new_line(Verbosity level, bool indent = true) const {
            _emit(indent ? "\n" + _indent(level) : "\n", level);
        }

        void print_with_new_line(const std::string& msg, Verbosity level, bool indent = true) const {
            const std::string new_line = indent ? "\n" + _indent(level) : "\n";
            _emit(new_line + msg, level);
        }

        void print_with_new_line(const std::string& msg, int indent = 0) const {
            const std::string new_line = "\n" + _indent(indent);
            _emit(new_line + msg, Verbosity::Minimal);
        }

        void flush_file() {
            if (file_.is_open()) file_.flush();
        }

    private:
        void _emit(const std::string& msg, Verbosity level) const {
            if (!enabled_) return;
            if (level <= console_verbosity_) {
                std::cout << msg;
            }
            if (file_.is_open() && level <= file_verbosity_) {
                file_ << msg;
            }
        }

        std::string _indent(int level) const {
            return std::string(level * tab_size_, ' ');
        }
        std::string _indent(Verbosity level) const {
            int depth = std::max(0, root_tab_ + static_cast<int>(level) - 1);
            return _indent(depth);
        }

        Verbosity console_verbosity_ = Verbosity::Summary;
        Verbosity file_verbosity_ = Verbosity::Full;
        bool enabled_ = true;
        int root_tab_ = 0;
        int tab_size_ = 4;
        mutable std::ofstream file_;
    };
    using spOutputSink = std::shared_ptr<OutputSink>;
}
