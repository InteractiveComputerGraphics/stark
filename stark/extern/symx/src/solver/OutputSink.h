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
        Full = 3,
    };
    inline std::string to_string(Verbosity v)
    {
        switch (v) {
            case Verbosity::Silent:  return "Silent";
            case Verbosity::Summary: return "Summary";
            case Verbosity::Step:    return "Step";
            case Verbosity::Full:  return "Full";
            default:           
                std::cout << "symx::Verbosity " << (int)v << " does not have a name. Exiting." << std::endl;
                exit(-1);
        }
    }

    enum class OutputTo
    {
        PrintOnly,
        FileOnly,
        PrintAndFile,
    };
    inline std::string to_string(OutputTo m)
    {
        switch (m) {
            case OutputTo::PrintOnly:     return "PrintOnly";
            case OutputTo::FileOnly:      return "FileOnly";
            case OutputTo::PrintAndFile:  return "PrintAndFile";
            default:
                std::cout << "symx::OutputTo " << (int)m << " does not have a name. Exiting." << std::endl;
                exit(-1);
        }
    }

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
        void set_output_to(OutputTo m) { output_to_ = m; }
        OutputTo get_output_to() const { return output_to_; }

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
        void set_enabled(bool v) { enabled_ = v; }
        bool is_enabled() const { return enabled_; }

        // --- Core API ---

        // Print with verbosity gate.
        void print(const std::string& msg, Verbosity level) const {
            if (level > verbosity_) return;
            _emit(msg);
        }

        // Ungated print — no verbosity check, no auto-indent.
        void print(const std::string& msg) const {
            _emit(msg);
        }

        void print_new_line(int indent = 0) const {
            _emit(indent ? "\n" + _indent(indent) : "\n");
        }

        void print_new_line(Verbosity level, bool indent = true) const {
            if (level > verbosity_) return;
            _emit(indent ? "\n" + _indent(level) : "\n");
        }

        void print_with_new_line(const std::string& msg, Verbosity level, bool indent = true) const {
            this->print_new_line(level, indent);
            this->print(msg, level);
        }

        void print_with_new_line(const std::string& msg, int indent = 0) const {
            this->print_new_line(indent);
            this->print(msg);
        }

        void flush_file() {
            if (file_.is_open()) file_.flush();
        }

    private:
        void _emit(const std::string& msg) const {
            if (!enabled_) return;
            if (output_to_ != OutputTo::FileOnly) {
                std::cout << msg;
            }
            if (output_to_ != OutputTo::PrintOnly) {
                if (file_.is_open()) {
                    file_ << msg;
                }
                else {
                    std::cout << "OutputSink error: File output mode enabled but file is not open. Use OutputSink::open_file() before directing to file." << std::endl;
                    exit(-1);
                }
            }
        }

        std::string _indent(int level) const {
            return std::string(level * tab_size_, ' ');
        }
        std::string _indent(Verbosity level) const {
            int depth = root_tab_ + static_cast<int>(level);
            return _indent(depth);
        }

        Verbosity verbosity_ = Verbosity::Step;
        OutputTo output_to_ = OutputTo::PrintOnly;
        bool enabled_ = true;
        int root_tab_ = 0;
        int tab_size_ = 2;
        mutable std::ofstream file_;
    };
    using spOutputSink = std::shared_ptr<OutputSink>;
}
