#include "gnuplot.h"

#include <fstream>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include <fmt/format.h>

namespace symx
{

bool is_gnuplot_available()
{
#ifdef _WIN32
    int result = std::system("where gnuplot >nul 2>&1");
#else
    int result = std::system("which gnuplot > /dev/null 2>&1");
#endif
    return (result == 0);
}

void print_gnuplot_install_instructions()
{
    std::cout << "\n[Line Search Debug] gnuplot is not installed or not in PATH.\n";
    std::cout << "To visualize the data, install gnuplot:\n";
#ifdef _WIN32
    std::cout << "  Windows: Download from http://www.gnuplot.info/ or use 'choco install gnuplot'\n";
#elif __APPLE__
    std::cout << "  macOS: brew install gnuplot\n";
#else
    std::cout << "  Linux: sudo apt install gnuplot (Debian/Ubuntu) or sudo dnf install gnuplot (Fedora)\n";
#endif
    std::cout << "Then run: gnuplot -p <script_file>\n";
}

void visualize_line_search_failure(
    const std::vector<std::vector<double>>& energy_samples,
    const std::string& output_dir,
    double E0,
    double E_threshold,
    double du_dot_grad)
{
    if (energy_samples.empty()) {
        return;
    }

    // Generate timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_now = std::localtime(&time_t_now);
    std::ostringstream timestamp_ss;
    timestamp_ss << std::put_time(tm_now, "%Y-%m-%d_%H-%M-%S");
    std::string timestamp = timestamp_ss.str();

    // Determine output directory (default to current dir)
    std::string dir = output_dir.empty() ? "." : output_dir;

    // CSV filename
    std::string csv_filename = dir + "/linesearch_failure_" + timestamp + ".csv";

    // Write CSV file
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        std::cout << "\n[Line Search Debug] ERROR: Could not write to " << csv_filename << std::endl;
        return;
    }

    // Header: step, newton_iter_0, newton_iter_1, ...
    csv_file << "step";
    for (size_t iter = 0; iter < energy_samples.size(); ++iter) {
        csv_file << ",newton_" << iter;
    }
    csv_file << "\n";

    // Data rows
    const int n_samples = 1000;
    for (int i = 0; i < n_samples; ++i) {
        double step_value = -0.5 + (2.0 * i) / (n_samples - 1);  // -0.5 to 1.5
        csv_file << fmt::format("{:.6e}", step_value);
        for (size_t iter = 0; iter < energy_samples.size(); ++iter) {
            const auto& samples = energy_samples[iter];
            if (i < static_cast<int>(samples.size())) {
                csv_file << "," << fmt::format("{:.10e}", samples[i]);
            } else {
                csv_file << ",";
            }
        }
        csv_file << "\n";
    }
    csv_file.close();

    std::cout << "\n========== LINE SEARCH FAILURE DEBUG ==========\n";
    std::cout << "E0 = " << fmt::format("{:6e}", E0) << ", E_threshold = " << fmt::format("{:6e}", E_threshold) << "\n";
    std::cout << "du_dot_grad = " << fmt::format("{:6e}", du_dot_grad) << "\n";
    std::cout << "CSV file written to: " << csv_filename << "\n";
    std::cout << "================================================\n";

    // Check if gnuplot is installed
    bool gnuplot_available = is_gnuplot_available();

    if (!gnuplot_available) {
        print_gnuplot_install_instructions();
    }

    // Write one gnuplot script per Newton iteration for separate windows
    std::vector<std::string> script_files;
    // for (size_t iter = 0; iter < energy_samples.size(); ++iter) {
    for (int iter = (int)energy_samples.size() - 1; iter >= 0; iter--) {
        std::string iter_script = dir + "/linesearch_failure_" + timestamp + "_iter" + std::to_string(iter) + ".gp";
        script_files.push_back(iter_script);
        
        std::ofstream gp_file(iter_script);
        if (gp_file.is_open()) {
            gp_file << "# Gnuplot script for line search failure - Newton iteration " << iter << "\n";
            gp_file << "set datafile separator ','\n";
            gp_file << "set xlabel 'Step size (fraction of du)'\n";
            gp_file << "set ylabel 'Energy'\n";
            gp_file << "set grid\n";
            gp_file << "set title 'Line Search Energy - Newton Iteration " << iter << "'\n";
            gp_file << "plot '" << csv_filename << "' using 1:" << (iter + 2) 
                    << " with lines lw 2 title 'Newton iter " << iter << "'\n";
            gp_file.close();
        }
    }

    if (gnuplot_available) {
        std::cout << "[Line Search Debug] Launching " << script_files.size() << " gnuplot windows in order...\n";
        // Launch in order with small delay to ensure window ordering
        for (size_t i = 0; i < script_files.size(); ++i) {
            const auto& script = script_files[i];
#ifdef _WIN32
            std::string cmd = "start \"\" gnuplot -p \"" + script + "\"";
#else
            std::string cmd = "gnuplot -p \"" + script + "\" &";
#endif
            std::system(cmd.c_str());
            // Small delay to ensure windows appear in order
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        std::cout << "Gnuplot scripts written to: " << dir << "/linesearch_failure_" << timestamp << "_iter*.gp\n";
    }
}

} // namespace symx
