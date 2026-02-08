#pragma once
#include <string>
#include <vector>

namespace symx
{
    /**
     * @brief Visualizes line search energy samples using gnuplot.
     * 
     * Writes a CSV file with the energy samples and generates gnuplot scripts.
     * If gnuplot is available, launches one window per Newton iteration in order.
     * 
     * @param energy_samples Per Newton iteration: energy samples from -0.5*du to 1.5*du
     * @param output_dir Directory to write CSV and gnuplot scripts
     * @param E0 Initial energy value
     * @param E_threshold Energy threshold for Armijo condition
     * @param du_dot_grad Dot product of du and gradient
     */
    void visualize_line_search_failure(
        const std::vector<std::vector<double>>& energy_samples,
        const std::string& output_dir,
        double E0,
        double E_threshold,
        double du_dot_grad
    );

    /**
     * @brief Checks if gnuplot is available in the system PATH.
     * @return true if gnuplot is found, false otherwise.
     */
    bool is_gnuplot_available();

    /**
     * @brief Prints instructions for installing gnuplot.
     */
    void print_gnuplot_install_instructions();

} // namespace symx
