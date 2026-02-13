#include "Settings.h"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <omp.h>

#include <fmt/format.h>

using namespace stark;
using namespace stark::core;

std::string time_stamp()
{
	auto now = std::chrono::system_clock::now();
	std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

#ifdef _WIN32
	std::tm localTime;
	localtime_s(&localTime, &currentTime);
#else
	std::tm localTime;
	localtime_r(&currentTime, &localTime);
#endif

	std::ostringstream oss;
	oss << std::put_time(&localTime, "%Y-%m-%d__%H-%M-%S");
	return oss.str();
}
using symx::to_string;
using symx::to_string_sci;
using symx::to_string_fixed;

std::string to_string(const Eigen::Vector3d& v)
{
	return fmt::format("({:f}, {:f}, {:f})", v[0], v[1], v[2]);
}

Settings::Settings()
{
	// Initialize default parameters
	this->execution.n_threads = omp_get_max_threads()/2;
	this->output.time_stamp = time_stamp();

	// Override symx defaults for simulation
	this->newton.residual_tolerance = 1e-6;
	this->newton.step_tolerance = 1e-3;
	this->newton.max_backtracking_invalid_state_iterations = 8; // increase IPC stiffness if penetrations at ~0.004 step length
	this->newton.projection_mode = symx::ProjectionToPD::Progressive;
}

std::string Settings::as_string() const
{
	std::string out;

	out += "\nStark Settings";

	out += "\n     Output";
	out += "\n         simulation_name: " + fmt::format("\"{}\"", this->output.simulation_name);
	out += "\n         output_directory: " + fmt::format("\"{}\"", this->output.output_directory);
	out += "\n         codegen_directory: " + fmt::format("\"{}\"", this->output.codegen_directory);
	out += "\n         time_stamp: " + fmt::format("\"{}\"", this->output.time_stamp);
	out += "\n         fps: " + std::to_string(this->output.fps);
	out += "\n         console_verbosity: " + to_string(this->output.console_verbosity);
	out += "\n         file_verbosity: " + to_string(this->output.file_verbosity);
	out += "\n         enable_frame_writes: " + to_string(this->output.enable_frame_writes);
	out += "\n         enable_output: " + to_string(this->output.enable_output);

	out += "\n     Simulation";
	out += "\n         gravity: " + to_string(this->simulation.gravity);
	out += "\n         init_frictional_contact: " + to_string(this->simulation.init_frictional_contact);
	out += "\n         max_time_step_size: " + to_string_sci(this->simulation.max_time_step_size, 4);
	out += "\n         use_adaptive_time_step: " + to_string(this->simulation.use_adaptive_time_step);
	out += "\n         time_step_size_success_multiplier: " + to_string_fixed(this->simulation.time_step_size_success_multiplier);
	out += "\n         time_step_size_lower_bound: " + to_string_sci(this->simulation.time_step_size_lower_bound);

	out += "\n     Newton's Method";
	out += this->newton.as_string("         ");

	out += "\n     Execution";
	out += "\n         allowed_execution_time: " + to_string_sci(this->execution.allowed_execution_time);
	out += "\n         end_simulation_time: " + to_string_sci(this->execution.end_simulation_time);
	out += "\n         end_frame: " + std::to_string(this->execution.end_frame);
	out += "\n         n_threads: " + std::to_string(this->execution.n_threads);
	out += "\n";
	return out;
}
