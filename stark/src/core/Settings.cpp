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
std::string to_string(ConsoleVerbosity v)
{
	switch (v)
	{
	case ConsoleVerbosity::NoOutput: return "NoOutput"; break;
	case ConsoleVerbosity::Frames: return "Frames"; break;
	case ConsoleVerbosity::TimeSteps: return "TimeSteps"; break;
	case ConsoleVerbosity::NewtonIterations: return "NewtonIterations"; break;
	default: return ""; break;
	}
}
std::string to_string(ConsoleOutputTo v)
{
	switch (v)
	{
	case ConsoleOutputTo::ConsoleOnly: return "ConsoleOnly"; break;
	case ConsoleOutputTo::FileOnly: return "FileOnly"; break;
	case ConsoleOutputTo::FileAndConsole: return "FileAndConsole"; break;
	case ConsoleOutputTo::NoOutput: return "NoOutput"; break;
	default: return ""; break;
	}
}
std::string to_string(ResidualType v)
{
	switch (v)
	{
	case ResidualType::Force: return "Force"; break;
	case ResidualType::Acceleration: return "Acceleration"; break;
	default: return ""; break;
	}
}
std::string to_string(LinearSystemSolver v)
{
	switch (v)
	{
	case LinearSystemSolver::CG: return "CG"; break;
	case LinearSystemSolver::DirectLU: return "DirectLU"; break;
	default: return ""; break;
	}
}
std::string to_string(const bool v)
{
	if (v) {
		return "true";
	}
	else {
		return "false";
	}
}
std::string to_string(const Eigen::Vector3d& v)
{
	return fmt::format("({:f}, {:f}, {:f})", v[0], v[1], v[2]);
}

Settings::Settings()
{
	// Initialize default parameters
	this->execution.n_threads = omp_get_max_threads();
	this->output.time_stamp = time_stamp();
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
	out += "\n         console_output_to: " + to_string(this->output.console_output_to);
	out += "\n         enable_output: " + to_string(this->output.enable_output);

	out += "\n     Simulation";
	out += "\n         gravity: " + to_string(this->simulation.gravity);
	out += "\n         init_frictional_contact: " + to_string(this->simulation.init_frictional_contact);
	out += "\n         max_time_step_size: " + to_string(this->simulation.max_time_step_size);
	out += "\n         use_adaptive_time_step: " + to_string(this->simulation.use_adaptive_time_step);
	out += "\n         time_step_size_success_muliplier: " + to_string(this->simulation.time_step_size_success_muliplier);
	out += "\n         time_step_size_lower_bound: " + fmt::format("{:.1e}", this->simulation.time_step_size_lower_bound);

	out += "\n     Newton's Method";
	out += "\n         residual_type: " + to_string(this->newton.residual.type);
	out += "\n         newton_tolerance: " + fmt::format("{:.1e}", this->newton.residual.tolerance);
	out += "\n         linear_system_solver: " + to_string(this->newton.linear_system_solver);
	out += "\n         project_to_PD: " + to_string(this->newton.project_to_PD);
	out += "\n         max_newton_iterations: " + std::to_string(this->newton.max_newton_iterations);
	out += "\n         max_line_search_iterations: " + std::to_string(this->newton.max_line_search_iterations);
	out += "\n         line_search_multiplier: " + fmt::format("{:f}", this->newton.line_search_multiplier);
	out += "\n         cg_max_iterations_multiplier: " + fmt::format("{:f}", this->newton.cg_max_iterations_multiplier);
	out += "\n         epsilon_residual: " + fmt::format("{:.1e}", this->newton.epsilon_residual);

	out += "\n     Execution";
	out += "\n         allowed_execution_time: " + fmt::format("{:.1e}", this->execution.allowed_execution_time);
	out += "\n         end_simulation_time: " + fmt::format("{:.1e}", this->execution.end_simulation_time);
	out += "\n         end_frame: " + fmt::format("{:d}", this->execution.end_frame);
	out += "\n         n_threads: " + fmt::format("{:d}", this->execution.n_threads);

	out += "\n     Debug";
	out += "\n         symx_check_for_NaNs: " + to_string(this->debug.symx_check_for_NaNs);
	out += "\n         symx_suppress_compiler_output: " + to_string(this->debug.symx_suppress_compiler_output);
	out += "\n         symx_force_compilation: " + to_string(this->debug.symx_force_compilation);
	out += "\n         symx_force_load: " + to_string(this->debug.symx_force_load);
	out += "\n         debug_line_search_output: " + to_string(this->debug.line_search_output);
	out += "\n\n";
	return out;
}
