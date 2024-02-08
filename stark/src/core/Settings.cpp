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
std::string to_string(const AdaptiveParameter& v)
{
	std::string out;
	out += fmt::format("\n             value: {:.2e}", v.value);
	out += fmt::format("\n             min: {:.2e}", v.min);
	out += fmt::format("\n             max: {:.2e}", v.max);
	out += fmt::format("\n             success_multiplier: {:f}", v.success_multiplier);
	out += fmt::format("\n             failure_multiplier: {:f}", v.failure_multiplier);
	out += fmt::format("\n             n_successful_iterations_to_increase: {:d}", v.n_successful_iterations_to_increase);
	return out;
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

	//// Adaptive Time step
	this->simulation.adaptive_time_step.value = 0.01; // [s]
	this->simulation.adaptive_time_step.min = 0.0; // [s]
	this->simulation.adaptive_time_step.max = 0.01; // [s]
	this->simulation.adaptive_time_step.success_multiplier = 1.2;
	this->simulation.adaptive_time_step.failure_multiplier = 0.5;
	this->simulation.adaptive_time_step.n_successful_iterations_to_increase = 5;

	//// Adaptive Contact stiffness (mild adaptivity, too strong results in fluctuations)
	this->contact.adaptive_contact_stiffness.value = 1e6;
	this->contact.adaptive_contact_stiffness.min = 1e6;
	this->contact.adaptive_contact_stiffness.max = 1e15;
	this->contact.adaptive_contact_stiffness.success_multiplier = 0.8;
	this->contact.adaptive_contact_stiffness.failure_multiplier = 2.0;
	this->contact.adaptive_contact_stiffness.n_successful_iterations_to_increase = 50;
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
	out += "\n         calculate_smooth_normals: " + to_string(this->output.calculate_smooth_normals);
	out += "\n         enable_output: " + to_string(this->output.enable_output);

	out += "\n     Simulation";
	out += "\n         adaptive_time_step" + to_string(this->simulation.adaptive_time_step);
	out += "\n         gravity: " + to_string(this->simulation.gravity);
	out += "\n         boundary_conditions_stiffness: " + fmt::format("{:.1e}", this->simulation.boundary_conditions_stiffness);

	out += "\n     Contact";
	out += "\n         adaptive_contact_stiffness" + to_string(this->contact.adaptive_contact_stiffness);
	out += "\n         dhat: " + fmt::format("{:.1e}", this->contact.dhat);
	out += "\n         friction_stick_slide_threshold: " + fmt::format("{:.1e}", this->contact.friction_stick_slide_threshold);
	out += "\n         edge_edge_cross_norm_sq_cutoff: " + fmt::format("{:.1e}", this->contact.edge_edge_cross_norm_sq_cutoff);
	out += "\n         friction_displacement_perturbation: " + fmt::format("{:.1e}", this->contact.friction_displacement_perturbation);
	out += "\n         better_friction_mode: " + to_string(this->contact.better_friction_mode);
	out += "\n         collisions_enabled: " + to_string(this->contact.collisions_enabled);
	out += "\n         friction_enabled: " + to_string(this->contact.friction_enabled);
	out += "\n         triangle_point_enabled: " + to_string(this->contact.triangle_point_enabled);
	out += "\n         edge_edge_enabled: " + to_string(this->contact.edge_edge_enabled);
	out += "\n         enable_intersection_test: " + to_string(this->contact.enable_intersection_test);

	out += "\n     Newton's Method";
	out += "\n         residual_type: " + to_string(this->newton.residual.type);
	out += "\n         newton_tolerance: " + fmt::format("{:.1e}", this->newton.residual.tolerance);
	out += "\n         linear_system_solver: " + to_string(this->newton.linear_system_solver);
	out += "\n         project_to_PD: " + to_string(this->newton.project_to_PD);
	out += "\n         max_newton_iterations: " + std::to_string(this->newton.max_newton_iterations);
	out += "\n         max_line_search_iterations: " + std::to_string(this->newton.max_line_search_iterations);
	out += "\n         line_search_multiplier: " + fmt::format("{:f}", this->newton.line_search_multiplier);
	out += "\n         cg_max_iterations_multiplier: " + fmt::format("{:f}", this->newton.cg_max_iterations_multiplier);

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
