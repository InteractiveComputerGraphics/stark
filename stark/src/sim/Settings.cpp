#include "Settings.h"

#include <omp.h>
#include <fmt/format.h>


std::string to_string(stark::Verbosity v)
{
	switch (v)
	{
	case stark::Verbosity::NoOutput: return "NoOutput"; break;
	case stark::Verbosity::Frames: return "Frames"; break;
	case stark::Verbosity::TimeSteps: return "TimeSteps"; break;
	case stark::Verbosity::NewtonIterations: return "NewtonIterations"; break;
	default: return ""; break;
	}
}
std::string to_string(stark::OutputTo v)
{
	switch (v)
	{
	case stark::OutputTo::ConsoleOnly: return "ConsoleOnly"; break;
	case stark::OutputTo::FileOnly: return "FileOnly"; break;
	case stark::OutputTo::FileAndConsole: return "FileAndConsole"; break;
	case stark::OutputTo::NoOutput: return "NoOutput"; break;
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
std::string to_string(const stark::AdaptiveParameter& v)
{
	std::string out;
	out += fmt::format("\n\t\t\t value: {:2e}", v.value);
	out += fmt::format("\n\t\t\t min: {:2e}", v.min);
	out += fmt::format("\n\t\t\t max: {:2e}", v.max);
	out += fmt::format("\n\t\t\t success_multiplier: {:f}", v.success_multiplier);
	out += fmt::format("\n\t\t\t failure_multiplier: {:f}", v.failure_multiplier);
	out += fmt::format("\n\t\t\t n_successful_iterations_to_increase: {:d}", v.n_successful_iterations_to_increase);
	return out;
}
std::string to_string(const Eigen::Vector3d& v)
{
	return fmt::format("({:f}, {:f}, {:f})", v[0], v[1], v[2]);
}


stark::Settings::Settings()
{
	// Initialize default parameters
	this->execution.n_threads = omp_get_num_threads();
	
	//// Adaptive Time step
	this->simulation.adaptive_time_step.value = 0.01; // [s]
	this->simulation.adaptive_time_step.min = 0.0; // [s]
	this->simulation.adaptive_time_step.max = 0.01; // [s]
	this->simulation.adaptive_time_step.success_multiplier = 1.2;
	this->simulation.adaptive_time_step.failure_multiplier = 0.5;
	this->simulation.adaptive_time_step.n_successful_iterations_to_increase = 5;

	//// Adaptive Contact stiffness
	this->simulation.adaptive_time_step.value = 1e3;
	this->simulation.adaptive_time_step.min = 1e3;
	this->simulation.adaptive_time_step.max = 1e15;
	this->simulation.adaptive_time_step.success_multiplier = 1.0;
	this->simulation.adaptive_time_step.failure_multiplier = 2.0;
	this->simulation.adaptive_time_step.n_successful_iterations_to_increase = std::numeric_limits<int>::max();
}

std::string stark::Settings::as_string() const
{
	std::string out;

	out += "\nStark Settings";
	
	out += "\n\t Output";
	out += "\n\t\t simulation_name: " + this->output.simulation_name;
	out += "\n\t\t output_directory: " + this->output.output_directory;
	out += "\n\t\t codegen_directory: " + this->output.codegen_directory;
	out += "\n\t\t fps: " + std::to_string(this->output.fps);
	out += "\n\t\t console_verbosity: " + to_string(this->output.console_verbosity);
	out += "\n\t\t console_output_to: " + to_string(this->output.console_output_to);
	out += "\n\t\t suppress_symx_compiler_output: " + to_string(this->output.suppress_symx_compiler_output);

	out += "\n\t Simulation";
	out += "\n\t\t adaptive_time_step" + to_string(this->simulation.adaptive_time_step);
	out += "\n\t\t gravity" + to_string(this->simulation.gravity);
	out += "\n\t\t boundary_conditions_stiffness" + fmt::format("{:1e}", this->simulation.boundary_conditions_stiffness);

	out += "\n\t Contact";
	out += "\n\t\t adaptive_contact_stiffness" + to_string(this->contact.adaptive_contact_stiffness);
	out += "\n\t\t dhat" + fmt::format("{:1e}", this->contact.dhat);

	out += "\n\t Newton's Method";
	out += "\n\t\t newton_tol" + fmt::format("{:e}", this->newton.newton_tol);
	out += "\n\t\t max_newton_iterations" + std::to_string(this->newton.max_newton_iterations);
	out += "\n\t\t max_line_search_iterations" + std::to_string(this->newton.max_line_search_iterations);
	out += "\n\t\t line_search_multiplier" + fmt::format("{:f}", this->newton.line_search_multiplier);
	out += "\n\t\t cg_tol" + fmt::format("{:e}", this->newton.cg_tol);
	out += "\n\t\t cg_max_iterations_multiplier" + fmt::format("{:f}", this->newton.cg_max_iterations_multiplier);
	out += "\n\t\t use_direct_linear_solve: " + to_string(this->newton.use_direct_linear_solve);
	out += "\n\t\t project_to_PD: " + to_string(this->newton.project_to_PD);
	out += "\n\t\t debug_line_search_output: " + to_string(this->newton.debug_line_search_output);

	out += "\n\t Execution";
	out += "\n\t\t allowed_execution_time" + fmt::format("{:f}", this->execution.allowed_execution_time);
	out += "\n\t\t end_simulation_time" + fmt::format("{:f}", this->execution.end_simulation_time);
	out += "\n\t\t end_frame" + fmt::format("{:d}", this->execution.end_frame);
	out += "\n\t\t n_threads" + fmt::format("{:d}", this->execution.n_threads);
	return out;
}
