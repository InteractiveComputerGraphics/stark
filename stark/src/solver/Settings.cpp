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

stark::Settings::Settings()
{
	// Initialize default parameters
	this->execution.n_threads = omp_get_max_threads()/2;
	
	//// Adaptive Time step
	this->simulation.adaptive_time_step.value = 0.01; // [s]
	this->simulation.adaptive_time_step.min = 0.0; // [s]
	this->simulation.adaptive_time_step.max = 0.01; // [s]
	this->simulation.adaptive_time_step.success_multiplier = 1.2;
	this->simulation.adaptive_time_step.failure_multiplier = 0.5;
	this->simulation.adaptive_time_step.n_successful_iterations_to_increase = 5;

	//// Adaptive Contact stiffness
	this->contact.adaptive_contact_stiffness.value = 1e3;
	this->contact.adaptive_contact_stiffness.min = 1e3;
	this->contact.adaptive_contact_stiffness.max = 1e15;
	this->contact.adaptive_contact_stiffness.success_multiplier = 1.0;
	this->contact.adaptive_contact_stiffness.failure_multiplier = 2.0;
	this->contact.adaptive_contact_stiffness.n_successful_iterations_to_increase = std::numeric_limits<int>::max();
}

std::string stark::Settings::as_string() const
{
	std::string out;

	out += "\nStark Settings";
	
	out += "\n     Output";
	out += "\n         simulation_name: " + fmt::format("\"{}\"", this->output.simulation_name);
	out += "\n         output_directory: " + fmt::format("\"{}\"", this->output.output_directory);
	out += "\n         codegen_directory: " + fmt::format("\"{}\"", this->output.codegen_directory);
	out += "\n         fps: " + std::to_string(this->output.fps);
	out += "\n         console_verbosity: " + to_string(this->output.console_verbosity);
	out += "\n         console_output_to: " + to_string(this->output.console_output_to);
	out += "\n         suppress_symx_compiler_output: " + to_string(this->output.suppress_symx_compiler_output);

	out += "\n     Simulation";
	out += "\n         adaptive_time_step" + to_string(this->simulation.adaptive_time_step);
	out += "\n         gravity: " + to_string(this->simulation.gravity);
	out += "\n         boundary_conditions_stiffness: " + fmt::format("{:.1e}", this->simulation.boundary_conditions_stiffness);

	out += "\n     Contact";
	out += "\n         adaptive_contact_stiffness" + to_string(this->contact.adaptive_contact_stiffness);
	out += "\n         dhat: " + fmt::format("{:.1e}", this->contact.dhat);
	out += "\n         friction_stick_slide_threshold: " + fmt::format("{:.1e}", this->contact.friction_stick_slide_threshold);
	out += "\n         friction_stick_slide_stiffness: " + fmt::format("{:.1e}", this->contact.friction_stick_slide_stiffness);
	out += "\n         edge_edge_cross_norm_sq_cutoff: " + fmt::format("{:.1e}", this->contact.edge_edge_cross_norm_sq_cutoff);
	out += "\n         friction_displacement_perturbation: " + fmt::format("{:.1e}", this->contact.friction_displacement_perturbation);
	out += "\n         collisions_enabled: " + to_string(this->contact.collisions_enabled);
	out += "\n         friction_enabled: " + to_string(this->contact.friction_enabled);
	out += "\n         triangle_point_enabled: " + to_string(this->contact.triangle_point_enabled);
	out += "\n         edge_edge_enabled: " + to_string(this->contact.edge_edge_enabled);
	out += "\n         enable_intersection_test: " + to_string(this->contact.enable_intersection_test);

	out += "\n     Newton's Method";
	out += "\n         newton_tol: " + fmt::format("{:.1e}", this->newton.newton_tol);
	out += "\n         max_newton_iterations: " + std::to_string(this->newton.max_newton_iterations);
	out += "\n         max_line_search_iterations: " + std::to_string(this->newton.max_line_search_iterations);
	out += "\n         line_search_multiplier: " + fmt::format("{:f}", this->newton.line_search_multiplier);
	out += "\n         cg_tol: " + fmt::format("{:.1e}", this->newton.cg_tol);
	out += "\n         cg_max_iterations_multiplier: " + fmt::format("{:f}", this->newton.cg_max_iterations_multiplier);
	out += "\n         use_direct_linear_solve: " + to_string(this->newton.use_direct_linear_solve);
	out += "\n         project_to_PD: " + to_string(this->newton.project_to_PD);
	out += "\n         debug_line_search_output: " + to_string(this->newton.debug_line_search_output);

	out += "\n     Execution";
	out += "\n         allowed_execution_time: " + fmt::format("{:.1e}", this->execution.allowed_execution_time);
	out += "\n         end_simulation_time: " + fmt::format("{:.1e}", this->execution.end_simulation_time);
	out += "\n         end_frame: " + fmt::format("{:d}", this->execution.end_frame);
	out += "\n         n_threads: " + fmt::format("{:d}", this->execution.n_threads);
	out += "\n\n";
	return out;
}
