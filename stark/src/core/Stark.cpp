#include "Stark.h"

#include <iostream>
#include <system_error>
#include <filesystem>

#include <Eigen/Dense>
#include <symx>
#include <fmt/format.h>

using namespace stark::core;

Stark::Stark(const Settings& settings)
	: settings(settings)
{
	// Set
	this->dt = this->settings.simulation.max_time_step_size;
	this->gravity = this->settings.simulation.gravity;

	// Check mandatory arguments
	if (settings.output.codegen_directory == "") {
		std::cout << "Stark::setup() error: Settings.output.codegen_directory must be set" << std::endl;
		exit(-1);
	}
	if (settings.output.output_directory == "") {
		std::cout << "Stark::setup() error: Settings.output.output_directory must be set" << std::endl;
		exit(-1);
	}
	if (settings.output.simulation_name == "") {
		std::cout << "Stark::setup() error: Settings.output.simulation_name must be set" << std::endl;
		exit(-1);
	}

	// Create folders
    std::error_code error;
    std::filesystem::create_directories(settings.output.codegen_directory, error);
	if (error) {
		std::cout << "Stark::setup() error: Cannot create folder " << settings.output.codegen_directory << std::endl;
		exit(-1);
	}

    std::filesystem::create_directories(settings.output.output_directory, error);
	if (error) {
		std::cout << "Stark::setup() error: Cannot create folder " << settings.output.output_directory << std::endl;
		exit(-1);
	}

	// Initialize Consoles and Loggers
	const std::string ending = "_" + settings.output.simulation_name + "__" + settings.output.time_stamp + ".txt";
	this->console.initialize(settings.output.output_directory + "/console" + ending, settings.output.console_verbosity, settings.output.console_output_to);
	this->logger.set_path(settings.output.output_directory + "/logger" + ending);
	this->newton.line_search_debug_logger.set_path(settings.output.output_directory + "/line_search" + ending);

	// Print settings
	this->console.print(this->settings.as_string(), ConsoleVerbosity::TimeSteps);

	// SymX
	this->global_energy.set_cse_mode(symx::CSE::Safe);
}
bool Stark::run(double duration, std::function<void()> callback)
{
	const double begin_time = this->current_time;

	// Check functions
	auto check_simulation_time = [&]()
	{
		if (this->current_time > this->settings.execution.end_simulation_time) {
			this->console.print(fmt::format("Simulation time exceeded. Exiting simulation.\n"), ConsoleVerbosity::Frames);
			return false;
		}
		return true;
	};
	auto check_duration = [&]()
	{
		if ((this->current_time - begin_time) > duration) {
			this->console.print(fmt::format("Duration time exceeded. Exiting simulation.\n"), ConsoleVerbosity::Frames);
			return false;
		}
		return true;
	};
	auto check_frame = [&]()
	{
		if (this->current_frame > this->settings.execution.end_frame) {
			this->console.print(fmt::format("Frame count exceeded. Exiting simulation.\n"), ConsoleVerbosity::Frames);
			return false;
		}
		return true;
	};
	auto check_execution_time = [&](double t0)
	{
		if ((omp_get_wtime() - t0) > this->settings.execution.allowed_execution_time) {
			this->console.print(fmt::format("Execution time exceeded. Exiting simulation.\n"), ConsoleVerbosity::Frames);
			return false;
		}
		return true;
	};

	// Simulation loop
	bool success = false;
	const double t0 = omp_get_wtime();
	while (check_simulation_time() && check_duration() && check_frame() && check_execution_time(t0))
	{
		if (callback != nullptr) { callback(); }
		success = this->run_one_step();
		if (!success) {
			break;
		}
	}

	// Finalize
	this->print();
	return success;
}
bool Stark::run_one_step()
{
	// Initialize
	if (!this->is_init) {
		this->_initialize();
	}
	bool success = false;
	this->logger.start_timing("total");

	// Time step begin
	this->console.print(fmt::format("\t dt: {:.6f} ms | ", 1000.0 * this->dt), ConsoleVerbosity::TimeSteps);
	this->callbacks.run_before_time_step();

	// Use Newton's Method to solve the time step update
	const double t0 = omp_get_wtime();
	NewtonState newton = this->newton.solve(this->dt, this->global_energy, this->callbacks, this->settings, this->console, this->logger);

	// Time step ended with success
	if (newton == NewtonState::Successful) {

		// After successful time step
		this->callbacks.run_on_time_step_accepted(); // Sets solution. x0 = x1. Exits minimization.
		this->callbacks.run_after_time_step();  // Can use the converged state x1, v1.
		this->current_time += this->dt;

		// Adaptive time step size
		this->dt = std::min(this->settings.simulation.max_time_step_size, this->dt * this->settings.simulation.time_step_size_success_muliplier);

		// Output
		const double runtime = omp_get_wtime() - t0;
		const double cr = runtime / this->dt;
		this->console.print(fmt::format(" | runtime: {:.0f} ms | cr: {:.1f}\n", 1000.0 * runtime, cr), ConsoleVerbosity::TimeSteps);
		this->logger.append_to_series("cr", cr);
		this->logger.append_to_series("dt", this->dt);
		this->logger.append_to_series("time", this->current_time);
		this->logger.append_to_series("frame", this->current_frame);
		this->logger.add_to_counter("time_steps", 1);
		this->logger.set("avg dt", this->current_time / (double)this->logger.get_int("time_steps"));
		this->logger.set("cr", this->logger.get_double("total") / this->current_time);
		this->_write_frame();

		// Return
		success = true;
	}

	// Time step failure
	else {
		// Output
		const double runtime = omp_get_wtime() - t0;
		this->console.print("\n", ConsoleVerbosity::TimeSteps);
		this->logger.add("failed_steps", runtime);

		// The failure was due to loose stiffnesses
		if (newton == NewtonState::InvalidConvergedState || newton == NewtonState::InvalidIntermediateConfiguration) {
			// Tighter stiffess should be already when measured during Newton's solve.
			// Do nothing, just run the time step again.
		}

		// The failure was due to the time step being too tough. Adapt the time step size and run the time step again.
		else {
			if (!this->settings.simulation.use_adaptive_time_step) {
				this->console.print("settings.simulation.adaptive_time_step is set to false. Exiting simulation.\n", ConsoleVerbosity::Frames);
				return false;
			}

			this->dt /= 2.0;
			if (this->dt < this->settings.simulation.time_step_size_lower_bound) {
				this->console.print(fmt::format("Adaptive time step size out of bounds ({:.e}). Exiting simulation.\n", this->settings.simulation.time_step_size_lower_bound), ConsoleVerbosity::Frames);
				return false;
			}
		}

		// Return
		success = true;
	}

	this->logger.stop_timing_add("total");
	return success;
}
std::string Stark::get_frame_path(std::string name) const
{
	return this->settings.output.output_directory + "/" + this->settings.output.simulation_name + "_" + name + "_" + std::to_string(this->current_frame);
}
void stark::core::Stark::print()
{
	// Info
	this->console.print("\nInfo\n", ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t # time_steps: {:d}\n", this->logger.get_int("time_steps")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t # newton/time_steps: {:.1f}\n", (double)this->logger.get_int("newton_iterations") / (double)this->logger.get_int("time_steps")), ConsoleVerbosity::Frames);
	if (this->settings.newton.linear_system_solver == LinearSystemSolver::CG) {
		this->console.print(fmt::format("\t # CG_iterations/newton: {:.1f}\n", (double)this->logger.get_int("CG_iterations") / (double)this->logger.get_int("newton_iterations")), ConsoleVerbosity::Frames);
	}
	this->console.print(fmt::format("\t # line_search/newton: {:.1f}\n", (double)this->logger.get_int("line_search_iterations") / (double)this->logger.get_int("newton_iterations")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t avg dt: {:.6f} ms\n", 1000.0 * this->logger.get_double("avg dt")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t cr: {:.1f}\n", this->logger.get_double("cr")), ConsoleVerbosity::Frames);

	// Runtime
	this->console.print("Runtime\n", ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t total: {:.3f} s\n", this->logger.get_double("total")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t evaluate_E_grad_hess: {:.3f} s\n", this->logger.get_double("evaluate_E_grad_hess")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t evaluate_E_grad: {:.3f} s\n", this->logger.get_double("evaluate_E_grad")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t evaluate_E: {:.3f} s\n", this->logger.get_double("evaluate_E")), ConsoleVerbosity::Frames);
	if (this->settings.newton.linear_system_solver == LinearSystemSolver::CG) {
		this->console.print(fmt::format("\t CG: {:.3f} s\n", this->logger.get_double("CG")), ConsoleVerbosity::Frames);
	}
	else if (this->settings.newton.linear_system_solver == LinearSystemSolver::DirectLU) {
		this->console.print(fmt::format("\t directLU: {:.3f} s\n", this->logger.get_double("directLU")), ConsoleVerbosity::Frames);
	}
	this->console.print(fmt::format("\t before_energy_evaluation: {:.3f} s\n", this->logger.get_double("before_energy_evaluation")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t after_energy_evaluation: {:.3f} s\n", this->logger.get_double("after_energy_evaluation")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t is_intermidiate_state_valid: {:.3f} s\n", this->logger.get_double("is_intermidiate_state_valid")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t failed steps: {:.3f} s\n", this->logger.get_double("failed_steps")), ConsoleVerbosity::Frames);
	this->console.print(fmt::format("\t write_frame: {:.3f} s\n", this->logger.get_double("write_frame")), ConsoleVerbosity::Frames);
}

void Stark::_initialize()
{
	this->is_init = true;

	if (this->global_energy.get_total_n_dofs() == 0) {
		this->console.print("Stark::_initialize() error: No degrees of freedom. Exiting simulation.", ConsoleVerbosity::Frames);
		exit(-1);
	}

	// SymX
	//// Compile
	symx::GlobalEnergy::CompilationOptions options;
	options.n_threads = this->settings.execution.n_threads;
	options.force_compilation = this->settings.debug.symx_force_compilation;
	options.force_load = this->settings.debug.symx_force_load;
	options.suppress_compiler_output = this->settings.debug.symx_suppress_compiler_output;
	options.msg_callback = [&](const std::string& msg) { this->console.print(msg, ConsoleVerbosity::Frames); };
	this->global_energy.compile(this->settings.output.codegen_directory, options);
	if (this->settings.debug.symx_force_load) {
		this->console.print("\t-|-|-|-|- WARNING: Forced load enabled. Loaded expressions might be outdated. -|-|-|-|-", ConsoleVerbosity::Frames);
	}

	//// Parameters
	if (this->settings.newton.project_to_PD) {
		this->global_energy.set_project_to_PD(true);
	}
	this->global_energy.set_check_for_NaNs(this->settings.debug.symx_check_for_NaNs);

	//// Print ndofs
	this->console.print("\nDegrees of freedom:", ConsoleVerbosity::Frames);
	for (int i = 0; i < (int)this->global_energy.dof_labels.size(); i++) {
		this->console.print(fmt::format("\n\t {} {:d}", this->global_energy.dof_labels[i], this->global_energy.dof_ndofs[i]()), ConsoleVerbosity::Frames);
	}
	this->console.print("\n\n", ConsoleVerbosity::Frames);

	// Write frame zero
	this->_write_frame();

	// Callbacks
	this->callbacks.run_before_simulation();

	// Check that the initial state is valid
	if (!this->callbacks.run_is_initial_state_valid()) {
		this->console.print("Initial state is not valid. Exiting simulation.", ConsoleVerbosity::Frames);
		exit(-1);
	}
}
void Stark::_write_frame()
{
	if (!this->settings.output.enable_output) { return; }

	auto write_frame_impl = [&]()
		{
			this->callbacks.run_write_frame();
			this->console.print(fmt::format("Frame: {:d}. Time: {:.3f} s.\n", this->current_frame, this->current_time), ConsoleVerbosity::Frames);
			this->current_frame++;
			this->logger.save_to_disk();
		};

	this->logger.start_timing("write_frame");
	if (this->settings.output.fps < 0) {
		write_frame_impl();
	}
	else {
		while (this->current_time > this->next_frame_time - std::numeric_limits<double>::epsilon()) {
			write_frame_impl();
			this->next_frame_time += 1.0 / (double)this->settings.output.fps;
		}
	}
	this->logger.stop_timing_add("write_frame");
}
