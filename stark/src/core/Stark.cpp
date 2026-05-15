#include "Stark.h"

#include <iostream>
#include <system_error>
#include <filesystem>

#include <algorithm>
#include <Eigen/Dense>
#include <symx>
#include <fmt/format.h>

using namespace stark::core;
using namespace symx;

Stark::Stark(const Settings& settings)
	: settings(settings)
{
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

	// Initialize SymX context
	const std::string filename = settings.output.output_directory + "/" + settings.output.simulation_name + "__" + settings.output.time_stamp;

	// Create SymX context and configure OutputSink
	this->context = Context::create();
	this->context->compilation_directory = this->settings.output.codegen_directory;
	this->context->n_threads = this->settings.execution.n_threads;
	
	//// Logger
	this->context->logger->set_path(filename + ".yaml");

	//// OutputSink
	this->context->output->set_enabled(this->settings.output.enable_output);
	this->context->output->set_console_verbosity(this->settings.output.console_verbosity);
	this->context->output->set_file_verbosity(this->settings.output.file_verbosity);
	this->context->output->open_file(filename + ".log");
	this->context->output->set_root_tab(1);

	// Initialize callbacks
	this->callbacks = Callbacks::create(this->context);

	// Set simulation parameters
	this->dt = this->settings.simulation.max_time_step_size;
	this->gravity = this->settings.simulation.gravity;

	// Create global potential
	this->global_potential = GlobalPotential::create();


	// Print settings
	this->context->output->print_with_new_line("================================== Settings ==============================");
	this->context->output->print(this->settings.as_string());
}
bool Stark::run(double duration, std::function<void()> callback)
{
	const double begin_time = this->current_time;

	// Check functions
	auto check_simulation_time = [&]()
	{
		if (this->current_time > this->settings.execution.end_simulation_time) {
			this->context->output->print_with_new_line("Simulation time exceeded. Exiting simulation.");
			return false;
		}
		return true;
	};
	auto check_duration = [&]()
	{
		if ((this->current_time - begin_time) > duration) {
			this->context->output->print_with_new_line("Duration time exceeded. Exiting simulation.");
			return false;
		}
		return true;
	};
	auto check_frame = [&]()
	{
		if (this->current_frame > this->settings.execution.end_frame) {
			this->context->output->print_with_new_line("Frame count exceeded. Exiting simulation.");
			return false;
		}
		return true;
	};
	auto check_execution_time = [&](double t0)
	{
		if ((omp_get_wtime() - t0) > this->settings.execution.allowed_execution_time) {
			this->context->output->print_with_new_line("Execution time exceeded. Exiting simulation.");
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
	auto& logger = this->context->logger;
	auto& output = this->context->output;

	// Initialize
	if (!this->is_init) {
		auto t_ = this->context->logger->time("initialization");
		this->_initialize();
	}

	// Check if the simulation should continue
	if (!this->callbacks->run_should_continue_execution()) {
		output->print_with_new_line("Simulation interrupted by user.", Verbosity::Minimal);
		return false;  // Exit simulation
	}

	// Time step begin
	if (output->get_console_verbosity() != Verbosity::Minimal) {
		output->print_with_new_line(fmt::format("{}. dt: {:5.2f} ms | ", this->current_time_step, 1000.0 * this->dt), Verbosity::Summary);
	}
	this->callbacks->run_before_time_step();

	// Use Newton's Method to solve the time step update
	const double t0 = omp_get_wtime();
	SolverReturn newton = this->newton->solve();

	// Time step ended with success
	if (newton == SolverReturn::Successful) {

		// Actually move the simulation forward with the solution
		this->callbacks->run_on_time_step_accepted();
		this->callbacks->run_after_time_step();
		this->current_time += this->dt;
		this->current_time_step++;

		// Adaptive time step size
		this->dt = std::min(this->settings.simulation.max_time_step_size, this->dt * this->settings.simulation.time_step_size_success_multiplier);

		// Output
		//// Print
		const double runtime = omp_get_wtime() - t0;
		const double cr = runtime / this->dt;
		auto stats = this->newton->get_last_solve_stats();
		if (output->get_console_verbosity() != Verbosity::Minimal) {
			if (output->get_console_verbosity() != Verbosity::Summary) { // For summary, just write at the cursor
				output->print_new_line();
				output->print("             "); // So the summary lines up
			}
			output->print(fmt::format(
				"#newton: {:2d} | ph: {:4.1f}% | #CG/newton: {:4d} | ls (cap|max|inv|bt): {:2d}|{:2d}|{:2d}|{:2d}| runtime: {:6.1f} ms | cr: {:6.1f}", 
				stats.newton_iterations, 
				100.0 * stats.projected_hessians_ratio, 
				(stats.newton_iterations > 0) ? stats.cg_iterations / stats.newton_iterations : 0,
				stats.ls_cap_iterations, stats.ls_max_iterations, stats.ls_inv_iterations, stats.ls_bt_iterations, 
				1000.0 * runtime, cr), 
				Verbosity::Summary);
		}

		// Log
		logger->append("dt", this->dt);
		logger->append("time", this->current_time);
		logger->append("frame", this->current_frame);
		logger->add("time_steps", 1);
		logger->set("avg dt", this->current_time / this->current_time_step);

		// Frames
		if (this->settings.output.enable_frame_writes) { 
			this->_write_frame();
		}

		// Log
		if (this->context->logger->time_since_last_write() > 10.0) {
			this->context->logger->save_to_disk();
		}

		// Return
		return true;  // Successful (and taken) time step
	}

	// Time step failure: do not advance time. Adjust parameters and signal the caller to retry.
	else {
		// Output
		const double runtime = omp_get_wtime() - t0;
		output->print("\n", Verbosity::Medium);
		this->context->logger->add("failed_steps", runtime);

		// Failure due to invalid converged state (e.g. contact penetration):
		// the relevant callback has already hardened the offending parameter (e.g. contact stiffness).
		// Signal the caller to retry the same time step with the new parameters.
		if (newton == SolverReturn::InvalidConvergedState || newton == SolverReturn::TooManyInvalidIntermediateIterations) {
			return true;  // Not successful, not taken (state not updated), but should retry with new parameters
		}

		// Failure due to the time step being too difficult to solve:
		// halve dt and signal the caller to retry.
		else {
			if (!this->settings.simulation.use_adaptive_time_step) {
				output->print_with_new_line("settings.simulation.use_adaptive_time_step is false. Exiting simulation.\n", Verbosity::Summary);
				return false;  // Not successful, not taken, and should not retry (exit simulation)
			}

			this->dt /= 2.0;
			if (this->dt < this->settings.simulation.time_step_size_lower_bound) {
				output->print_with_new_line(fmt::format("Adaptive time step size out of bounds ({:.e}). Exiting simulation.\n", this->settings.simulation.time_step_size_lower_bound), Verbosity::Summary);
				return false; // Not successful, not taken, and should not retry (exit simulation)
			}
		}
	}

	return true;  // avoids compiler warning about no return, but this line should never be reached
}
std::string Stark::get_frame_path(std::string name) const
{
	return this->settings.output.output_directory + "/" + this->settings.output.simulation_name + "_" + name + "_" + std::to_string(this->current_frame);
}
void stark::core::Stark::print()
{
	this->context->output->print_new_line(Verbosity::Minimal);
	this->context->output->print_with_new_line("================================== Summary ===============================");

	auto& logger = this->context->logger;
	auto* out = this->context->output.get();

	// Guard: if no successful time steps, skip detailed stats
	if (this->current_time_step == 0) {
		out->print_with_new_line("  No completed time steps.");
		out->print_new_line();
		logger->save_to_disk();
		return;
	}

	const int time_steps = std::max(logger->get_int("time_steps"), 1);
	auto dt_stats = logger->get_stats("dt");

	// Info
	out->print_with_new_line("Info");
	out->print_with_new_line(fmt::format("  Name:               {}", this->settings.output.simulation_name));
	out->print_with_new_line(fmt::format("  Simulation time:    {:.3f} s", this->current_time));
	out->print_with_new_line(fmt::format("  ndofs:              {}", this->global_potential->get_total_n_dofs()));
	out->print_with_new_line(fmt::format("  Frames:             {}", this->current_frame));
	out->print_with_new_line(fmt::format("  Time steps:         {}", logger->get_int("time_steps")));
	out->print_with_new_line(fmt::format("  dt [ms]:            avg: {:.1f} | min: {:.1f} | max: {:.1f}", 1000.0*dt_stats.avg, 1000.0*dt_stats.min, 1000.0*dt_stats.max));

	// Solve + Runtime (delegated to NewtonsMethod)
	this->newton->print_summary();

	// Save final YAML log
	logger->save_to_disk();
}

void Stark::_initialize()
{
	this->is_init = true;

	if (this->global_potential->get_total_n_dofs() == 0) {
		this->context->output->print("Stark::_initialize() error: No degrees of freedom. Exiting simulation.", Verbosity::Summary);
		exit(-1);
	}

    // Newton Solver (context already created in constructor)
	context->output->print_with_new_line("==================================== SymX ================================");
    this->newton = NewtonsMethod::create(this->global_potential, this->context, this->callbacks->newton);  // Compilation occurs here
	this->newton->settings = this->settings.newton;

	// Stark banner
	this->context->output->print_new_line(Verbosity::Minimal);
	this->context->output->print_with_new_line("==================================== STARK ===============================");

	// Write frame zero
	this->_write_frame();

	// Callbacks
	this->callbacks->run_before_simulation();

	// Check that the initial state is valid
	if (!this->callbacks->newton->run_is_initial_state_valid()) {
		this->context->output->print("Initial state is not valid. Exiting simulation.", Verbosity::Summary);
		exit(-1);
	}
}
void Stark::_write_frame()
{
	auto write_frame_impl = [&]()
		{
			if (this->settings.output.fps != 0) {
				this->callbacks->run_write_frame();
			}
			this->context->output->print_with_new_line(fmt::format("[Frame: {:d}] Time: {:.3f} s", this->current_frame, this->current_time));
			this->current_frame++;
		};

	if (this->settings.output.fps < 0) {  // Write every time step
		write_frame_impl();
	}
	else if (this->current_frame == 0) {
		write_frame_impl();
		this->next_frame_time += 1.0 / (double)this->settings.output.fps;
	}
	else {
		while (this->current_time > this->next_frame_time + 100.0*std::numeric_limits<double>::epsilon()) {
			write_frame_impl();
			this->next_frame_time += 1.0 / (double)this->settings.output.fps;
		}
	}
}
