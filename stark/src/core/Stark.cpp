#include "Stark.h"

#include <iostream>
#include <system_error>
#include <filesystem>

#include <Eigen/Dense>
#include <symx>
#include <fmt/format.h>

using namespace stark::core;
using namespace symx;

Stark::Stark(const Settings& settings)
	: settings(settings)
{
	// Set
	this->dt = this->settings.simulation.max_time_step_size;
	this->gravity = this->settings.simulation.gravity;

	// Create global potential
	this->global_potential = GlobalPotential::create();

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
	// this->console.initialize(settings.output.output_directory + "/console" + ending, settings.output.console_verbosity, settings.output.console_output_to);
	this->logger.set_path(settings.output.output_directory + "/logger" + ending);

	// Create SymX context and configure OutputSink
	this->context = Context::create();
	this->context->compilation_directory = this->settings.output.codegen_directory;
	this->context->n_threads = this->settings.execution.n_threads;

	this->output = this->context->output;
	this->output->set_enabled(this->settings.output.enable_output);
	this->output->set_verbosity(this->settings.output.verbosity);
	this->output->set_output_to(this->settings.output.output_to);
	this->output->open_file(this->settings.output.output_directory + "/console" + ending);
	this->output->set_root_tab(1);

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
			this->output->print_with_new_line("Simulation time exceeded. Exiting simulation.");
			return false;
		}
		return true;
	};
	auto check_duration = [&]()
	{
		if ((this->current_time - begin_time) > duration) {
			this->output->print_with_new_line("Duration time exceeded. Exiting simulation.");
			return false;
		}
		return true;
	};
	auto check_frame = [&]()
	{
		if (this->current_frame > this->settings.execution.end_frame) {
			this->output->print_with_new_line("Frame count exceeded. Exiting simulation.");
			return false;
		}
		return true;
	};
	auto check_execution_time = [&](double t0)
	{
		if ((omp_get_wtime() - t0) > this->settings.execution.allowed_execution_time) {
			this->output->print_with_new_line("Execution time exceeded. Exiting simulation.");
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

	// Check if the simulation should continue
	if (!this->callbacks.run_should_continue_execution()) {
		this->logger.set("success", 0);
		this->output->print_with_new_line("Simulation interrupted by user.");
		return false;
	}

	// Time step begin
	if (this->output->get_verbosity() != Verbosity::Silent) {
		this->output->print_with_new_line(fmt::format("dt: {:5.2f} ms | ", 1000.0 * this->dt), Verbosity::Summary);
	}
	this->callbacks.run_before_time_step();

	// Use Newton's Method to solve the time step update
	const double t0 = omp_get_wtime();
	SolverReturn newton = this->newton->solve();

	// Time step ended with success
	if (newton == SolverReturn::Successful) {

		// After successful time step
		this->callbacks.run_on_time_step_accepted(); // Sets solution. x0 = x1. Exits minimization.
		this->callbacks.run_after_time_step();  // Can use the converged state x1, v1.
		this->current_time += this->dt;

		// Adaptive time step size
		this->dt = std::min(this->settings.simulation.max_time_step_size, this->dt * this->settings.simulation.time_step_size_success_multiplier);

		// Output
		//// Print
		const double runtime = omp_get_wtime() - t0;
		const double cr = runtime / this->dt;
		auto stats = this->newton->get_last_solve_stats();
		if (this->output->get_verbosity() == Verbosity::Silent) {
			this->output->print_with_new_line(fmt::format("dt: {:5.2f} ms | ", 1000.0 * this->dt), Verbosity::Summary);
		}
		this->output->print(fmt::format(
			"#newton: {:2d} | ph: {:4.1f}% | #CG/newton: {:4d} | ls hit: {:2d} | ls bt: {:2d} | runtime: {:6.1f} ms | cr: {:6.1f}", 
			stats.newton_iterations, 100.0 * stats.projected_hessians_ratio, stats.cg_iterations / stats.newton_iterations, stats.max_step_iterations, stats.line_search_iterations, 1000.0 * runtime, cr), Verbosity::Summary);

		// Log
		this->logger.append_to_series("cr", cr);
		this->logger.append_to_series("dt", this->dt);
		this->logger.append_to_series("time", this->current_time);
		this->logger.append_to_series("frame", this->current_frame);
		this->logger.add_to_counter("time_steps", 1);
		this->logger.set("avg dt", this->current_time / (double)this->logger.get_int("time_steps"));
		this->logger.set("cr", this->logger.get_double("total") / this->current_time);

		// Frames
		this->_write_frame();

		// Return
		success = true;
	}

	// Time step failure
	else {
		// Output
		const double runtime = omp_get_wtime() - t0;
		this->output->print("\n", Verbosity::Step);
		this->logger.add("failed_steps", runtime);

		// The failure was due to loose stiffnesses
		if (newton == SolverReturn::InvalidConvergedState || newton == SolverReturn::InvalidIntermediateConfiguration) {
			// Tighter stiffess should be already when measured during Newton's solve.
			// Do nothing, just run the time step again.
		}

		// The failure was due to the time step being too tough. Adapt the time step size and run the time step again.
		else {
			if (!this->settings.simulation.use_adaptive_time_step) {
				this->output->print("settings.simulation.adaptive_time_step is set to false. Exiting simulation.\n", Verbosity::Summary);
				return false;
			}

			this->dt /= 2.0;
			if (this->dt < this->settings.simulation.time_step_size_lower_bound) {
				this->output->print(fmt::format("Adaptive time step size out of bounds ({:.e}). Exiting simulation.\n", this->settings.simulation.time_step_size_lower_bound), Verbosity::Summary);
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
	this->context->output->print_new_line(Verbosity::Summary);
	this->context->output->print_with_new_line("================================== Summary ===============================");
	this->newton->get_log().print();
	std::cout << "TODO: Final Stark::print() implementation" << std::endl;

	// // Info
	// this->console.print("\nInfo\n", ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t # time_steps: {:d}\n", this->logger.get_int("time_steps")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t # newton/time_steps: {:.1f}\n", (double)this->logger.get_int("newton_iterations") / (double)this->logger.get_int("time_steps")), ConsoleVerbosity::Frames);
	// if (this->settings.newton.linear_system_solver == LinearSystemSolver::CG) {
	// 	this->console.print(fmt::format("\t # CG_iterations/newton: {:.1f}\n", (double)this->logger.get_int("CG_iterations") / (double)this->logger.get_int("newton_iterations")), ConsoleVerbosity::Frames);
	// }
	// this->console.print(fmt::format("\t # line_search/newton: {:.1f}\n", (double)this->logger.get_int("line_search_iterations") / (double)this->logger.get_int("newton_iterations")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t avg dt: {:.6f} ms\n", 1000.0 * this->logger.get_double("avg dt")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t cr: {:.1f}\n", this->logger.get_double("cr")), ConsoleVerbosity::Frames);

	// // Runtime
	// this->console.print("Runtime\n", ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t total: {:.3f} s\n", this->logger.get_double("total")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t evaluate_E_grad_hess: {:.3f} s\n", this->logger.get_double("evaluate_E_grad_hess")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t evaluate_E_grad: {:.3f} s\n", this->logger.get_double("evaluate_E_grad")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t evaluate_E: {:.3f} s\n", this->logger.get_double("evaluate_E")), ConsoleVerbosity::Frames);
	// if (this->settings.newton.linear_system_solver == LinearSystemSolver::CG) {
	// 	this->console.print(fmt::format("\t CG: {:.3f} s\n", this->logger.get_double("CG")), ConsoleVerbosity::Frames);
	// }
	// else if (this->settings.newton.linear_system_solver == LinearSystemSolver::DirectLU) {
	// 	this->console.print(fmt::format("\t directLU: {:.3f} s\n", this->logger.get_double("directLU")), ConsoleVerbosity::Frames);
	// }
	// this->console.print(fmt::format("\t before_energy_evaluation: {:.3f} s\n", this->logger.get_double("before_energy_evaluation")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t after_energy_evaluation: {:.3f} s\n", this->logger.get_double("after_energy_evaluation")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t is_intermediate_state_valid: {:.3f} s\n", this->logger.get_double("is_intermediate_state_valid")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t failed steps: {:.3f} s\n", this->logger.get_double("failed_steps")), ConsoleVerbosity::Frames);
	// this->console.print(fmt::format("\t write_frame: {:.3f} s\n", this->logger.get_double("write_frame")), ConsoleVerbosity::Frames);
}

void Stark::_initialize()
{
	this->is_init = true;

	if (this->global_potential->get_total_n_dofs() == 0) {
		this->output->print("Stark::_initialize() error: No degrees of freedom. Exiting simulation.", Verbosity::Summary);
		exit(-1);
	}

    // Newton Solver (context already created in constructor)
	context->output->print_with_new_line("==================================== SymX ================================");
    this->newton = NewtonsMethod::create(this->global_potential, this->context);  // Compilation occurs here
	this->newton->settings = this->settings.newton;
	this->newton->callbacks = this->callbacks.newton;

	// Print ndofs
	this->output->print_with_new_line("\nDegrees of freedom:");
	for (int set_i = 0; set_i < this->global_potential->get_n_dof_sets(); set_i++) {
		this->output->print_with_new_line(fmt::format("Set {}: {:d}", set_i, this->global_potential->get_n_dofs(set_i)), Verbosity::Summary);
	}
	this->output->print_with_new_line(fmt::format("Total: {:d}", this->global_potential->get_total_n_dofs()), Verbosity::Summary);

	// Stark banner
	this->output->print_new_line(Verbosity::Summary);
	context->output->print_with_new_line("==================================== STARK ===============================");

	// Write frame zero
	this->_write_frame();

	// Callbacks
	this->callbacks.run_before_simulation();

	// Check that the initial state is valid
	if (!this->callbacks.newton.run_is_initial_state_valid()) {
		this->output->print("Initial state is not valid. Exiting simulation.", Verbosity::Summary);
		exit(-1);
	}
}
void Stark::_write_frame()
{
	if (!this->settings.output.enable_output) { return; }

	auto write_frame_impl = [&]()
		{
			if (this->settings.output.fps != 0) {
				this->callbacks.run_write_frame();
			}
			this->output->print_with_new_line(fmt::format("[Frame: {:d}] Time: {:.3f} s", this->current_frame, this->current_time));
			this->current_frame++;
		};

	this->logger.start_timing("runtime_write_frame");
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
	this->logger.stop_timing_add("runtime_write_frame");
	this->logger.save_to_disk();
}
