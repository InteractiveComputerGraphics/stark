#include "Stark.h"

#include <iostream>

#include <JanBenderUtilities/FileSystem.h>


stark::Stark::Stark(const Settings& settings)
{
	this->settings = settings;

	// Check mandatory arguments
	if (settings.output.codegen_directory == "") {
		std::cout << "stark::Stark::setup() error: Settings.output.codegen_directory must be set" << std::endl;
		exit(-1);
	}
	if (settings.output.output_directory == "") {
		std::cout << "stark::Stark::setup() error: Settings.output.output_directory must be set" << std::endl;
		exit(-1);
	}
	if (settings.output.simulation_name == "") {
		std::cout << "stark::Stark::setup() error: Settings.output.simulation_name must be set" << std::endl;
		exit(-1);
	}

	// Create folders
	int status = JanBenderUtilities::FileSystem::makeDirs(settings.output.codegen_directory);
	if (status != 0) {
		std::cout << "stark::Stark::setup() error: Cannot create folder " << settings.output.codegen_directory << std::endl;
		exit(-1);
	}
	status = JanBenderUtilities::FileSystem::makeDirs(settings.output.output_directory);
	if (status != 0) {
		std::cout << "stark::Stark::setup() error: Cannot create folder " << settings.output.output_directory << std::endl;
		exit(-1);
	}

	// Initialize Consoles and Loggers
	const std::string ending = "_" + settings.output.simulation_name + "__" + settings.output.time_stamp + ".txt";
	this->console.initialize(settings.output.output_directory + "/console" + ending, settings.output.console_verbosity, settings.output.console_output_to);
	this->logger.set_path(settings.output.output_directory + "/logger" + ending);
	this->newton.line_search_debug_logger.set_path(settings.output.output_directory + "/line_search" + ending);

	// Print settings
	this->console.print(this->settings.as_string(), Verbosity::TimeSteps);
}
bool stark::Stark::run(std::function<void()> callback)
{
	const double t0 = omp_get_wtime();
	while (
		this->current_time < this->settings.execution.end_simulation_time && 
		this->current_frame < this->settings.execution.end_frame && 
		(omp_get_wtime() - t0) < this->settings.execution.allowed_execution_time) 
	{
		if (callback != nullptr) { callback(); }
		bool success = this->run_one_step();
		if (!success) {
			break;
		}
	}

	// Final print
	//// Info
	this->console.print("\nInfo\n", Verbosity::Frames);
	this->console.print(fmt::format("\t # time_steps: {:d}\n", this->logger.ints["time_steps"]), Verbosity::Frames);
	this->console.print(fmt::format("\t # newton/time_steps: {:.1f}\n", (double)this->logger.ints["newton_iterations"]/(double)this->logger.ints["time_steps"]), Verbosity::Frames);
	if (!this->settings.newton.use_direct_linear_solve) {
		this->console.print(fmt::format("\t # CG_iterations/newton: {:.1f}\n", (double)this->logger.ints["CG_iterations"]/(double)this->logger.ints["newton_iterations"]), Verbosity::Frames);
	}
	this->console.print(fmt::format("\t # line_search/newton: {:.1f}\n", (double)this->logger.ints["line_search_iterations"]/(double)this->logger.ints["newton_iterations"]), Verbosity::Frames);
	this->console.print(fmt::format("\t avg dt: {:.6f} ms\n", 1000.0*this->logger.doubles["avg dt"]), Verbosity::Frames);
	this->console.print(fmt::format("\t cr: {:.1f}\n", this->logger.doubles["cr"]), Verbosity::Frames);

	//// Runtime
	this->console.print("Runtime\n", Verbosity::Frames);
	this->console.print(fmt::format("\t step: {:.3f} s\n", this->logger.doubles["step"]), Verbosity::Frames);
	this->console.print(fmt::format("\t evaluate_E_grad_hess: {:.3f} s\n", this->logger.doubles["evaluate_E_grad_hess"]), Verbosity::Frames);
	this->console.print(fmt::format("\t evaluate_E_grad: {:.3f} s\n", this->logger.doubles["evaluate_E_grad"]), Verbosity::Frames);
	this->console.print(fmt::format("\t evaluate_E: {:.3f} s\n", this->logger.doubles["evaluate_E"]), Verbosity::Frames);
	if (!this->settings.newton.use_direct_linear_solve) {
		this->console.print(fmt::format("\t CG: {:.3f} s\n", this->logger.doubles["CG"]), Verbosity::Frames);
	}
	else {
		this->console.print(fmt::format("\t directLU: {:.3f} s\n", this->logger.doubles["directLU"]), Verbosity::Frames);
	}
	this->console.print(fmt::format("\t before_energy_evaluation: {:.3f} s\n", this->logger.doubles["before_energy_evaluation"]), Verbosity::Frames);
	this->console.print(fmt::format("\t after_energy_evaluation: {:.3f} s\n", this->logger.doubles["after_energy_evaluation"]), Verbosity::Frames);
	this->console.print(fmt::format("\t is_state_valid: {:.3f} s\n", this->logger.doubles["is_state_valid"]), Verbosity::Frames);
	this->console.print(fmt::format("\t failed steps: {:.3f} s\n", this->logger.doubles["failed_steps"]), Verbosity::Frames);
	return true;
}
bool stark::Stark::run_one_step()
{
	if (!this->is_init) {
		this->_initialize();
	}

	this->console.print(fmt::format("\t dt: {:.6f} ms | kc: {:.1e} | ", 1000.0 * this->settings.simulation.adaptive_time_step.value, this->settings.contact.adaptive_contact_stiffness.value), Verbosity::TimeSteps);

	this->callbacks.run_before_time_step();

	// Time step
	if (this->global_energy.get_total_n_dofs() > 0) {
		const double t0 = omp_get_wtime();
		NewtonError err = this->newton.solve(this->global_energy, this->callbacks, this->settings, this->console, this->logger);
		const double t1 = omp_get_wtime();
		const double runtime = t1 - t0;

		if (err == NewtonError::Successful) {
			const double cr = runtime / this->settings.simulation.adaptive_time_step.value;

			this->console.print(fmt::format(" | runtime: {:.0f} ms | cr: {:.1f}\n", 1000.0 * runtime, cr), Verbosity::TimeSteps);
			this->logger.add("step", runtime);
			this->logger.append_to_series("step", runtime);
			this->logger.append_to_series("cr", cr);
			this->logger.append_to_series("dt", this->settings.simulation.adaptive_time_step.value);
			this->logger.append_to_series("time", this->current_time);
			this->logger.append_to_series("frame", this->current_frame);
			this->logger.set("avg dt", this->current_time / (double)this->logger.ints["time_steps"]);
			this->logger.set("cr", this->logger.doubles["step"] / this->current_time);

			this->callbacks.run_after_time_step();

			this->_write_frame();
			this->settings.contact.adaptive_contact_stiffness.successful_iteration();
			this->settings.simulation.adaptive_time_step.successful_iteration();
			this->logger.add_to_counter("time_steps", 1);
			this->current_time += this->settings.simulation.adaptive_time_step.value;
			return true;
		}
		else {
			this->logger.add("failed_steps", runtime);
			if (err == NewtonError::InvalidConfiguration) {
				const bool out_of_bounds = this->settings.contact.adaptive_contact_stiffness.failed_iteration();
				if (out_of_bounds) {
					this->console.print(fmt::format("Adaptive contact stiffness out of bounds ({:.e}). Exiting simulation.\n", this->settings.contact.adaptive_contact_stiffness.value), Verbosity::Frames);
					return false;
				}
			}
			else {
				const bool out_of_bounds = this->settings.simulation.adaptive_time_step.failed_iteration();
				if (out_of_bounds) {
					this->console.print(fmt::format("Adaptive time step size out of bounds ({:.e}). Exiting simulation.\n", this->settings.simulation.adaptive_time_step.value), Verbosity::Frames);
					return false;
				}
			}
			return this->run_one_step();
		}
	}
	else {
		return true;
	}
}
std::string stark::Stark::get_frame_path(std::string name) const
{
	return this->settings.output.output_directory + "/" + this->settings.output.simulation_name + "_" + name + "_" + std::to_string(this->current_frame);
}

void stark::Stark::_initialize()
{
	this->is_init = true;

	// SymX
	//// Compile
	const std::string symx_print = this->global_energy.compile(this->settings.output.codegen_directory, this->settings.execution.n_threads, this->settings.debug.symx_suppress_compiler_output);
	this->console.print(symx_print, Verbosity::Frames);

	//// Project to PD
	if (this->settings.newton.project_to_PD) {
		this->global_energy.set_project_to_PD(true);
	}

	//// Print ndofs
	this->console.print("\nDegrees of freedom:", Verbosity::Frames);
	for (int i = 0; i < (int)this->global_energy.dof_labels.size(); i++) {
		this->console.print(fmt::format("\n\t {} {:d}", this->global_energy.dof_labels[i], this->global_energy.dof_ndofs[i]()), Verbosity::Frames);
	}
	this->console.print("\n\n", Verbosity::Frames);

	// Write frame zero
	this->_write_frame();
}
void stark::Stark::_write_frame()
{
	auto write_frame_impl = [&]() 
	{
		this->callbacks.run_write_frame();
		this->console.print(fmt::format("Frame: {:d}. Time: {:.3f} s.\n", this->current_frame, this->current_time), Verbosity::Frames);
		this->current_frame++;
		this->logger.save_to_disk();
	};

	if (this->settings.output.fps < 0) {
		write_frame_impl();
	}
	else {
		while (this->current_time > this->next_frame_time) {
			write_frame_impl();
			this->next_frame_time += 1.0/(double)this->settings.output.fps;
		}
	}
}

