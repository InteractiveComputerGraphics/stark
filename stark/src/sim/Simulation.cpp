#include "Simulation.h"

#include <iostream>

#include <JanBenderUtilities/FileSystem.h>

void stark::Simulation::setup(Settings& settings)
{
	this->settings = settings;

	// Check mandatory arguments
	if (settings.output.codegen_directory == "") {
		std::cout << "stark::Simulation::setup() error: Settings.output.codegen_directory must be set" << std::endl;
		exit(-1);
	}
	if (settings.output.output_directory == "") {
		std::cout << "stark::Simulation::setup() error: Settings.output.output_directory must be set" << std::endl;
		exit(-1);
	}
	if (settings.output.simulation_name == "") {
		std::cout << "stark::Simulation::setup() error: Settings.output.simulation_name must be set" << std::endl;
		exit(-1);
	}

	// Create folders
	int status = JanBenderUtilities::FileSystem::makeDirs(settings.output.codegen_directory);
	if (status != 0) {
		std::cout << "stark::Simulation::setup() error: Cannot create folder " << settings.output.codegen_directory << std::endl;
		exit(-1);
	}
	status = JanBenderUtilities::FileSystem::makeDirs(settings.output.output_directory);
	if (status != 0) {
		std::cout << "stark::Simulation::setup() error: Cannot create folder " << settings.output.output_directory << std::endl;
		exit(-1);
	}

	// Initialize Consoles and Loggers
	this->console.initialize(settings.output.output_directory + "/console.txt", settings.output.console_verbosity, settings.output.console_output_to);
	this->logger.set_path(settings.output.output_directory + "/logger.txt");
	this->newton.line_search_debug_logger.set_path(settings.output.output_directory + "/line_search.txt");
}
void stark::Simulation::init()
{
	this->_init();
	this->_print_header();
	this->simws.compile();
	this->_write_frame();
	this->is_init = true;
}
bool stark::Simulation::run_one_step()
{
	this->output.console.print("Simulation::run_one_step\n", 3);

	if (!this->is_init) {
		std::cout << "sym error: Call Simulation.init() before Simulation.run_one_step()." << std::endl;
		return false;
	}
	this->time.compute_current_max_dt();

	Output::logger->start_timing("total");
	this->output.console.print("\tdt: " + std::to_string(1000.0 * this->time.dt) + " ms", 1);
	this->output.console.print("  |  log10(k): " + to_string_with_precision(std::log10(this->adaptive_contact_stiffness.value), 2), 1);

	for (auto& ps : this->physical_systems) {
		ps->prepare_time_step(this->simws);  // This just sets initial guess to zero
	}

	const double t0 = omp_get_wtime();
	Output::logger->start_timing("step");
	NewtonError err = this->minimizer.solve(this->simws);
	Output::logger->stop_timing_add("step");
	const double t1 = omp_get_wtime();
	this->output.console.print("  |  runtime: " + to_string_with_precision(1000.0 * (t1 - t0), 2) + " ms\n", 1);

	if (err == NewtonError::InvalidConfiguration) {
		this->adaptive_contact_stiffness.failed_iteration();
		return this->run_one_step();
	}
	else if (err != NewtonError::Successful) {
		this->time.reduce_time_step_size();
		if (this->time.dt < this->time.dt_min) {
			this->output.console.print("Min time step size reached (" + std::to_string(this->time.dt) + "). Exiting simulation.\n", 1);
			Output::logger->save_to_disk();
			return false;
		}
		return this->run_one_step();
	}
	Output::logger->stop_timing_add("total");
	Output::logger->add("dt", this->time.dt);
	Output::logger->add("time", this->time.current);

	for (auto& ps : this->physical_systems) {
		ps->postprocess_time_step(this->simws);
	}

	this->_write_frame();
	this->time.successful_time_step();
	this->adaptive_contact_stiffness.successful_iteration();
	return true;
}
bool stark::Simulation::run(const double duration, std::function<void()> callback)
{
	if (this->print_verbosity >= 3) { std::cout << "Simulation::init" << std::endl; }

	this->init();
	const double t0 = omp_get_wtime();
	while (this->time.current < duration && (omp_get_wtime() - t0) < this->timeout) {
		if (callback != nullptr) { callback(); }
		this->run_one_step();
	}

	Output::logger->add("newton_iterations: ", this->minimizer.it_count);
	std::cout << "newton_iterations: " << this->minimizer.it_count << std::endl;
	std::cout << "assemble_linear_system: " << Output::logger->timers["assemble_linear_system"] << " s" << std::endl;
	std::cout << "CG: " << Output::logger->timers["CG"] << " s" << std::endl;
	std::cout << "step: " << Output::logger->timers["step"] << " s" << std::endl;
	std::cout << "assemble_gradient: " << Output::logger->timers["assemble_gradient"] << " s" << std::endl;
	std::cout << "line_search: " << Output::logger->timers["line_search"] << " s" << std::endl;
	std::cout << "assemble_E: " << Output::logger->timers["assemble_E"] << " s" << std::endl;
	std::cout << "total: " << Output::logger->timers["total"] << " s" << std::endl;
	return true;
}

void stark::Simulation::_initialize_symx()
{
	// Compile
	this->global_energy.compile(this->settings.output.codegen_directory, this->settings.execution.n_threads, this->settings.output.suppress_symx_compiler_output);
	how do I get the output?

	// Project to PD
	for (auto& energy : this->global_energy.energies) {
		energy->project_to_PD = energy->project_to_PD || this->project_to_PD;
	}
}
void stark::Simulation::_write_frame()
{
	this->output.console.print("Simulation::_write_frame\n", 3);

	const std::string base_path = this->output_directory + "/" + this->name + "_";
	if (this->fps < 0.0) {
		Output::logger->save_to_disk();
		for (auto& ps : this->physical_systems) {
			ps->write_frame(base_path + ps->get_name() + "_" + std::to_string(this->working_on_frame_number) + ".vtk");
		}
		this->output.console.print(std::to_string(this->working_on_frame_number) + ".vtk [" + std::to_string(this->time.current) + " s]\n", 1);
		this->working_on_frame_number++;
	}
	else {
		while (this->time.current > this->next_frame_time) {
			Output::logger->save_to_disk();
			for (auto& ps : this->physical_systems) {
				ps->write_frame(base_path + ps->get_name() + "_" + std::to_string(this->working_on_frame_number) + ".vtk");
			}
			this->output.console.print(std::to_string(this->working_on_frame_number) + ".vtk [" + std::to_string(this->time.current) + " s]\n", 1);
			this->next_frame_time += 1.0/(double)this->fps;
			this->working_on_frame_number++;
		}
	}
}
void stark::Simulation::_print_header()
{
	this->output.console.print("Simulation name: " + this->name + "\n", 0);
	this->output.console.print("Codegen directory: " + this->codegen_directory + "\n", 0);
	this->output.console.print("Output directory: " + this->output_directory + "\n", 0);
	this->output.console.print("Number of threads: " + std::to_string(this->n_threads) + "\n", 0);
	this->output.console.print("Minimizer: " + this->minimizer.get_name() + "\n", 0);

	std::string physical_systems = "";
	for (auto& ps : this->physical_systems) {
		physical_systems += ps->get_name() + ", ";
	}
	this->output.console.print("Physical Systems: " + physical_systems + "\n", 0);

	std::string dofs = "Degrees of freedom:";
	for (auto& array : this->simws.arrays) {
		if (array->is_dof) {
			dofs += "\n\t" + array->name + ": " + std::to_string(array->size);
		}
	}
	this->output.console.print(dofs + "\n", 0);
}

