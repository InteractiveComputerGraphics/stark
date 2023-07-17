#pragma once
#include <string>

#include <Eigen/Dense>
#include <symx>

#include "AdaptiveParameter.h"
#include "Callbacks.h"

namespace stark
{
	class Simulation
	{
	public:
		/* Fields*/
		symx::GlobalEnergy global_energy;
		Callbacks callbacks;

		// Global parameters
		AdaptiveParameter time_step;
		Eigen::Vector3d gravity;

		// Options
		std::string name = "";
		std::string codegen_directory = "";
		std::string output_directory = "";
		int print_verbosity = 1;
		int n_threads = -1;

		/* Methods */
		Simulation();
		void init();
		bool run_one_step();
		bool run(const double duration, std::function<void()> callback = nullptr);
		void setup(const std::string name, const std::string codegen_directory, const std::string output_directory);

	private:
		bool is_init = false;
		void _check();
		void _init();
		void _write_frame();
		void _print_header();
	};


	template<typename PhysicalSystemType>
	inline void Simulation::add_physical_system(PhysicalSystemType* physical_system)
	{
		this->physical_systems.push_back(physical_system);
	}
}