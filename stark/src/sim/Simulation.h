#pragma once
#include <string>

#include <Eigen/Dense>
#include <symx>

#include "AdaptiveParameter.h"

namespace stark
{
	class Simulation
	{
	public:
		struct Callbacks
		{
			std::vector<std::function<void(Simulation& sim)>> before_time_step;
			std::vector<std::function<void(Simulation& sim)>> before_energy_evaluation;
			std::vector<std::function<void(Simulation& sim)>> after_energy_evaluation;
			std::vector<std::function<void(Simulation& sim)>> after_time_step;
			std::vector<std::function<double(Simulation& sim)>> max_allowed_step;
			std::vector<std::function<bool(Simulation& sim)>> is_state_valid;
			std::vector<std::function<void(Simulation& sim)>> write_frame;
		};

		/* Fields*/
		symx::GlobalEnergy global_energy;
		Callbacks callbacks;

		// Global parameters
		AdaptiveParameter time_step;
		AdaptiveParameter collision_stiffness;
		Eigen::Vector3d gravity;
		double boundary_conditions_stiffness = 1e6;

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
}
