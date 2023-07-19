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
		// ---------------------------------------------------------------------------------
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
		struct Parameters
		{
			AdaptiveParameter time_step;
			AdaptiveParameter collision_stiffness;
			Eigen::Vector3d gravity;
			double boundary_conditions_stiffness = 1e6;
		};
		struct Output
		{
			std::string sim_name = "";
			std::string directory = "";
			int fps = 30;
			int frame_it = 0;
			double next_frame_time = -1e-12;
			std::string get_vtk_path(std::string name) const
			{
				return this->directory + "/" + this->sim_name + "_" + name + "_" + std::to_string(this->frame_it) + ".vtk";
			}
		};
		struct Time
		{
			double current = 0.0;
			double end = 0.0;
			double execution_left = std::numeric_limits<double>::max();
		};
		struct Options
		{
			int print_verbosity = 1;
			int n_threads = -1;
		};
		// ---------------------------------------------------------------------------------


		/* Fields*/
		std::string codegen_directory = "";
		symx::GlobalEnergy global_energy;
		Parameters parameters;
		Callbacks callbacks;
		Time time;
		Output output;
		Options options;

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
