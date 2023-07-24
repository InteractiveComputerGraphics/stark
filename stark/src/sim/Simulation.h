#pragma once
#include <string>

#include <Eigen/Dense>
#include <symx>

#include "Callbacks.h"
#include "AdaptiveParameter.h"
#include "NewtonsMethod.h"

namespace stark
{
	class Simulation
	{
	public:
		// ---------------------------------------------------------------------------------
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
			std::string codegen_directory = "";
			int fps = 30;
			int frame_it = 0;
			double next_frame_time = -1e-12;
			Logger logger; // TODO: https://github.com/gabime/spdlog
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
		symx::GlobalEnergy global_energy;
		NewtonsMethod minimizer;
		Callbacks callbacks;
		Parameters parameters;
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
