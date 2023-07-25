#pragma once
#include <string>

#include <Eigen/Dense>
#include <symx>

#include "Settings.h"
#include "Callbacks.h"
#include "AdaptiveParameter.h"
#include "NewtonsMethod.h"
#include "../utils/Console.h"
#include "../utils/Logger.h"

namespace stark
{
	class Simulation
	{
	public:
		// ---------------------------------------------------------------------------------
		struct Output
		{
		public:
			std::string get_vtk_path(std::string name) const
			{
				return this->settings->output.output_directory + "/" + this->settings->output.simulation_name + "_" + name + "_" + std::to_string(this->frame_it) + ".vtk";
			}
		private:
			Settings* settings = nullptr;
			int frame_it = 0;
			double next_frame_time = -1e-12;
		};
		// ---------------------------------------------------------------------------------


		/* Fields */
		symx::GlobalEnergy global_energy;
		NewtonsMethod newton;
		Callbacks callbacks;
		utils::Console console;
		utils::Logger logger;
		Settings settings;

		double current_time = 0.0;
		int current_frame = 0;



		Output output;

		/* Methods */
		void setup(Settings& settings);
		bool run_one_step();
		bool run(const double duration, std::function<void()> callback = nullptr);

	private:
		bool is_init = false;
		void _check();
		void _initialize_symx();
		void _write_frame();
		void _print_header();
	};
}
