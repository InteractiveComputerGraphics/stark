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
	class Stark
	{
	public:
		/* Fields */
		symx::GlobalEnergy global_energy;
		NewtonsMethod newton;
		Callbacks callbacks;
		utils::Console console;
		utils::Logger logger;
		Settings settings;

		// Frame output
		double current_time = 0.0;
		int current_frame = 0;


		/* Methods */
		Stark(const Settings& settings);
		bool run_one_step();
		bool run(std::function<void()> callback = nullptr);
		std::string get_vtk_path(std::string name) const;

	private:
		/* Fields */
		bool is_init = false;
		double next_frame_time = -std::numeric_limits<double>::epsilon();

		/* Methods */
		void _initialize();
		void _write_frame();
	};
}
