#pragma once
#include <string>
#include <functional>
#include <limits>

#include "Settings.h"
#include "Callbacks.h"
#include "NewtonsMethod.h"
#include "Console.h"
#include "Logger.h"

namespace stark::core
{
	class Stark
	{
	public:
		/* Fields */
		symx::GlobalEnergy global_energy;
		NewtonsMethod newton;
		Callbacks callbacks;
		Console console;
		Logger logger;
		Settings settings;

		// Frame output
		double current_time = 0.0;
		int current_frame = 0;

		/* Methods */
		Stark(const Settings& settings);
		bool run_one_step();
		bool run(std::function<void()> callback = nullptr);
		std::string get_frame_path(std::string name) const;

	private:
		/* Fields */
		bool is_init = false;
		double next_frame_time = -std::numeric_limits<double>::epsilon();

		/* Methods */
		void _initialize();
		void _write_frame();
	};
}
