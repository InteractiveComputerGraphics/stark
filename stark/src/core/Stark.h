#pragma once
#include <string>
#include <functional>
#include <limits>

#include "Settings.h"
#include "Callbacks.h"
#include "NewtonsMethod.h"
#include "Console.h"
#include "Logger.h"
#include "EventDrivenScript.h"

namespace stark::core
{
	class Stark
	{
	public:
		/* Fields */
		const Settings settings;
		symx::GlobalEnergy global_energy;
		Callbacks callbacks;
		Console console;
		Logger logger;
		EventDrivenScript script;

		// Parameters
		double current_time = 0.0;
		int current_frame = 0;
		double dt = -1.0;
		Eigen::Vector3d gravity = { 0.0, 0.0, -9.81 };

		/* Methods */
		Stark(const Settings& settings);
		bool run_one_step();
		bool run(double duration, std::function<void()> callback = nullptr);
		std::string get_frame_path(std::string name) const;
		void print();

	private:
		/* Fields */
		NewtonsMethod newton;
		bool is_init = false;
		double next_frame_time = -std::numeric_limits<double>::epsilon();
		double execution_time = 0.0;

		/* Methods */
		void _initialize();
		void _write_frame();
	};
}
