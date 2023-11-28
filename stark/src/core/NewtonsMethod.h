#pragma once
#include <Eigen/Dense>
#include <symx>

#include "Settings.h"
#include "Callbacks.h"
#include "Console.h"
#include "Logger.h"


namespace stark::core
{
	enum class NewtonError
	{
		Successful = 0,
		TooManyNewtonIterations = 1,
		TooManyLineSearchIterations = 2,
		TooManyCGIterations = 3,
		InvalidConfiguration = 4,
		LineSearchDoesntDescend = 5,
	};

	class NewtonsMethod
	{
	public:
		/* Fields */
		// Buffers
		Eigen::VectorXd du;
		Eigen::VectorXd u0;
		Eigen::VectorXd u1;

		// Misc
		int it_count = 0;

		// Debug
		int debug_output_counter = 0;
		Logger line_search_debug_logger;

		/* Methods */
		NewtonError solve(symx::GlobalEnergy& global_energy, const Callbacks& callbacks, Settings& settings, Console& console, Logger& logger);
	};
}