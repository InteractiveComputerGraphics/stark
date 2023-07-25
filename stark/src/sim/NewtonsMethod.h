#pragma once
#include <string>

#include <Eigen/Dense>

#include <symx>

#include "Settings.h"
#include "Callbacks.h"
#include "../utils/Console.h"
#include "../utils/Logger.h"


namespace stark
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
		utils::Logger line_search_debug_logger;

		/* Methods */
		NewtonError solve(symx::GlobalEnergy& global_energy, const Callbacks& callbacks, Settings& settings, utils::Console& console, utils::Logger& logger);
	};
}