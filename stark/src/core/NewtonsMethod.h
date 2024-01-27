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
		Restart = 6
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
		
		// Adaptive dofs
		bool run_adaptive_dofs = true;
		double dof_deactivation_tolerance_multiplier = 1.0;
		double dofs_percentage_for_full_solve = 0.25;
		int n_rings = 1;
		int n_full_solve_iterations = 0;
		int adaptive_steps_limit = 999999999;
		bool write_du = false;
		int max_substeps = 15;

		int activation_dof_it_count = 0;
		std::vector<uint8_t> active_nodes;
		std::vector<int> active_to_global_node_map;
		std::vector<int> global_to_active_node_map;
		std::vector<Eigen::Triplet<double>> triplet_buffer;
		std::vector<Eigen::Triplet<double>> active_hess_triplets;

		// Debug
		int debug_output_counter = 0;
		Logger line_search_debug_logger;

		/* Methods */
		NewtonError solve(symx::GlobalEnergy& global_energy, const Callbacks& callbacks, Settings& settings, Console& console, Logger& logger);
	};
}