#pragma once
#include <string>

#include <Eigen/Dense>

#include <symx>

#include "Callbacks.h"

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
		int n_threads = -1;
		double newton_tol = 1e-8;
		double cg_tol = 1e-12;
		double gc_max_iterations_multiplier = 1.0;
		Eigen::VectorXd du;
		Eigen::VectorXd u0;
		Eigen::VectorXd u1;
		int max_newton_iterations = 10;
		int max_line_search_iterations = 10;
		double line_search_multiplier = 0.5;
		int it_count = 0;
		bool use_direct_linear_solve = false;
		int debug_output_counter = 0;

		NewtonError solve(symx::GlobalEnergy& global_energy, const Callbacks& callbacks, Output& output);
	};
}