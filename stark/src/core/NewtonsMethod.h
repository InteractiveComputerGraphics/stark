#pragma once
#include <Eigen/Dense>
#include <symx>

#include "Settings.h"
#include "Callbacks.h"
#include "Console.h"
#include "Logger.h"


namespace stark::core
{
	enum class NewtonState
	{
		Successful = 0,
		TooManyNewtonIterations = 1,
		TooManyLineSearchIterations = 2,
		LinearSystemFailure = 3,
		InvalidIntermediateConfiguration = 4,
		LineSearchDoesntDescend = 5,
		InvalidConvergedState = 6,
		Running = 7
	};

	class NewtonsMethod
	{
	public:

		/* Fields */

		// Buffers
		Eigen::VectorXd du;
		Eigen::VectorXd u0;
		Eigen::VectorXd u1;
		Eigen::VectorXd residual;

		// Counters
		int step_newton_it = 1;
		int step_line_search_count = 1;
		int cg_iterations_in_step = 0;

		// Debug
		int debug_output_counter = 0;
		Logger line_search_debug_logger;

		// Pointers for easy access within the methods (TODO: Refactor)
		symx::GlobalEnergy* global_energy = nullptr;
		Callbacks* callbacks = nullptr;
		const Settings* settings = nullptr;
		Console* console = nullptr;
		Logger* logger = nullptr;

		/* Methods */
		NewtonState solve(const double& dt, symx::GlobalEnergy& global_energy, Callbacks& callbacks, const Settings& settings, Console& console, Logger& logger);

	private:
		void _run_before_evaluation();
		void _run_after_evaluation();
		symx::Assembled _evaluate_E_grad_hess();
		symx::Assembled _evaluate_E_grad();
		symx::Assembled _evaluate_E();
		Eigen::VectorXd _compute_residual(const Eigen::VectorXd& grad, double dt);
		double _compute_acceleration_correction(double du, double dt);
		double _forcing_sequence(const Eigen::VectorXd& rhs);

		bool _solve_linear_system(Eigen::VectorXd& du, const symx::Assembled& assembled, double dt);
		double _inplace_max_step_in_search_direction(const Eigen::VectorXd& du);
		double _inplace_backtracking_line_search(const Eigen::VectorXd& du, double E0, double E, double step_valid_configuration, double du_dot_grad);
	};
}
