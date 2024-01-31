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
		TooManyCGIterations = 3,
		InvalidConfiguration = 4,
		LineSearchDoesntDescend = 5,
		Restart = 6,
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

		// Adaptivity Buffers
		bool force_full_step = false;
		bool last_was_full_step = false;
		std::vector<uint8_t> active_nodes;
		std::vector<int> active_to_global_node_map;
		std::vector<int> global_to_active_node_map;
		std::vector<Eigen::Triplet<double>> triplet_buffer;
		std::vector<Eigen::Triplet<double>> active_hess_triplets;

		// Counters
		int step_newton_it = 1;
		int step_newton_sub_it = 1;
		int step_newton_sub_it_count = 0;
		int step_line_search_count = 1;
		int cg_iterations_in_step = 0;
		int accumulated_newton_it_count = 0;
		int accumulated_newton_sub_it_count = 0;
		
		// Debug
		int debug_output_counter = 0;
		Logger line_search_debug_logger;

		// Pointers
		symx::GlobalEnergy* global_energy = nullptr;
		Callbacks* callbacks = nullptr;
		Settings* settings = nullptr; 
		Console* console = nullptr; 
		Logger* logger = nullptr;

		/* Methods */
		NewtonState solve(symx::GlobalEnergy& global_energy, Callbacks& callbacks, Settings& settings, Console& console, Logger& logger);

	private:
		void _run_before_evaluation();
		void _run_after_evaluation();
		symx::Assembled _evaluate_E_grad_hess();
		symx::Assembled _evaluate_E_grad();
		symx::Assembled _evaluate_E();
		Eigen::VectorXd _compute_residual(const Eigen::VectorXd& grad, double dt);
		double _compute_acceleration_correction(double du, double dt);
		int _get_active_dofs_count();
		double _forcing_sequence(const Eigen::VectorXd& rhs);

		Eigen::VectorXd _solve_linear_system_and_tick_adaptivity(const symx::Assembled& assembled, double dt);
		double _inplace_backtracking_line_search(const Eigen::VectorXd& du, double E0, double step_valid_configuration, double du_dot_grad);
		double _inplace_max_step_in_search_direction(const Eigen::VectorXd& du);

		void _debug_print_initial_residual(const Eigen::VectorXd& residual, double min);
	};
}
