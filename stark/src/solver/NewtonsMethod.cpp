#include "NewtonsMethod.h"

#include "../utils/linear_system.h"


stark::NewtonError stark::NewtonsMethod::solve(symx::GlobalEnergy& global_energy, const Callbacks& callbacks, Settings& settings, utils::Console& console, utils::Logger& logger)
{
	const int ndofs = global_energy.get_total_n_dofs();
	this->du.resize(ndofs);
	this->u0.resize(ndofs);
	this->u1.resize(ndofs);

	int newton_it = 0;
	int total_line_search_it = 0;
	int total_CG_it = 0;
	double residual = std::numeric_limits<double>::max();
	while (residual > settings.newton.newton_tol) {
		
		newton_it++;
		this->it_count++;
		if (newton_it == settings.newton.max_newton_iterations) {
			console.print(fmt::format("\n\t\t -> Max Newton iterations reached ({:d}) with residual {:.2e}\n", settings.newton.max_newton_iterations, residual), Verbosity::TimeSteps);
			return NewtonError::TooManyNewtonIterations;
		}
		console.print(fmt::format("\n\t\t {:d}. ", newton_it), Verbosity::NewtonIterations);

		// Linear system
		//// Evaluate and assemble
		logger.start_timing("before_energy_evaluation");
		callbacks.run_before_energy_evaluation();
		logger.stop_timing_add("before_energy_evaluation");

		logger.start_timing("evaluate_E_grad_hess");
		symx::Assembled assembled = global_energy.evaluate_E_grad_hess(settings.debug.symx_check_for_NaNs);
		logger.stop_timing_add("evaluate_E_grad_hess");

		logger.start_timing("after_energy_evaluation");
		callbacks.run_after_energy_evaluation();
		logger.stop_timing_add("after_energy_evaluation");

		logger.add_to_timer("compiled_E_g_h (acc)", assembled.compiled_runtime);
		console.print(fmt::format("dE0 = {:.2e} | ", assembled.grad->maxCoeff()), Verbosity::NewtonIterations);

		//// Solve
		this->du.resize(ndofs);
		const Eigen::VectorXd rhs = -1.0 * (*assembled.grad);

		if (settings.newton.use_direct_linear_solve) {  // TODO: Clarify that one is directLU and the other is BCG without PSD checks
			logger.start_timing("directLU");
			utils::solve_linear_system_with_directLU(this->du, *assembled.hess, rhs);
			logger.stop_timing_add("directLU");
		}
		else {
			logger.start_timing("CG");

			// Forcing sequence
			const double grad_norm = assembled.grad->norm();
			const double cg_tol = std::min(0.1 /*TODO: arbritrary.*/, grad_norm*std::min(0.5, std::sqrt(grad_norm)));

			this->du.setZero();
			const int max_iterations = std::max(1000, (int)(settings.newton.cg_max_iterations_multiplier * ndofs)); // Very small sims will need to exceed ndofs iterations
			const int iterations = utils::solve_linear_system_with_CG(this->du, *assembled.hess, rhs, max_iterations, cg_tol, settings.execution.n_threads);
			total_CG_it += iterations;
			logger.stop_timing_add("CG");

			if (iterations == max_iterations) {  // TODO: Check
				console.print("\n\t\t -> CG didn't converge.\n", Verbosity::TimeSteps);
				return NewtonError::TooManyCGIterations;
			}
		}
		console.print(fmt::format("du = {:.2e} | ", this->du.norm()), Verbosity::NewtonIterations);

		// Line search (for later use)
		const double base_E = *assembled.E;
		const double precomputed_dot = this->du.dot(*assembled.grad);
		const double suitable_backtracking_energy = base_E + 1e-4 * precomputed_dot;
		console.print(fmt::format("dE0*du = {:.2e} | ", precomputed_dot), Verbosity::NewtonIterations);

		// Sufficient descend
		if (precomputed_dot > 0.0) {
			console.print("\n\t\t -> Line search doesn't descend.\n", Verbosity::TimeSteps);
			return NewtonError::LineSearchDoesntDescend;
		}

		// Max step in the search direction
		double step = callbacks.run_max_allowed_step();

		// Step and valid step
		global_energy.get_dofs(this->u0.data());
		while (true) {

			if (step < 0.01) {
				console.print("\n\t\t -> Valid step too small (less than 0.1%).\n", Verbosity::TimeSteps);
				return NewtonError::InvalidConfiguration;
			}

			this->u1 = this->u0 + step * this->du;
			global_energy.set_dofs(this->u1.data());

			logger.start_timing("is_state_valid");
			const bool is_valid_state = callbacks.run_is_state_valid();
			logger.stop_timing_add("is_state_valid");

			if (is_valid_state) {
				break;
			}

			step *= 0.5;
		}
		console.print(fmt::format("max step = {:.2e} | ", step), Verbosity::NewtonIterations);

		// Convergence?
		logger.start_timing("before_energy_evaluation");
		callbacks.run_before_energy_evaluation();
		logger.stop_timing_add("before_energy_evaluation");

		logger.start_timing("evaluate_E_grad");
		assembled = global_energy.evaluate_E_grad(settings.debug.symx_check_for_NaNs);
		logger.stop_timing_add("evaluate_E_grad");

		logger.start_timing("after_energy_evaluation");
		callbacks.run_after_energy_evaluation();
		logger.stop_timing_add("after_energy_evaluation");

		logger.add_to_timer("compiled_E_g (acc)", assembled.compiled_runtime);
		residual = assembled.grad->maxCoeff();
		console.print(fmt::format("dE1 = {:.2e}", residual), Verbosity::NewtonIterations);

		if (residual < settings.newton.newton_tol) {
			break;
		}

		// Line search: Backtracking
		const double step_valid_configuration = step;
		logger.start_timing("line_search");
		int line_search_it = 0;
		while (*assembled.E > suitable_backtracking_energy) {
			console.print(fmt::format("\n\t\t\t {:d}. step = {:.2e} | (1.0 - E/E_bt) = {:.2e}", line_search_it, step, 1.0 - (*assembled.E)/suitable_backtracking_energy), Verbosity::NewtonIterations);

			// Reduce step
			step *= settings.newton.line_search_multiplier;
			this->u1 = this->u0 + step * this->du;
			global_energy.set_dofs(this->u1.data());

			// Sequence
			logger.start_timing("before_energy_evaluation");
			callbacks.run_before_energy_evaluation();
			logger.stop_timing_add("before_energy_evaluation");

			logger.start_timing("evaluate_E");
			assembled = global_energy.evaluate_E(settings.debug.symx_check_for_NaNs);
			logger.stop_timing_add("evaluate_E");

			logger.start_timing("after_energy_evaluation");
			callbacks.run_after_energy_evaluation();
			logger.stop_timing_add("after_energy_evaluation");

			logger.add_to_timer("compiled_E (acc)", assembled.compiled_runtime);

			// Counters
			line_search_it++;

			if (line_search_it == settings.newton.max_line_search_iterations) {
				console.print(fmt::format("\n\t\t\t\t -> Max line search iterations reached ({:d}).\n", settings.newton.max_line_search_iterations), Verbosity::TimeSteps);
				return NewtonError::TooManyLineSearchIterations;
			}
		}

		// Log line search energy profile
		if (settings.debug.line_search_output) {
			if (step_valid_configuration > 0.99 && step < 0.99) {
				const std::string label = std::to_string(this->debug_output_counter);

				this->line_search_debug_logger.append_to_series(label, fmt::format("{:.6e}", base_E));
				this->line_search_debug_logger.append_to_series(label, fmt::format("{:.6e}", 1.0 - suitable_backtracking_energy / base_E));
				this->line_search_debug_logger.append_to_series(label, fmt::format("{:.6e}", this->du.norm()));
				for (double fstep = -1.0; fstep < 2.0; fstep += 0.01) {
					if (this->debug_output_counter == 0) {
						this->line_search_debug_logger.append_to_series("normalized_step_length", fmt::format("{:.6e}", fstep));
					}

					this->u1 = this->u0 + fstep * this->du;
					global_energy.set_dofs(this->u1.data());
					callbacks.run_before_energy_evaluation();
					assembled = global_energy.evaluate_E();
					callbacks.run_after_energy_evaluation();
					const double v = (1.0 - (*assembled.E) / base_E);
					this->line_search_debug_logger.append_to_series(label, fmt::format("{:.6e}", v));
				}

				this->line_search_debug_logger.save_to_disk();
				this->debug_output_counter++;

				// Restore state
				this->u1 = this->u0 + step * this->du;
				global_energy.set_dofs(this->u1.data());
				console.print(fmt::format("\n\t\t\t\t line_search.txt updated [{}]", label), Verbosity::NewtonIterations);
			}
		}

		total_line_search_it += line_search_it;
		logger.stop_timing_add("line_search");
	}

	console.print("\n\t\t", Verbosity::NewtonIterations);
	console.print("#newton: " + std::to_string(newton_it), Verbosity::TimeSteps);
	console.print(" | #CG/newton: " + std::to_string((int)(total_CG_it/newton_it)), Verbosity::TimeSteps);
	console.print(" | #line_search/newton: " + std::to_string((int)(total_line_search_it/newton_it)), Verbosity::TimeSteps);

	logger.add_to_counter("newton_iterations", newton_it);
	logger.add_to_counter("CG_iterations", total_CG_it);
	logger.add_to_counter("line_search_iterations", total_line_search_it);

	return NewtonError::Successful;
}
