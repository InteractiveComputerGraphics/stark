#include "NewtonsMethod.h"

#include "../utils/linear_system.h"


stark::NewtonError stark::NewtonsMethod::solve(symx::GlobalEnergy& global_energy, const Callbacks& callbacks, utils::Console& console, utils::Logger& logger)
{
	const int ndofs = global_energy.get_total_n_dofs();
	this->du.resize(ndofs);
	this->u0.resize(ndofs);
	this->u1.resize(ndofs);

	int newton_it = 0;
	int total_line_search_it = 0;
	int total_CG_it = 0;
	double residual = std::numeric_limits<double>::max();
	while (residual > this->newton_tol) {
		console.print(fmt::format("\t\t {:d}. ", newton_it), Verbosity::NewtonIterations);
		
		newton_it++;
		this->it_count++;
		if (newton_it == this->max_newton_iterations) {
			console.print(fmt::format("\t| Max Newton iterations reached ({:d}) with residual {:.2e}\n", this->max_newton_iterations, residual), Verbosity::TimeSteps);
			return NewtonError::TooManyNewtonIterations;
		}

		// Linear system
		//// Evaluate and assemble
		logger.start_timing("assemble_linear_system");
		symx::Assembled assembled = global_energy.evaluate_E_grad_hess();
		logger.stop_timing_add("assemble_linear_system");
		logger.add_to_timer("compiled_E_g_h (acc)", assembled.compiled_runtime);
		console.print(fmt::format("dE = {:.2e}  |", assembled.grad.norm()), Verbosity::NewtonIterations);

		//// Solve
		this->du.resize(ndofs);
		const Eigen::VectorXd rhs = -1.0 * assembled.grad;

		if (this->use_direct_linear_solve) {  // TODO: Clarify that one is directLU and the other is BCG without PSD checks
			logger.start_timing("directLU");
			utils::solve_linear_system_with_directLU(this->du, *assembled.hess, rhs);
			logger.stop_timing_add("directLU");
		}
		else {
			logger.start_timing("CG");
			this->du.setZero();
			const int max_iterations = std::max(100, (int)(this->gc_max_iterations_multiplier * ndofs)); // Very small sims will need to exceed ndofs iterations
			const int iterations = utils::solve_linear_system_with_CG(this->du, *assembled.hess, rhs, max_iterations, this->cg_tol, this->n_threads);
			total_CG_it += iterations;
			logger.stop_timing_add("CG");

			if (iterations == max_iterations) {  // TODO: Check
				console.print("\t| CG didn't converged.", Verbosity::TimeSteps);
				return NewtonError::TooManyCGIterations;
			}
		}
		console.print(fmt::format("du = {:.2e}  |", this->du.norm()), Verbosity::NewtonIterations);

		// Line search (for later use)
		const double base_E = assembled.E;
		const double precomputed_dot = this->du.dot(assembled.grad);
		const double suitable_backtracking_energy = assembled.E + 1e-4 * precomputed_dot;
		console.print(fmt::format("dE*du = {:.2e}  |", precomputed_dot), Verbosity::NewtonIterations);

		// Sufficient descend
		if (precomputed_dot > 0.0) {
			console.print("\t| Line search doesn't descend.", Verbosity::TimeSteps);
			return NewtonError::LineSearchDoesntDescend;
		}

		// Max step in the search direction
		double step = callbacks.run_max_allowed_step();

		// Step and valid step
		global_energy.get_dofs(this->u0.data());
		while (true) {

			if (step < 0.01) {
				console.print("\t| Valid step too small (less than 1%).", Verbosity::TimeSteps);
				return NewtonError::InvalidConfiguration;
			}

			this->u1 = this->u0 + step * this->du;
			global_energy.set_dofs(this->u1.data());

			if (callbacks.run_is_state_valid()) {
				break;
			}

			step *= 0.5;
		}
		console.print(fmt::format("max step = {:.2e}  |", step), Verbosity::NewtonIterations);

		// Convergence?
		logger.start_timing("assemble_gradient");
		assembled = global_energy.evaluate_E_grad();
		logger.stop_timing_add("assemble_gradient");
		logger.add_to_timer("compiled_E_g (acc)", assembled.compiled_runtime);
		residual = assembled.grad.norm();
		console.print(fmt::format("dE1 = {:.2e}", residual), Verbosity::NewtonIterations);

		if (residual < this->newton_tol) {
			break;
		}

		// Line search: Backtracking
		const double step_valid_configuration = step;
		logger.start_timing("line_search");
		int line_search_it = 0;
		while (assembled.E > suitable_backtracking_energy) {
			console.print(fmt::format("\n\t\t\t {:d}. E/E_bt = {:.2e}", line_search_it, assembled.E/suitable_backtracking_energy), Verbosity::NewtonIterations);

			// Reduce step
			step *= this->line_search_multiplier;
			this->u1 = this->u0 + step * this->du;
			global_energy.set_dofs(this->u1.data());

			// Sequence
			assembled = global_energy.evaluate_E();
			logger.add_to_timer("compiled_E (acc)", assembled.compiled_runtime);

			// Counters
			line_search_it++;

			if (line_search_it == this->max_line_search_iterations) {
				console.print(fmt::format("\t| Max line search iterations reached ({d})", this->max_line_search_iterations), Verbosity::TimeSteps);
				return NewtonError::TooManyLineSearchIterations;
			}
		}
		console.print("\n", Verbosity::NewtonIterations);

		// Log line search energy profile
		if (this->debug_line_search_print) {
			if (step_valid_configuration > 0.99 && step < 0.99) {
				const std::string label = std::to_string(this->debug_output_counter);

				this->line_search_debug_logger.add(label, fmt::format("{:.6e}", base_E));
				this->line_search_debug_logger.add(label, fmt::format("{:.6e}", 1.0 - suitable_backtracking_energy / base_E));
				for (double fstep = -1.0; fstep < 2.0; fstep += 0.01) {
					if (this->debug_output_counter == 0) {
						this->line_search_debug_logger.add("normalized_step_length", fmt::format("{:.6e}", fstep));
					}

					this->u1 = this->u0 + fstep * this->du;
					global_energy.set_dofs(this->u1.data());
					assembled = global_energy.evaluate_E();
					const double v = (1.0 - assembled.E / base_E);
					this->line_search_debug_logger.add(label, fmt::format("{:.6e}", v));
				}

				this->line_search_debug_logger.save_to_disk();

				// Restore state
				this->u1 = this->u0 + step * this->du;
				global_energy.set_dofs(this->u1.data());
			}
		}

		total_line_search_it += line_search_it;
		logger.stop_timing_add("line_search");
	}

	console.print("  |  #newton: " + std::to_string(newton_it), Verbosity::TimeSteps);
	console.print("  |  #CG/newton: " + std::to_string((int)(total_CG_it/newton_it)), Verbosity::TimeSteps);
	console.print("  |  #line_search/newton: " + std::to_string((int)(total_line_search_it/newton_it)), Verbosity::TimeSteps);
	return NewtonError::Successful;
}
