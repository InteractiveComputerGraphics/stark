#include "NewtonsMethod.h"

#include <fstream>

stark::NewtonError stark::NewtonsMethod::solve(symx::GlobalEnergy& global_energy, const Callbacks& callbacks, Output& output)
{
	const int ndofs = global_energy.get_n_dofs();
	this->du.resize(ndofs);
	this->u0.resize(ndofs);
	this->u1.resize(ndofs);

	int newton_it = 0;
	int total_line_search_it = 0;
	int total_CG_it = 0;
	double residual = std::numeric_limits<double>::max();
	while (residual > this->newton_tol) {
		newton_it++;
		this->it_count++;
		if (newton_it == this->max_newton_iterations) {
			Output::output->cout("\t| Max Newton iterations reached (" + std::to_string(this->max_newton_iterations) + ") with residual " + std::to_string(residual), 1);
			return NewtonError::TooManyNewtonIterations;
		}

		// Linear system
		//// Evaluate and assemble
		Output::logger->start_timing("assemble_linear_system");
		stark::Assembled assembled = global_energy.evaluate_E_grad_hess();
		Output::logger->stop_timing_add("assemble_linear_system");
		Output::logger->add_to_timer("compiled_E_g_h (acc)", assembled.compiled_runtime);

		//// Solve
		Output::logger->start_timing("CG");
		{
			auto& sparse = *assembled.hess;
			sparse.set_preconditioner(bsm::Preconditioner::BlockDiagonal);
			sparse.prepare_preconditioning(this->n_threads);
			
			const int max_iterations = std::max(100, (int)(1.0 * ndofs)); // Very small sims will need to exceed ndofs iterations
			this->du.setZero();
			const Eigen::VectorXd rhs = -1.0 * (*assembled.grad);

			if (this->use_direct_linear_solve) {
				std::vector<Eigen::Triplet<double>> triplets;
				assembled.hess->to_triplets(triplets);

				Eigen::SparseMatrix<double> s;
				stb::solver::make_sparse_matrix(s, triplets, ndofs);
				stb::solver::solve_directLU(this->du, s, rhs);
			}
			else {
				cg::Info info = cg::solve<double>(this->du.data(), rhs.data(), ndofs, this->cg_tol, max_iterations,
					[&](double* b, const double* x, const int size) { sparse.spmxv_from_ptr(b, x, this->n_threads);  },
					[&](double* z, const double* r, const int size) { sparse.apply_preconditioning(z, r, this->n_threads); }
				, this->n_threads);
				total_CG_it += info.n_iterations;

				Output::logger->stop_timing_add("CG");
				if (!info.converged) {
					Output::output->cout("\t| CG didn't converged.", 1);
					return NewtonError::TooManyCGIterations;
				}
			}
		}

		// Line search (for later use)
		const double base_E = assembled.E;
		const double precomputed_dot = this->du.dot(*assembled.grad);
		const double suitable_backtracking_energy = assembled.E + 1e-4 * precomputed_dot;

		// Sufficient descend
		if (precomputed_dot > 0.0) {
			Output::output->cout("\t| Line search doesn't descend.", 1);
			return NewtonError::LineSearchDoesntDescend;
		}

		// Max step in the search direction
		double step = global_energy.max_allowed_step_in_search_direction(this->du.data());

		// Step and valid step
		global_energy.get_dofs(this->u0.data());
		while (true) {

			if (step < 0.01) {
				Output::output->cout("\t| Valid step too small (less than 1%).", 1);
				return NewtonError::InvalidConfiguration;
			}

			this->u1 = this->u0 + step * this->du;
			global_energy.set_dofs(this->u1.data());

			if (global_energy.is_valid_configuration()) {
				break;
			}

			step *= 0.5;
		}

		// Convergence?
		Output::logger->start_timing("assemble_gradient");
		assembled = global_energy.evaluate_E_grad();
		Output::logger->stop_timing_add("assemble_gradient");
		Output::logger->add_to_timer("compiled_E_g (acc)", assembled.compiled_runtime);
		residual = assembled.grad->norm();

		if (residual < this->newton_tol) {
			break;
		}

		// Line search: Backtracking
		const double step_valid_configuration = step;
		Output::logger->start_timing("line_search");
		int line_search_it = 0;
		while (assembled.E > suitable_backtracking_energy) {

			// Reduce step
			step *= this->line_search_multiplier;
			this->u1 = this->u0 + step * this->du;
			global_energy.set_dofs(this->u1.data());

			// Sequence
			assembled = global_energy.evaluate_E();
			Output::logger->add_to_timer("compiled_E (acc)", assembled.compiled_runtime);

			// Counters
			line_search_it++;

			if (line_search_it == this->max_line_search_iterations) {
				Output::output->cout("\t| Max line search iterations reached (" + std::to_string(this->max_line_search_iterations) + ")", 1);
				return NewtonError::TooManyLineSearchIterations;
			}
		}

		// DEBUG
		if (false) {
			if (step_valid_configuration > 0.99 && step < 0.99) {
				std::cout << "\nLine search (" << step << ")[" + std::to_string(this->debug_output_counter) + ".bin]";
				std::cout << "(" << residual << " | " << this->du.maxCoeff() << " | " << step_valid_configuration << " | " << step << " | " << precomputed_dot << " | " << assembled.E << ")" << std::endl;

				std::vector<double> values;
				values.push_back(base_E);
				values.push_back(1.0 - suitable_backtracking_energy / base_E);
				for (double fstep = -1.0; fstep < 2.0; fstep += 0.01) {
					this->u1 = this->u0 + fstep * this->du;
					global_energy.set_dofs(this->u1.data());
					assembled = global_energy.evaluate_E();
					const double v = (1.0 - assembled.E / base_E);
					values.push_back(v);
				}

				// Write to file
				std::ofstream fs(std::to_string(this->debug_output_counter) + ".bin", std::ios::out | std::ios::binary);
				fs.write(reinterpret_cast<char*>(values.data()), values.size() * sizeof(double));
				this->debug_output_counter++;

				// Restore state
				this->u1 = this->u0 + step * this->du;
				global_energy.set_dofs(this->u1.data());
			}
		}

		total_line_search_it += line_search_it;
		Output::logger->stop_timing_add("line_search");
	}

	Output::output->cout("  |  #newton: " + std::to_string(newton_it), 1);
	Output::output->cout("  |  #CG/newton: " + std::to_string((int)(total_CG_it/newton_it)), 1);
	Output::output->cout("  |  #line_search/newton: " + std::to_string((int)(total_line_search_it/newton_it)), 1);
	return NewtonError::Successful;
}
