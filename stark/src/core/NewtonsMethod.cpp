#include "NewtonsMethod.h"

#include <symx>
#include <fmt/format.h>

#include "linear_system.h"


stark::core::NewtonError stark::core::NewtonsMethod::solve(symx::GlobalEnergy& global_energy, const Callbacks& callbacks, Settings& settings, Console& console, Logger& logger)
{
	const int ndofs = global_energy.get_total_n_dofs();
	const int n_nodes = ndofs / 3;
	this->du.resize(ndofs);
	this->u0.resize(ndofs);
	this->u1.resize(ndofs);

	std::vector<std::vector<int>> dofs_activation;

	int newton_it = 0;
	int total_line_search_it = 0;
	int total_CG_it = 0;
	double residual = std::numeric_limits<double>::max();
	while (residual > settings.newton.newton_tol) {

		dofs_activation.push_back({});
		std::vector<int>& active_nodes = dofs_activation.back();
		active_nodes.resize(n_nodes, 0);
		
		newton_it++;
		this->it_count++;
		if (newton_it == settings.newton.max_newton_iterations) {
			console.print(fmt::format("\n\t\t -> Max Newton iterations reached ({:d}) with residual {:.2e}\n", settings.newton.max_newton_iterations, residual), ConsoleVerbosity::TimeSteps);
			return NewtonError::TooManyNewtonIterations;
		}
		console.print(fmt::format("\n\t\t {:d}. ", newton_it), ConsoleVerbosity::NewtonIterations);

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
		console.print(fmt::format("dE0 = {:.2e} | ", assembled.grad->maxCoeff()), ConsoleVerbosity::NewtonIterations);

		//// Converged right away? Can happen in the first time step. Avoids unnecessary CG iterations.
		const double grad_norm = assembled.grad->norm();
		if (grad_norm < 1e-10) {
			break;
		}

		// Adaptive DoFs
		//// Boolean activation flags
		for (int i = 0; i < ndofs; i++) {
			if (std::abs((*assembled.grad)[i]) > this->dof_deactivation_tolerance_multiplier *settings.newton.newton_tol) {
				active_nodes[i / 3] = 1;
			}
		}

		//// Mapping
		this->active_to_global_node_map.clear();
		this->global_to_active_node_map = std::vector<int>(n_nodes, -1);
		for (int i = 0; i < n_nodes; i++) {
			if (active_nodes[i] == 1) {
				this->global_to_active_node_map[i] = (int)this->active_to_global_node_map.size();
				this->active_to_global_node_map.push_back(i);
			}
		}
		const int n_active_nodes = (int)this->active_to_global_node_map.size();
		const int n_active_dofs = 3 * n_active_nodes;

		//// Solve
		this->du.resize(ndofs);
		Eigen::VectorXd rhs = -1.0 * (*assembled.grad);
		if (settings.newton.use_direct_linear_solve) {  // TODO: Clarify that one is directLU and the other is BCG without PSD checks
			logger.start_timing("directLU");
			solve_linear_system_with_directLU(this->du, *assembled.hess, rhs);
			logger.stop_timing_add("directLU");
		}
		else {
			logger.start_timing("CG");

			// Forcing sequence
			const double cg_tol = std::min(0.1 /*TODO: arbritrary.*/, grad_norm * std::min(0.5, std::sqrt(grad_norm)));

			if (this->run_adaptive_dofs && (double)n_active_dofs < (double)ndofs*this->dofs_percentage_for_full_solve) {
				logger.append_to_series("active_dofs", n_active_dofs);

				// Build reduced system
				//// Hessian
				this->active_hess_triplets.clear();
				this->triplet_buffer.clear();
				assembled.hess->to_triplets(this->triplet_buffer);
				for (const auto& triplet : this->triplet_buffer) {
					if (active_nodes[triplet.row() / 3] == 1 && active_nodes[triplet.col() / 3] == 1) {
						const int row = this->global_to_active_node_map[triplet.row() / 3] * 3 + triplet.row() % 3;
						const int col = this->global_to_active_node_map[triplet.col() / 3] * 3 + triplet.col() % 3;
						this->active_hess_triplets.push_back(Eigen::Triplet<double>(row, col, triplet.value()));
					}
				}
				Eigen::SparseMatrix<double, Eigen::RowMajor> s;
				s.resize(n_active_dofs, n_active_dofs);
				s.setFromTriplets(this->active_hess_triplets.begin(), this->active_hess_triplets.end());
				s.makeCompressed();

				//// rhs
				Eigen::VectorXd active_rhs = Eigen::VectorXd::Zero(n_active_dofs);
				for (int i = 0; i < n_active_nodes; i++) {
					for (int j = 0; j < 3; j++) {
						active_rhs[3*i + j] = rhs[3*this->active_to_global_node_map[i] + j];
					}
				}
				
				// Solve
				Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::RowMajor> cg;
				cg.compute(s);
				cg.setTolerance(cg_tol);
				Eigen::VectorXd active_du = cg.solve(active_rhs);

				// Map back
				for (int i = 0; i < n_active_nodes; i++) {
					for (int j = 0; j < 3; j++) {
						this->du[3*this->active_to_global_node_map[i] + j] = active_du[3*i + j];
					}
				}
				for (int i = 0; i < n_nodes; i++) {
					if (active_nodes[i] == 0) {
						for (int j = 0; j < 3; j++) {
							this->du[3*i + j] = 0.0;
							rhs[3*i + j] = 0.0;
							(*assembled.grad)[3*i + j] = 0.0;
						}
					}
				}
			}
			else {
				logger.append_to_series("active_dofs", ndofs);

				this->du.setZero();
				const int max_iterations = std::max(1000, (int)(settings.newton.cg_max_iterations_multiplier * ndofs)); // Very small sims will need to exceed ndofs iterations
				const int iterations = solve_linear_system_with_CG(this->du, *assembled.hess, rhs, max_iterations, cg_tol, settings.execution.n_threads);
				total_CG_it += iterations;
				if (iterations == max_iterations) {  // TODO: Check
					console.print("\n\t\t -> CG didn't converge.\n", ConsoleVerbosity::TimeSteps);
					return NewtonError::TooManyCGIterations;
				}
			}

			logger.stop_timing_add("CG");
		}
		console.print(fmt::format("ndofs = {:d}/{:d} | ", n_active_dofs, ndofs), ConsoleVerbosity::NewtonIterations);
		console.print(fmt::format("du = {:.2e} | ", this->du.norm()), ConsoleVerbosity::NewtonIterations);

		// Line search (for later use)
		const double base_E = *assembled.E;
		const double precomputed_dot = this->du.dot(*assembled.grad);
		const double suitable_backtracking_energy = base_E + 1e-4 * precomputed_dot;
		console.print(fmt::format("dE0*du = {:.2e} | ", precomputed_dot), ConsoleVerbosity::NewtonIterations);

		// Sufficient descend
		if (precomputed_dot > 0.0) {
			console.print("\n\t\t -> Line search doesn't descend.\n", ConsoleVerbosity::TimeSteps);
			return NewtonError::LineSearchDoesntDescend;
		}

		// Max step in the search direction
		double step = callbacks.run_max_allowed_step();

		// Step and valid step
		global_energy.get_dofs(this->u0.data());
		while (true) {

			if (step < 0.01) {
				console.print("\n\t\t -> Valid step too small (less than 0.1%).\n", ConsoleVerbosity::TimeSteps);
				return NewtonError::InvalidConfiguration;
			}

			this->u1 = this->u0 + step * this->du;
			global_energy.set_dofs(this->u1.data());

			logger.start_timing("is_intermidiate_state_valid");
			const bool is_valid_state = callbacks.run_is_intermidiate_state_valid();
			logger.stop_timing_add("is_intermidiate_state_valid");

			if (is_valid_state) {
				break;
			}

			step *= 0.5;
		}
		console.print(fmt::format("max step = {:.2e} | ", step), ConsoleVerbosity::NewtonIterations);

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
		console.print(fmt::format("dE1 = {:.2e}", residual), ConsoleVerbosity::NewtonIterations);

		if (residual < settings.newton.newton_tol) {
			break;
		}

		// Line search: Backtracking
		const double step_valid_configuration = step;
		logger.start_timing("line_search");
		int line_search_it = 0;
		while (*assembled.E > suitable_backtracking_energy) {
			console.print(fmt::format("\n\t\t\t {:d}. step = {:.2e} | (1.0 - E/E_bt) = {:.2e}", line_search_it, step, 1.0 - (*assembled.E)/suitable_backtracking_energy), ConsoleVerbosity::NewtonIterations);

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
				console.print(fmt::format("\n\t\t\t\t -> Max line search iterations reached ({:d}).\n", settings.newton.max_line_search_iterations), ConsoleVerbosity::TimeSteps);
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
				console.print(fmt::format("\n\t\t\t\t line_search.txt updated [{}]", label), ConsoleVerbosity::NewtonIterations);
			}
		}

		total_line_search_it += line_search_it;
		logger.stop_timing_add("line_search");
	}

	// Write dofs activation to file
	std::ofstream file;
	file.open(settings.output.output_directory + "/dofs_" + std::to_string(this->activation_dof_it_count) + ".txt", std::ios::out | std::ios::app);
	for (auto& current_dofs_active : dofs_activation) {
		for (int i : current_dofs_active) {
			file << i << ", ";
		}
		file << "\n";
	}
	file.close();
	this->activation_dof_it_count++;

	if (!callbacks.run_is_converged_state_valid()) {
		console.print("\n\t\t -> Converged state is not valid.\n", ConsoleVerbosity::TimeSteps);
		return NewtonError::Restart;
	}

	console.print("\n\t\t", ConsoleVerbosity::NewtonIterations);
	console.print("#newton: " + std::to_string(newton_it), ConsoleVerbosity::TimeSteps);
	console.print(" | #CG/newton: " + std::to_string((int)(total_CG_it/newton_it)), ConsoleVerbosity::TimeSteps);
	console.print(" | #line_search/newton: " + std::to_string((int)(total_line_search_it/newton_it)), ConsoleVerbosity::TimeSteps);

	logger.add_to_counter("newton_iterations", newton_it);
	logger.add_to_counter("CG_iterations", total_CG_it);
	logger.add_to_counter("line_search_iterations", total_line_search_it);

	return NewtonError::Successful;
}
