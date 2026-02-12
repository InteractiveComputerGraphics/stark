#include "NewtonsMethod.h"
#include "gnuplot.h"
#include <fmt/format.h>
#include <BlockedSparseMatrix/solve_pcg.h>
#include <Eigen/SparseLU>
#include <limits>
#include <omp.h>
#include <algorithm>

using namespace symx;

symx::NewtonsMethod::NewtonsMethod(spGlobalPotential global_potential, spContext context, spSolverCallbacks callbacks)
    : global_potential(global_potential), context(context), callbacks(callbacks)
{
    this->output = context->output;
    this->logger = context->logger;
    if (callbacks == nullptr) {
        this->callbacks = std::make_shared<SolverCallbacks>(context);
    }
    this->compiled = std::make_shared<SecondOrderCompiledGlobal>(global_potential, context);
}

spNewtonsMethod symx::NewtonsMethod::create(spGlobalPotential global_potential, spContext context, spSolverCallbacks callbacks)
{
    return std::make_shared<NewtonsMethod>(global_potential, context, callbacks);
}

SolverReturn NewtonsMethod::solve()
{
    const int ndofs = global_potential->get_total_n_dofs();
    const int n_dof_blocks = ndofs / 3;

    // Check
    if (!global_potential) {
        std::cout << "symx error NewtonsMethod::solve(): GlobalPotential not set." << std::endl;
        exit(1);
    }

    if (ndofs <= 0) {
        std::cout << "symx error NewtonsMethod::solve(): No degrees of freedom." << std::endl;
        exit(1);
    }

    // Set up
    this->stats = SolveStats(); // Reset stats
    this->du.resize(ndofs);
    this->dofs_ls.resize(ndofs);
    this->rhs.resize(ndofs);
    this->grad.resize(ndofs);
    double E0 = 0.0;
    double du_dot_grad = 0.0;
    spElementHessians element_hessians = nullptr;
    ElementHessians::spBSM hess = nullptr;
    SolverReturn result = SolverReturn::Running;

    // Projection state (persists across Newton iterations for adaptive modes)
    this->pdn_countdown = 0;     // ProjectOnDemand: iterations remaining with full projection
    this->ppn_threshold = -1.0;  // Progressive: gradient threshold (-1 = not yet determined)
    this->ppn_active_dof_blocks_for_projection.assign(n_dof_blocks, static_cast<uint8_t>(false));

    // Prepare for line search plot if needed
    if (this->settings.print_line_search_upon_failure && !this->_ls_logging_mode) {
        this->initial_dofs.resize(ndofs);
        this->global_potential->get_dofs(this->initial_dofs.data());
    }

    // Check initial state validity
    bool initial_valid = this->callbacks->run_is_initial_state_valid();
    if (!initial_valid) {
        result = SolverReturn::InvalidIntermediateConfiguration;
    }

    // ====== MAIN NEWTON LOOP ======
    int newton_iteration = -1;
    while (result == SolverReturn::Running) {
        newton_iteration++;

        // Check maximum iterations
        if (newton_iteration > this->settings.max_iterations) {
            result = SolverReturn::TooManyNewtonIterations;
            break;
        }

        // Print iteration header
        this->output->print_with_new_line(fmt::format("{:2d}. ", newton_iteration), Verbosity::Medium);

        // Evaluate energy, gradient, and Hessian
        this->callbacks->run_before_energy_evaluation();
        element_hessians = this->compiled->evaluate_P__dP_du__local_d2P_du2(E0, this->grad);
        this->callbacks->run_after_energy_evaluation();

        // Compute residual
        double residual_norm = this->callbacks->compute_residual(this->grad);
        this->output->print(fmt::format("r0: {:.2e} | ", residual_norm), Verbosity::Medium);

        // Residual too small for numerical stability?
        if (residual_norm < this->settings.epsilon_residual) {
            result = SolverReturn::Successful;
            break;
        }

        // Residual convergence?
        if (newton_iteration >= this->settings.min_iterations && residual_norm < this->settings.residual_tolerance) {
            result = SolverReturn::Successful;
            break;
        }

        // Inner loop: project + solve until we get a descent direction (or give up)
        hess = nullptr;  // Reset for fresh assembly this iteration
        const int initial_cp_iterations = this->stats.cg_iterations;
        bool linear_system_solve_success = false;
        while (!linear_system_solve_success) {

            // Inner indentation
            this->output->print_new_line(Verbosity::Full);

            // Project element Hessians and assemble global Hessian
            bool all_projected = this->_project_and_assemble(element_hessians, hess, ndofs);

            // Solve linear system
            linear_system_solve_success = this->_solve_linear_system(this->du, hess, this->grad);

            // Handle linear system failure
            if (!linear_system_solve_success) {
                bool can_project_more = (this->settings.projection_mode != ProjectionToPD::Newton) && !all_projected;
                if (!can_project_more) {
                    this->output->print("Linear system failed. ", Verbosity::Full);
                    result = SolverReturn::LinearSystemFailure;
                    break;
                }
            }

            // Check if search direction descends
            bool descends = false;
            if (linear_system_solve_success) {
                du_dot_grad = this->du.dot(this->grad);
                descends = du_dot_grad < 0.0;
                if (!descends) {
                    bool can_project_more = (this->settings.projection_mode != ProjectionToPD::Newton) && !all_projected;
                    if (!can_project_more) {
                        this->output->print("Does not descend. ", Verbosity::Full);
                        result = SolverReturn::LineSearchDoesNotDescend;
                        break;
                    }
                }
            }

            // Success: linear system solved and direction descends
            if (linear_system_solve_success && descends) {
                break;
            }
            
            // Failed: request more projection and retry
            this->_increase_projection();
            this->output->print("Retrying with more projection.", Verbosity::Full);
            
        } // End of linear system solve loop
        
        if (result != SolverReturn::Running) {
            this->output->print_with_new_line("Newton failure: Could not solve the linear system or find a descend direction.", Verbosity::Summary);
            break;
        }
        
        // Solve successful
        if (this->output->get_verbosity() != Verbosity::Full) {
            this->output->print(fmt::format("ph: {:4.1f}% | #CG: {:4d} | ", 
                element_hessians->projection_ratio() * 100.0, this->stats.cg_iterations - initial_cp_iterations), Verbosity::Medium);
        }

        //// Increase projection
        this->_decrease_projection();

        //// Log
        const double n_hessians = (double)element_hessians->size();
        const double n_projected_hessians = (double)element_hessians->n_projected();
        const double projected_hessians_ratio = n_projected_hessians/n_hessians;
        logger->add_and_append("n_hessians", n_hessians);
        logger->add_and_append("n_projected_hessians", n_projected_hessians);
        logger->add_and_append("projected_hessians_ratio", projected_hessians_ratio);
        this->stats.n_hessians += n_hessians;
        this->stats.n_projected_hessians += n_projected_hessians;
        logger->add_and_append("cg_iterations", this->last_cg_iterations);

        // Convergence: Step tolerance
        double du_max = this->du.cwiseAbs().maxCoeff();
        this->output->print(fmt::format("du: {:.1e} | ", du_max), Verbosity::Medium);
        if (newton_iteration >= this->settings.min_iterations && du_max < this->settings.step_tolerance) {
            result = SolverReturn::Successful;
            break;
        }

        // Line search
        result = this->_line_search_inplace(E0, du_dot_grad, du_max);
        if (result == SolverReturn::TooManyLineSearchIterations) {
            this->output->print_with_new_line("Newton failure: Too many line search iterations.", Verbosity::Summary);
        }

        // Exit Newton's loop
        if (result != SolverReturn::Running) {
            break;
        }
    }

    // Check if converged state is valid
    if (result == SolverReturn::Successful) {
        bool converged_valid = true;
        converged_valid = this->callbacks->run_is_converged_state_valid();
        if (!converged_valid) {
            result = SolverReturn::InvalidConvergedState;
        }
    }

    // Log
    this->stats.newton_iterations = newton_iteration + 1;
    if (this->stats.n_hessians > 0) {
        this->stats.projected_hessians_ratio = (double)this->stats.n_projected_hessians / (double)this->stats.n_hessians;
    }

    // Solve stats accumulation
    logger->add_and_append("newton_iterations", stats.newton_iterations);

    return result;
}

bool NewtonsMethod::_project_and_assemble(spElementHessians element_hessians, ElementHessians::spBSM& hess, int ndofs)
{
    // Handles both initial assembly and incremental updates (for PPN).
    // PPN workflow: assemble unprojected -> project some -> update_global with diff
    
    constexpr uint8_t TRUE_U8 = static_cast<uint8_t>(true);
    constexpr uint8_t FALSE_U8 = static_cast<uint8_t>(false);

    const int n_dof_blocks = ndofs / 3;
    bool all_projected = false;
    double projection_ratio = 0.0;

    // PPN requires global Hessian assembled first (uses incremental updates)
    if (this->settings.projection_mode == ProjectionToPD::Progressive && hess == nullptr) {
        auto timer_assembly = this->logger->time("assembly");
        hess = element_hessians->assemble_global(this->context->n_threads, ndofs);
    }

    // Project element Hessians to PD
    {
        auto timer_eval = this->logger->time("project_to_PD");

        switch (this->settings.projection_mode) {
            case ProjectionToPD::Newton:
                // Pure Newton: no projection
                break;

            case ProjectionToPD::ProjectedNewton:
                // Project all element Hessians in-place before assembly
                element_hessians->project_to_PD_inplace__all(
                    this->settings.projection_eps,
                    this->settings.project_to_pd_use_mirroring);
                all_projected = true;
                projection_ratio = 1.0;
                break;

            case ProjectionToPD::ProjectOnDemand:
                if (this->pdn_countdown > 0) {
                    if (hess == nullptr) {
                        element_hessians->project_to_PD_inplace__all(
                            this->settings.projection_eps,
                            this->settings.project_to_pd_use_mirroring);
                    } else {
                        element_hessians->project_to_PD_for_update__all(
                            this->settings.projection_eps,
                            this->settings.project_to_pd_use_mirroring);
                    }
                    all_projected = true;
                    projection_ratio = 1.0;
                }
                break;

            case ProjectionToPD::Progressive:
                if (this->ppn_threshold > 0.0) {

                    // Default to PN
                    if (this->ppn_threshold > 0.0 && this->ppn_threshold < 1e-12) {
                        this->ppn_threshold = 0.0;
                    }

                    // Select blocks based on gradient magnitude
                    all_projected = true;
                    for (int block_row = 0; block_row < n_dof_blocks; block_row++) {
                        if (this->grad.segment<3>(3 * block_row).cwiseAbs().maxCoeff() > this->ppn_threshold) {
                            this->ppn_active_dof_blocks_for_projection[block_row] = TRUE_U8;
                        } else {
                            this->ppn_active_dof_blocks_for_projection[block_row] = FALSE_U8;
                            all_projected = false;
                        }
                    }
                    element_hessians->project_to_PD_for_update__selectively(
                        this->settings.projection_eps,
                        this->settings.project_to_pd_use_mirroring,
                        this->ppn_active_dof_blocks_for_projection);
                    projection_ratio = element_hessians->projection_ratio();
                }
                break;

            default:
                std::cout << "Error: Unknown projection mode." << std::endl;
                exit(1);
        }

        this->output->print(fmt::format("ph: {:4.1f}% | ", 100.0 * projection_ratio), Verbosity::Full);
    }

    // Assemble or update global Hessian
    {
        auto timer_assembly = this->logger->time("assembly");
        if (hess == nullptr) {
            hess = element_hessians->assemble_global(this->context->n_threads, ndofs);
        } 
        else {
            hess = element_hessians->update_global(this->context->n_threads);
        }
    }

    return all_projected;
}

void NewtonsMethod::_increase_projection()
{
    switch (this->settings.projection_mode) {
        case ProjectionToPD::ProjectOnDemand:
            this->pdn_countdown = this->settings.project_on_demand_countdown;
            break;

        case ProjectionToPD::Progressive:
            if (this->ppn_threshold < 0.0) {
                this->ppn_threshold = this->grad.cwiseAbs().maxCoeff();
            }
            this->ppn_threshold *= this->settings.ppn_tightening_factor;
            break;

        default:
            break;  // Other modes don't have adaptive projection
    }
}
void NewtonsMethod::_decrease_projection()
{
    switch (this->settings.projection_mode) {
        case ProjectionToPD::ProjectOnDemand:
            this->pdn_countdown--;
            break;

        case ProjectionToPD::Progressive:
            this->ppn_threshold *= this->settings.ppn_release_factor;
            break;

        default:
            break;  // Other modes don't have adaptive projection
    }
}

bool NewtonsMethod::_solve_linear_system(Eigen::VectorXd& du, const ElementHessians::spBSM& hess, const Eigen::VectorXd& grad)
{
    auto timer_ls = this->logger->time("linear_system_solve");

    this->rhs = -grad;
    const int ndofs = (int)grad.size();

    if (this->settings.linear_solver == LinearSolver::DirectLU) {
        // Direct LU solver via Eigen
        std::vector<Eigen::Triplet<double>> triplets;
        hess->to_triplets(triplets);
        
        Eigen::SparseMatrix<double> sparse_hess(ndofs, ndofs);
        sparse_hess.setFromTriplets(triplets.begin(), triplets.end());
        
        Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
        solver.compute(sparse_hess);
        
        if (solver.info() != Eigen::Success) {
            return false;
        }

        du = solver.solve(this->rhs);
        
        return solver.info() == Eigen::Success;
    }
    
    // Block Diagonal Preconditioner Conjugate Gradient
    else if (this->settings.linear_solver == LinearSolver::BDPCG) {
        // Forcing sequence
        const double grad_norm = rhs.norm();
        const double forcing_cg_tol = std::min(1e-2, grad_norm * std::min(0.5, std::sqrt(grad_norm)));

        // Prepare preconditioning
        hess->set_preconditioner(bsm::Preconditioner::BlockDiagonal);
        hess->prepare_preconditioning(this->context->n_threads);

        // Initial guess
        du.resize(ndofs);
        du.setZero();

        // PCG solver
        bsm::PCGInfo pcg_info = bsm::solve_pcg(
            *hess,
            du.data(),
            this->rhs.data(),
            ndofs,
            // this->settings.cg_abs_tolerance,  // DEBUG
            forcing_cg_tol,  // DEBUG
            this->settings.cg_rel_tolerance,
            this->settings.cg_max_iterations,
            this->context->n_threads,
            this->settings.cg_stop_on_indefiniteness,
            this->pcg_context
        );

        this->output->print(fmt::format("#CG {:5d} | ", pcg_info.n_iterations), Verbosity::Full);
        this->stats.cg_iterations += pcg_info.n_iterations;
        this->last_cg_iterations = pcg_info.n_iterations;
        return pcg_info.converged;
    } 
    else {
        std::cout << "symx error: Unknown linear solver." << std::endl;
        exit(-1);
    }
}

SolverReturn NewtonsMethod::_line_search_inplace(double E0, double du_dot_grad, double du_max)
{
    /*
    * There are four line search checks. In order:
    *   - [cap] Hard cap to the user-specified maximum a single step can take
    *   - [max] Hard cap based on `max_allowed_step()` callbacks. E.g. CCD.
    *   - [inv] Backtracking based on `is_intermediate_state_valid()` callbacks. E.g. penetration or inversions
    *   - [bt]  Armijo sufficient descend backtracking based on global energy
    */

    // Define apply step function
    this->dofs_before_ls.resize(this->du.size());
    this->global_potential->get_dofs(this->dofs_before_ls.data());
    
    auto apply_scaled_du = [this](double s) {
        this->dofs_ls = this->dofs_before_ls + s * this->du;
        this->global_potential->set_dofs(this->dofs_ls.data());
    };
    
    /* ============================== Limit the step ============================== */
    /* ------------------------------------ Cap ----------------------------------- */
    if (du_max > this->settings.step_cap) {
        this->du *= this->settings.step_cap / du_max;
        du_max = this->settings.step_cap;
        this->output->print(fmt::format("du cap {:.1e} | ", du_max), Verbosity::Medium);
        this->stats.ls_cap_iterations++;
        logger->add_and_append("ls_cap", 1);
    } 
    else {
        logger->add_and_append("ls_cap", 0);
    }
    
    /* ------------------------------------ Max ----------------------------------- */
    double max_step = this->callbacks->run_max_allowed_step();
    if (du_max > max_step) {
        this->du *= max_step / du_max;
        du_max = max_step;
        this->output->print(fmt::format("ls max {:.1e} | ", du_max), Verbosity::Medium);
        this->stats.ls_max_iterations++;
        logger->add_and_append("ls_max", 1);
    }
    else {
        logger->add_and_append("ls_max", 0);
    }
    
    /* ============================== Backtracking ============================== */
    constexpr double shrink = 0.5;
    double step = 1.0;
    
    // Apply full step forward
    apply_scaled_du(step);
    
    /* ------------------------------------ Inv ----------------------------------- */
    int ls_inv_it = 0; 
    for ( ; ls_inv_it < this->settings.max_backtracking_invalid_state_iterations; ++ls_inv_it) {
        if (this->callbacks->run_is_intermediate_state_valid()) {
            break;
        } 
        else {
            this->output->print_with_new_line(fmt::format("{:d}. step: {:.2e} | Invalid state", ls_inv_it, step), Verbosity::Full);
            step *= shrink;
            apply_scaled_du(step);
            this->stats.ls_inv_iterations++;
        }
    }
    logger->add_and_append("ls_inv", ls_inv_it);
    
    // Print
    if (this->output->get_verbosity() != Verbosity::Full && ls_inv_it > 0) {
        this->output->print(fmt::format("ls inv {:2d} | ", ls_inv_it), Verbosity::Medium);
    }
    
    // Exit: Failed to find valid state within line search iterations
    if (ls_inv_it == this->settings.max_backtracking_invalid_state_iterations) {
        this->callbacks->run_on_intermediate_state_invalid();
        return SolverReturn::InvalidIntermediateConfiguration;
    }
    
    
    /* ------------------------------------ Armijo ----------------------------------- */
    if (!this->settings.enable_armijo_bracktracking) {
        return SolverReturn::Running;
    }

    // Fail mode: Generate data for plotting the energy profile along the step
    if (this->_ls_logging_mode) {

        // Undo current step to get back to x0
        apply_scaled_du(step);
        
        // Sample energy from -0.5*du to 1.5*du using 1000 samples
        const int n_samples = 1000;
        std::vector<double> samples(n_samples);
        for (int i = 0; i < n_samples; ++i) {
            double s = -0.5 + 2.0 * static_cast<double>(i) / static_cast<double>(n_samples - 1);
            apply_scaled_du(s);
            double E_sample = 0.0;
            this->callbacks->run_before_energy_evaluation();
            this->compiled->evaluate_P(E_sample);
            this->callbacks->run_after_energy_evaluation();
            samples[i] = E_sample;
        }
        this->_ls_energy_samples.push_back(samples);
    }

    // Sufficient descend loop
    const double expected_decrease = this->settings.line_search_armijo_beta * du_dot_grad;
    const double E_threshold = E0 + expected_decrease;
    double E1 = 0.0;
    bool success = false;
    int armijo_iterations = 0;
    for (; armijo_iterations < this->settings.max_backtracking_armijo_iterations; ++armijo_iterations) {
        
        // Evaluate energy
        this->callbacks->run_before_energy_evaluation();
        this->compiled->evaluate_P(E1);
        this->callbacks->run_after_energy_evaluation();
        
        // Print
        this->output->print_with_new_line(fmt::format("{:d}. step: {:.2e} | E: {:.2e} | E_bt: {:.2e} | E/E_bt: {:.2e} | ", armijo_iterations, step, E1, E_threshold, E1 / E_threshold), Verbosity::Full);
        
        // Check Armijo condition
        if (E1 < E_threshold) {
            success = true;
            break;
        }
        else {
            step *= shrink;
            apply_scaled_du(step);
            this->stats.ls_bt_iterations++;
            this->output->print("Backtrack", Verbosity::Full);
        }
    }

    // Print
    logger->add_and_append("ls_bt", armijo_iterations);
    if (this->output->get_verbosity() != Verbosity::Full && armijo_iterations > 0) {
        this->output->print(fmt::format("ls bt {:2d} | ", armijo_iterations), Verbosity::Medium);
    }

    // Success
    if (success) {
        return SolverReturn::Running;
    }

    // Failure mode: Visualize line search failure if in logging mode
    if (this->settings.print_line_search_upon_failure) {

        // The whole Newton search needs to be recorded
        if (!this->_ls_logging_mode) {
            this->_ls_logging_mode = true;

            // Restore initial state and run
            this->global_potential->set_dofs(this->initial_dofs.data());
            SolverReturn retry_result = this->solve();
        
            // If the second attempt succeeds, something is wrong (non-deterministic behavior)
            if (retry_result == SolverReturn::Successful) {
                std::cout << "Error: Line search debug retry succeeded unexpectedly. Non-deterministic behavior detected." << std::endl;
                exit(1);
            }
        }

        // Generate logs and plots
        visualize_line_search_failure(
            this->_ls_energy_samples,
            this->context->compilation_directory,
            E0,
            E_threshold,
            du_dot_grad
        );

        // Exit
        std::cout << "Exiting stark..." << std::endl;
        exit(-1);
    }

    return SolverReturn::TooManyLineSearchIterations;
}

void NewtonsMethod::print_summary(double total_time) const
{
	auto* out = this->output.get();

	// ── Solve table ──
	//const double n_hess = logger->get_double("n_hessians");
	//const double n_proj = logger->get_double("n_projected_hessians");
	//const double proj_ratio = n_hess > 0 ? 100.0 * n_proj / n_hess : 0.0;

	out->print_with_new_line("");
	out->print_with_new_line(fmt::format("  {:<24} {:>10} {:>8} {:>8} {:>8}", "Solve", "Total", "Avg", "Min", "Max"));
	out->print_with_new_line(fmt::format("  {}", std::string(62, '-')));
	{
		std::vector<std::pair<std::string, std::string>> rows = {
			{"Newton iterations", "newton_iterations"},
			{"CG iterations", "cg_iterations"},
			{"Line search cap", "ls_cap"},
			{"Line search max", "ls_max"},
			{"Line search inv", "ls_inv"},
			{"Line search bt", "ls_bt"},
		};
		for (const auto& [label, key] : rows) {
			auto s = logger->get_stats(key);
			out->print_with_new_line(fmt::format("  {:<24} {:>10} {:>8.1f} {:>8} {:>8}",
				label, (long long)s.total, s.avg, (int)s.min, (int)s.max));
		}
	}
    auto s = logger->get_stats("projected_hessians_ratio");
	out->print_with_new_line(fmt::format("  {:<24} {:>10} {:>8.1f}% {:>7.1f}% {:>7.1f}%", "Projected hessians", "", 100.0*s.avg, 100.0*s.min, 100.0*s.max));

	// Compute total_time if not provided
	if (total_time <= 0.0) {
		total_time = 0.0;
		for (const auto& label : logger->get_timer_labels()) {
			total_time += logger->get_timer_total(label);
		}
	}
	out->print_with_new_line(fmt::format("  {}", std::string(62, '-')));

	// ── Runtime table — sorted by decreasing time ──
	struct TimerEntry { std::string label; double time; };
	std::vector<TimerEntry> timer_entries;
	double acc = 0.0;
	for (const auto& label : logger->get_timer_labels()) {
		if (label == "total") continue;
		double time = logger->get_timer_total(label);
		acc += time;
		if (total_time > 0.0 && time / total_time < 0.001) continue;
		timer_entries.push_back({label, time});
	}
	const double total_logged = logger->get_timer_total("total");
	const double misc = (total_logged > 0.0 ? total_logged : total_time) - acc;
	timer_entries.push_back({"misc", misc});
	std::sort(timer_entries.begin(), timer_entries.end(),
		[](const TimerEntry& a, const TimerEntry& b) { return a.time > b.time; });

	out->print_with_new_line("");
	out->print_with_new_line(fmt::format("  {:<40} {:>10}  {:>6}", "Runtime", "Time (s)", "%"));
	out->print_with_new_line(fmt::format("  {}", std::string(60, '-')));

	for (const auto& entry : timer_entries) {
		const double pct = total_time > 0 ? 100.0 * entry.time / total_time : 0.0;
		out->print_with_new_line(fmt::format("  {:<40} {:>10.6f}  {:>5.1f}%", entry.label, entry.time, pct));
	}

	out->print_with_new_line(fmt::format("  {}", std::string(60, '-')));
	out->print_with_new_line(fmt::format("  {:<40} {:>10.6f}  {:>5.1f}%", "Total", total_time, 100.0));
	out->print_new_line();
}
