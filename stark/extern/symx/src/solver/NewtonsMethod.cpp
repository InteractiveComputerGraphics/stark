#include "NewtonsMethod.h"
#include "gnuplot.h"
#include <fmt/format.h>
#include <BlockedSparseMatrix/solve_pcg.h>
#include <Eigen/SparseLU>
#include <limits>
#include <omp.h>
#include <algorithm>

using namespace symx;

symx::NewtonsMethod::NewtonsMethod(spGlobalPotential global_potential, spContext context)
    : global_potential(global_potential), context(context)
{
    this->output = context->output;
    this->compiled = std::make_shared<SecondOrderCompiledGlobal>(global_potential, context);
}

spNewtonsMethod symx::NewtonsMethod::create(spGlobalPotential global_potential, spContext context)
{
    return std::make_shared<NewtonsMethod>(global_potential, context);
}

// Force rebuild
SolverReturn NewtonsMethod::solve()
{
    auto timer_total = log.scope("Total Solve Time");

    // Check
    if (!global_potential) {
        std::cout << "Error: GlobalPotential not set." << std::endl;
        exit(1);
    }

    const int ndofs = global_potential->get_total_n_dofs();
    if (ndofs <= 0) {
        std::cout << "Error: No degrees of freedom." << std::endl;
        exit(1);
    }

    // Clear logging state
    this->_ls_logging_mode = false;
    this->_ls_energy_samples.clear();

    // Save initial DOF state for potential retry with logging
    Eigen::VectorXd initial_dofs(ndofs);
    this->global_potential->get_dofs(initial_dofs.data());

    // Run solve
    SolverReturn result = this->_solve_impl();

    // Retry with logging if line search failed and flag is set
    if (result == SolverReturn::TooManyLineSearchIterations && 
        this->settings.print_line_search_upon_failure && 
        !this->_ls_logging_mode) {
        
        // Restore initial state
        this->global_potential->set_dofs(initial_dofs.data());
        
        this->_ls_logging_mode = true;
        this->_ls_energy_samples.clear();
        
        SolverReturn retry_result = this->_solve_impl();
        
        // If the second attempt succeeds, something is wrong (non-deterministic behavior)
        if (retry_result == SolverReturn::Successful) {
            std::cout << "Error: Line search debug retry succeeded unexpectedly. Non-deterministic behavior detected." << std::endl;
            exit(1);
        }
        
        return retry_result;
    }

    return result;
}

SolverReturn NewtonsMethod::_solve_impl()
{
    const int ndofs = global_potential->get_total_n_dofs();
    const int n_dof_blocks = ndofs / 3;

    // Set up
    this->stats = SolveStats(); // Reset stats
    du.resize(ndofs);
    step_du.resize(ndofs);
    rhs.resize(ndofs);
    grad.resize(ndofs);
    double E0 = 0.0;
    double du_dot_grad = 0.0;
    spElementHessians element_hessians = nullptr;
    ElementHessians::spBSM hess = nullptr;
    SolverReturn result = SolverReturn::Running;

    // Projection state (persists across Newton iterations for adaptive modes)
    this->pdn_countdown = 0;     // ProjectOnDemand: iterations remaining with full projection
    this->ppn_threshold = -1.0;  // Progressive: gradient threshold (-1 = not yet determined)
    this->ppn_active_dof_blocks_for_projection.assign(n_dof_blocks, static_cast<uint8_t>(false));

    // Check initial state validity
    if (!this->callbacks.run_is_initial_state_valid()) {
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
        this->output->print(fmt::format("\n{}{:3d}. ", this->settings.output_prefix, newton_iteration), Verbosity::Step);

        // Evaluate energy, gradient, and Hessian
        this->callbacks.run_before_energy_evaluation();
        {
            auto timer_eval = this->log.scope("Energy Evaluation");
            element_hessians = this->compiled->evaluate_P__dP_du__local_d2P_du2(E0, this->grad);
        }
        this->callbacks.run_after_energy_evaluation();

        // Compute residual
        double residual_norm = this->callbacks.compute_residual(this->grad);
        this->output->print(fmt::format("r0 = {:.2e} | ", residual_norm), Verbosity::Step);

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
        bool linear_system_solve_success = false;
        while (!linear_system_solve_success) {

            // Project element Hessians and assemble global Hessian
            bool all_projected = this->_project_and_assemble(element_hessians, hess, ndofs);

            // Solve linear system
            {
                auto timer_ls = this->log.scope("Linear System Solve");
                linear_system_solve_success = this->_solve_linear_system(this->du, hess, this->grad);
            }

            // Handle linear system failure
            if (!linear_system_solve_success) {
                bool can_project_more = (this->settings.projection_mode != ProjectionToPD::Newton) && !all_projected;
                if (!can_project_more) {
                    this->output->print("Linear system failed. ", Verbosity::Step);
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
                        this->output->print("Does not descend. ", Verbosity::Step);
                        result = SolverReturn::LineSearchDoesNotDescend;
                        break;
                    }
                }
            }

            // Success: linear system solved and direction descends
            if (linear_system_solve_success && descends) {
                this->stats.n_hessians += element_hessians->size();
                this->stats.n_projected_hessians += element_hessians->n_projected();
                break;
            }

            // Failed: request more projection and retry
            this->_increase_projection();
            this->output->print("Retrying with more projection. \n\t\t     ", Verbosity::Step);

        } // End of linear system solve loop

        if (result != SolverReturn::Running) {
            break;
        }

        // Solve successful
        this->_decrease_projection();

        // Convergence: Step tolerance
        double du_max = this->du.cwiseAbs().maxCoeff();
        this->output->print(fmt::format("du = {:.1e} | ", du_max), Verbosity::Step);
        if (newton_iteration >= this->settings.min_iterations && du_max < this->settings.step_tolerance) {
            result = SolverReturn::Successful;
            break;
        }

        // Apply step size limit
        if (du_max > this->settings.step_cap) {
            this->du *= this->settings.step_cap / du_max;
            du_max = this->settings.step_cap;
            this->output->print(fmt::format("du cap {:.1e} | ", du_max), Verbosity::Step);
            this->stats.max_step_iterations++;
        }

        // Max allowed step (e.g. CCD)
        double max_step = this->callbacks.run_max_allowed_step();
        if (du_max > max_step) {
            this->du *= max_step / du_max;
            du_max = max_step;
            this->output->print(fmt::format("ls hit {:.1e} | ", du_max), Verbosity::Step);
            this->stats.max_step_iterations++;
        }

        // Line search
        {
            auto timer_linesearch = this->log.scope("Line Search");
            int armijo_iterations = 0;
            result = this->_line_search_inplace(armijo_iterations, E0, du_dot_grad);
            if (armijo_iterations > 0) {
                this->_increase_projection();
            }
        }
        if (result != SolverReturn::Running) {
            break;
        }
    }

    // Check if converged state is valid
    if (result == SolverReturn::Successful && !this->callbacks.run_is_converged_state_valid()) {
        result = SolverReturn::InvalidConvergedState;
    }

    // End
    this->stats.newton_iterations = newton_iteration + 1;
    if (this->stats.n_hessians > 0) {
        this->stats.projected_hessians_ratio = (double)this->stats.n_projected_hessians / (double)this->stats.n_hessians;
    }
    // this->_print_return(result, newton_iteration, this->total_line_search_iterations);
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
        auto timer_assembly = this->log.scope("Hessian Assembly");
        hess = element_hessians->assemble_global(this->context->n_threads, ndofs);
    }

    // Project element Hessians to PD
    {
        auto timer_eval = this->log.scope("Project to PD");

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

        this->output->print(fmt::format("ph = {:5.1f}% | ", 100.0 * projection_ratio), Verbosity::Step);
    }

    // Assemble or update global Hessian
    {
        auto timer_assembly = this->log.scope("Hessian Assembly");
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
            this->settings.cg_abs_tolerance,
            this->settings.cg_rel_tolerance,
            this->settings.cg_max_iterations,
            this->context->n_threads,
            this->settings.cg_stop_on_indefiniteness,
            this->pcg_context
        );

        this->output->print(fmt::format("#CG {:5d} | ", pcg_info.n_iterations), Verbosity::Step);
        this->stats.cg_iterations += pcg_info.n_iterations;
        return pcg_info.converged;
    } 
    else {
        std::cout << "symx error: Unknown linear solver." << std::endl;
        exit(-1);
    }
}

SolverReturn NewtonsMethod::_line_search_inplace(int& armijo_iterations, double E0, double du_dot_grad)
{
    // Backtracking line search with Armijo condition.
    // First ensures intermediate state validity, then checks energy descent.
    
    Eigen::VectorXd original_dofs(this->du.size());
    this->global_potential->get_dofs(original_dofs.data());
    
    auto apply_scaled_du = [this, &original_dofs](double s) {
        this->step_du = original_dofs + s * this->du;
        this->global_potential->set_dofs(this->step_du.data());

        // this->global_potential->apply_dof_increment(this->step_du.data());
    };
    
    constexpr double shrink = 0.5;
    double step = 1.0;
    int it = 0;
    armijo_iterations = 0;

    // Apply full step forward
    apply_scaled_du(step);
    
    // First loop: find a valid intermediate state
    for (; it < this->settings.max_line_search_iterations; ++it) {
        
        if (this->callbacks.run_is_intermediate_state_valid()) {
            break;
        } 
        else {
            this->output->print(fmt::format("\n\t{}{:d}. step = {:.2e} | Invalid state", 
                this->settings.output_prefix, it, step), Verbosity::Full);
            step *= shrink;
            apply_scaled_du(step);  // Undo and apply smaller step
            this->stats.max_step_iterations++;
        }
    }

    // Failed to find valid state within line search iterations
    if (it == this->settings.max_line_search_iterations) {
        this->callbacks.run_on_intermediate_state_invalid();
        return SolverReturn::InvalidIntermediateConfiguration;
    }
    
    if (!this->settings.enable_armijo_bracktracking) {
        return SolverReturn::Running;
    }

    // Sample energy for logging if in logging mode
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
            this->callbacks.run_before_energy_evaluation();
            this->compiled->evaluate_P(E_sample);
            this->callbacks.run_after_energy_evaluation();
            samples[i] = E_sample;
            // apply_scaled_du(-s);  // Undo
        }
        this->_ls_energy_samples.push_back(samples);
        
        // // Re-apply the current step
        // apply_scaled_du(step);
    }

    // Second loop: check Armijo condition (sufficient energy descent)
    // Armijo: E(x + s*du) <= E(x) + beta * <du, gradE>
    const double expected_decrease = this->settings.line_search_armijo_beta * du_dot_grad;
    const double E_threshold = E0 + expected_decrease;
    double E1 = 0.0;
    for (; it < this->settings.max_line_search_iterations; ++it) {
        armijo_iterations++;
        
        // Evaluate energy
        this->callbacks.run_before_energy_evaluation();
        this->compiled->evaluate_P(E1);
        this->callbacks.run_after_energy_evaluation();
        
        // Print
        if (it > 0) {
            this->output->print(fmt::format("\n\t{}{:d}. step = {:.2e} | E/E_bt = {:.6e} | E/E0 = {:.6e}",
                this->settings.output_prefix, it, step, E1 / E_threshold, E1 / E0),
                Verbosity::Full);
        }
            
        // Check Armijo condition
        if (E1 < E_threshold) {
            return SolverReturn::Running;
        }
        else {
            step *= shrink;
            apply_scaled_du(step);  // Undo and apply smaller step
            this->stats.line_search_iterations++;
        }
    }

    // Visualize line search failure if in logging mode
    if (this->_ls_logging_mode && !this->_ls_energy_samples.empty()) {
        visualize_line_search_failure(
            this->_ls_energy_samples,
            this->context->compilation_directory,
            E0,
            E_threshold,
            du_dot_grad
        );
    }

    return SolverReturn::TooManyLineSearchIterations;
}

void NewtonsMethod::_print_return(SolverReturn result, int newton_iterations, int ls_iterations) const
{
    this->output->print(fmt::format("\n{}#newton: {:d} ", this->settings.output_prefix, newton_iterations), Verbosity::Step);

    if (newton_iterations > 0) {
        this->output->print(fmt::format(" | #ls/newton: {:.2f}", (double)(ls_iterations) / (double)newton_iterations), Verbosity::Step);
    } else {
        this->output->print(" | #ls/newton: N/A", Verbosity::Step);
    }
    
    if (result == SolverReturn::Successful) {
        this->output->print(" | converged", Verbosity::Step);
    } else {
        this->output->print(" | not converged", Verbosity::Step);
    }

    switch (result) {
        case SolverReturn::TooManyNewtonIterations:
            this->output->print(fmt::format("\n\t\t -> Max Newton iterations reached ({:d}). ", this->settings.max_iterations), Verbosity::Step);
            break;
        case SolverReturn::TooManyLineSearchIterations:
            this->output->print(fmt::format("\n\t\t -> Max line search iterations reached ({:d}). ", this->settings.max_line_search_iterations), Verbosity::Step);
            break;
        case SolverReturn::LinearSystemFailure:
            this->output->print("\n\t\t -> Linear system could not find a solution. ", Verbosity::Step);
            break;
        case SolverReturn::InvalidIntermediateConfiguration:
            this->output->print("\n\t\t -> Invalid intermediate configuration could not be avoided. ", Verbosity::Step);
            break;
        case SolverReturn::LineSearchDoesNotDescend:
            this->output->print("\n\t\t -> Line search direction does not descend. ", Verbosity::Step);
            break;
        case SolverReturn::InvalidConvergedState:
            this->output->print("\n\t\t -> Converged state is invalid. ", Verbosity::Step);
            break;
        case SolverReturn::Successful:
        case SolverReturn::Running:
            break;  // No error message needed
    }
    this->output->print("\n", Verbosity::Step);
}
