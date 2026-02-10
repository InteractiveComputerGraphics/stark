#include "SecondOrderCompiledGlobal.h"

#include <Eigen/Sparse>
#include "../solver_utils.h"

using namespace symx;

SecondOrderCompiledGlobal::SecondOrderCompiledGlobal(spGlobalPotential global_potential, spContext context)
    : global_potential(global_potential), context(context)
{
    // Get
    const std::vector<DataMap<double>>& dofs_maps = global_potential->get_dof_maps();
    const std::vector<std::unique_ptr<Potential>>& potentials = global_potential->get_potentials();
    // Verify inputs
    if (dofs_maps.size() == 0) {
        std::cout << "symx error: SecondOrderCompiledGlobal() got GlobalPotential with zero DoF maps." << std::endl;
        exit(-1);
    }
    if (potentials.size() == 0) {
        std::cout << "symx error: SecondOrderCompiledGlobal() got GlobalPotential with zero Potentials." << std::endl;
        exit(-1);
    }

    // Initialize compiled potentials
    const int n_potentials = (int)potentials.size();
    this->compiled_potentials.resize(n_potentials);
    context->output->print_with_new_line("Second Order Potentials:");

    // Parallel init
    DeferredParallelTasks tasks;

    //// Try to load / queue compilation
    const double t0 = omp_get_wtime();
    #pragma omp parallel for num_threads(context->n_threads) schedule(dynamic)
    for (int i = 0; i < n_potentials; i++) {
        const Potential& potential = *potentials[i];
        
        const double ta = omp_get_wtime();
        this->compiled_potentials[i] = std::make_unique<SecondOrderCompiledPotential>(potential, dofs_maps, context->compilation_directory, tasks);
        const double tb = omp_get_wtime();
        
		#pragma omp critical
		{
            if (compiled_potentials[i]->was_cached()) {
                context->output->print_with_new_line(in_two_columns(potentials[i]->get_name(), "loaded", 70), Verbosity::Summary);
			}
			else {
                context->output->print_with_new_line(in_two_columns(potentials[i]->get_name(), "queued for compilation", 70), Verbosity::Summary);
			}
		}
    }
    
    //// Run compilation tasks
    context->output->print_with_new_line("Compiling... ");
    tasks.run(context->n_threads);
    context->output->print("done.", Verbosity::Summary);
    const double t1 = omp_get_wtime();
    context->output->print_with_new_line("Total time: " + std::to_string(t1 - t0) + " s");
}

void SecondOrderCompiledGlobal::evaluate_P(double &out_P)
{
    // Assembly options
    const bool hess = false;
    const bool grad = false;

    // Reset assembly
    this->assembly.start(global_potential->get_dofs_offsets(), context->n_threads, hess, grad);

    // Evaluate potentials
    for (const auto& compiled_potential : this->compiled_potentials) {
        compiled_potential->evaluate_P(this->assembly);
    }

    // Finish
    this->assembly.stop(hess, grad);

    // Return
    out_P = this->assembly.E.get_solution();
}

void SecondOrderCompiledGlobal::evaluate_P__dP_du(double& out_P, Eigen::VectorXd& out_dP_du)
{
    // Assembly options
    const bool hess = false;
    const bool grad = true;

    // Reset assembly
    this->assembly.start(global_potential->get_dofs_offsets(), context->n_threads, hess, grad);

    // Evaluate potentials
    for (const auto& compiled_potential : this->compiled_potentials) {
        compiled_potential->evaluate_P__dP_du(this->assembly);
    }

    // Finish
    this->assembly.stop(hess, grad);

    // Return
    out_P = this->assembly.E.get_solution();
    out_dP_du = this->assembly.grad.get_solution();
}

spElementHessians SecondOrderCompiledGlobal::evaluate_P__dP_du__local_d2P_du2(double& out_P, Eigen::VectorXd& out_dP_du)
{
    // Assembly options
    const bool hess = true;
    const bool grad = true;

    // Reset assembly
    this->assembly.start(global_potential->get_dofs_offsets(), context->n_threads, hess, grad);

    // Evaluate potentials
    for (const auto& compiled_potential : this->compiled_potentials) {
        compiled_potential->evaluate_P__dP_du__local_d2P_du2(this->assembly);
    }

    // Finish
    this->assembly.stop(hess, grad);

    // Return
    out_P = this->assembly.E.get_solution();
    out_dP_du = this->assembly.grad.get_solution();
    return this->assembly.element_hessians;
}

double SecondOrderCompiledGlobal::test_derivatives_with_FD(const double h)
{
	// Get the current dofs
	const int ndofs = this->global_potential->get_total_n_dofs();
	Eigen::VectorXd u = Eigen::VectorXd::Zero(ndofs);
	global_potential->get_dofs(u.data());

	// Evaluate
    double E = 0.0;
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(ndofs);
	auto element_hessians = this->evaluate_P__dP_du__local_d2P_du2(E, grad);

    // Assemble global Hessian
    auto bsm_hess = element_hessians->assemble_global(context->n_threads, ndofs);

	std::vector<Eigen::Triplet<double>> triplets;
	bsm_hess->to_triplets(triplets);
	Eigen::SparseMatrix<double> hess_sparse(ndofs, ndofs);
	hess_sparse.setFromTriplets(triplets.begin(), triplets.end());
	const Eigen::MatrixXd hess = hess_sparse.toDense();

	// Compute the FD gradient and Hessian
	Eigen::VectorXd grad_FD(ndofs);
	Eigen::MatrixXd hess_FD(ndofs, ndofs);
	for (int i = 0; i < ndofs; i++) {
		u[i] += h;
		global_potential->set_dofs(u.data());
		double E_plus = 0.0;
		Eigen::VectorXd grad_plus = Eigen::VectorXd::Zero(ndofs);
		this->evaluate_P__dP_du(E_plus, grad_plus);

		u[i] -= 2.0 * h;
		
		global_potential->set_dofs(u.data());
		double E_minus = 0.0;
		Eigen::VectorXd grad_minus = Eigen::VectorXd::Zero(ndofs);
		this->evaluate_P__dP_du(E_minus, grad_minus);
		u[i] += h;

		grad_FD[i] = (E_plus - E_minus) / (2.0 * h);
		hess_FD.row(i) = (grad_plus - grad_minus) / (2.0 * h);
	}

	// Compare
	double max_diff_grad = (grad - grad_FD).cwiseAbs().maxCoeff();
	double max_diff_hess = (hess - hess_FD).cwiseAbs().maxCoeff();
	return std::max(max_diff_grad, max_diff_hess);
}
