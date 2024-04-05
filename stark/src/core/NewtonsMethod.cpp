#include "NewtonsMethod.h"

#include <symx>
#include <fmt/format.h>
#include <BlockedSparseMatrix/ConjugateGradientMethod.h>


stark::core::NewtonState stark::core::NewtonsMethod::solve(const double& dt, symx::GlobalEnergy& global_energy, Callbacks& callbacks, const Settings& settings, Console& console, Logger& logger)
{
	// Set pointers for easy access accross methods
	this->global_energy = &global_energy;
	this->callbacks = &callbacks;
	this->settings = &settings;
	this->console = &console;
	this->logger = &logger;

	// Short naming
	const int ndofs = global_energy.get_total_n_dofs();
	const int n_nodes = ndofs / 3;

	// Buffers
	this->u0.resize(ndofs);
	this->u1.resize(ndofs);
	this->residual.resize(ndofs);

	// Counters
	this->step_newton_it = 0;
	this->step_line_search_count = 0;
	this->cg_iterations_in_step = 0;

	// Newtons method
	double residual_max = std::numeric_limits<double>::max();
	NewtonState newton_state = NewtonState::Running;
	while (newton_state == NewtonState::Running) {
		this->step_newton_it++;

		// Exit due to max newton iterations
		if (this->step_newton_it == settings.newton.max_newton_iterations) {
			newton_state = NewtonState::TooManyNewtonIterations;
			break;
		}

		// Test
		if (settings.debug.symx_finite_difference_check) {
			this->global_energy->test_derivatives_with_FD(1e-4);
			this->global_energy->test_derivatives_with_FD(1e-6);
			this->global_energy->test_derivatives_with_FD(1e-8);
		}

		// Print line header
		console.print(fmt::format("\n\t\t {:d}. ", this->step_newton_it), ConsoleVerbosity::NewtonIterations);

		// Assemble linear system
		symx::Assembled assembled = this->_evaluate_E_grad_hess();

		// Energy before the step
		const double E0 = *assembled.E;

		// Residual before the step
		this->residual = this->_compute_residual(*assembled.grad, dt);
		residual_max = this->residual.maxCoeff();
		console.print(fmt::format("r0 = {:.2e} | ", residual_max), ConsoleVerbosity::NewtonIterations);

		//// Converged right away? 
		//// Can happen in the first time step sometimes (e.g. no gravity). 
		//// Avoids unnecessary CG iterations and problems with the forcing sequence.
		//// If this uses the standard residual, it may kill visible oscillations if not even a single newton iteration is run.
		if (residual_max < this->settings->newton.epsilon_residual) {
			newton_state = NewtonState::Successful;
			break;
		}

		// Solve linear system
		const bool linear_system_success = this->_solve_linear_system(this->du, assembled, dt);
		if (!linear_system_success) {
			newton_state = NewtonState::LinearSystemFailure;
			break;
		}

		// Correction step
		const double max_da = this->_compute_acceleration_correction(this->du.cwiseAbs().maxCoeff(), dt);
		console.print(fmt::format("da = {:.2e} | ", max_da), ConsoleVerbosity::NewtonIterations);

		// Sufficient descend
		const double du_dot_grad = this->du.dot(*assembled.grad);
		if (du_dot_grad > 0.0) {
			newton_state = NewtonState::LineSearchDoesntDescend;
			break;
		}

		// Max step in the search direction
		double step_valid_configuration = this->_inplace_max_step_in_search_direction(this->du);
		if (step_valid_configuration < 0.01) {
			callbacks.run_on_intermidiate_state_invalid();
			newton_state = NewtonState::InvalidIntermediateConfiguration;
			break;
		}

		// Residual after a valid step
		assembled = this->_evaluate_E_grad();
		this->residual = this->_compute_residual(*assembled.grad, dt);
		residual_max = this->residual.maxCoeff();
		console.print(fmt::format("r1 = {:.2e}", residual_max), ConsoleVerbosity::NewtonIterations);

		//// Converged?
		if (residual_max < this->settings->newton.residual.tolerance) {
			newton_state = NewtonState::Successful;
			break;
		}

		// Line search: Backtracking (Nocedal)
		double step_that_worked = this->_inplace_backtracking_line_search(this->du, E0, *assembled.E, step_valid_configuration, du_dot_grad);
		if (step_that_worked == 0.0) {
			newton_state = NewtonState::TooManyLineSearchIterations;
			break;
		}
	} // Newton iterations

	// Is converged state valid?
	if (!callbacks.run_is_converged_state_valid()) {
		newton_state = NewtonState::InvalidConvergedState;
	}

	// Print
	console.print("\n\t\t", ConsoleVerbosity::NewtonIterations);
	console.print(fmt::format("#newton: {:d} ", this->step_newton_it), ConsoleVerbosity::TimeSteps);
	console.print(" | #CG/newton: " + std::to_string((int)(this->cg_iterations_in_step / this->step_newton_it)), ConsoleVerbosity::TimeSteps);
	console.print(" | #line_search/newton: " + std::to_string((int)(this->step_line_search_count / this->step_newton_it)), ConsoleVerbosity::TimeSteps);
	if (newton_state == NewtonState::Successful) {
		console.print(" | converged", ConsoleVerbosity::TimeSteps);
	}
	else {
		console.print(" | not converged", ConsoleVerbosity::TimeSteps);
	}

	// Print error message
	switch (newton_state)
	{
		case NewtonState::TooManyNewtonIterations:
			console.print(fmt::format("\n\t\t -> Max Newton iterations reached ({:d}) with residual_max {:.2e}. ", settings.newton.max_newton_iterations, residual_max), ConsoleVerbosity::TimeSteps);
			break;
		case NewtonState::TooManyLineSearchIterations:
			console.print(fmt::format("\n\t\t -> Max line search iterations reached ({:d}). ", settings.newton.max_line_search_iterations), ConsoleVerbosity::TimeSteps);
			break;
		case NewtonState::LinearSystemFailure:
			console.print("\n\t\t -> Linear system couldn't find a solution. ", ConsoleVerbosity::TimeSteps);
			break;
		case NewtonState::InvalidIntermediateConfiguration:
			console.print("\n\t\t -> Invalid intermediate configuration couldn't be avoided. ", ConsoleVerbosity::TimeSteps);
			break;
		case NewtonState::LineSearchDoesntDescend:
			console.print("\n\t\t -> Line search doesn't descend. ", ConsoleVerbosity::TimeSteps);
			break;
		case NewtonState::InvalidConvergedState:
			console.print("\n\t\t -> Converged state is not valid. ", ConsoleVerbosity::TimeSteps);
			break;
		default:
			break;
	}
	if (newton_state != NewtonState::Successful) {
		this->console->print_error_msg_and_clear(ConsoleVerbosity::TimeSteps);
	}

	// Log
	logger.add_to_counter("newton_iterations", this->step_newton_it);
	logger.add_to_counter("CG_iterations", this->cg_iterations_in_step);
	logger.add_to_counter("line_search_iterations", this->step_line_search_count);

	// Return
	return newton_state;
}

void stark::core::NewtonsMethod::_run_before_evaluation()
{
	this->logger->start_timing("before_energy_evaluation");
	this->callbacks->run_before_energy_evaluation();
	this->logger->stop_timing_add("before_energy_evaluation");
}
void stark::core::NewtonsMethod::_run_after_evaluation()
{
	this->logger->start_timing("after_energy_evaluation");
	this->callbacks->run_after_energy_evaluation();
	this->logger->stop_timing_add("after_energy_evaluation");
}
symx::Assembled stark::core::NewtonsMethod::_evaluate_E_grad_hess()
{
	this->_run_before_evaluation();

	this->logger->start_timing("evaluate_E_grad_hess");
	symx::Assembled assembled = this->global_energy->evaluate_E_grad_hess();
	this->logger->stop_timing_add("evaluate_E_grad_hess");

	this->_run_after_evaluation();

	return assembled;
}
symx::Assembled stark::core::NewtonsMethod::_evaluate_E_grad()
{
	this->_run_before_evaluation();

	this->logger->start_timing("evaluate_E_grad");
	symx::Assembled assembled = this->global_energy->evaluate_E_grad();
	this->logger->stop_timing_add("evaluate_E_grad");

	this->_run_after_evaluation();

	return assembled;
}
symx::Assembled stark::core::NewtonsMethod::_evaluate_E()
{
	this->_run_before_evaluation();

	this->logger->start_timing("evaluate_E");
	symx::Assembled assembled = this->global_energy->evaluate_E();
	this->logger->stop_timing_add("evaluate_E");

	this->_run_after_evaluation();

	return assembled;
}
Eigen::VectorXd stark::core::NewtonsMethod::_compute_residual(const Eigen::VectorXd& grad, double dt)
{
	/*
		Note:
			The degrees of freedom are velocities, not displacements.
			Therefore the residual force used for termination is f = dE/dv * 1.0/time_step_size.
	*/
	if (this->settings->newton.residual.type == ResidualType::Force) {
		return grad.cwiseAbs() / dt;
	}
	else if (this->settings->newton.residual.type == ResidualType::Acceleration) {
		std::vector<int> dofs_offsets = this->global_energy->get_dofs_offsets();
		this->residual = grad.cwiseAbs() / dt; // Force
		int dof_count = 0;

		std::unordered_map<int, std::function<void(double*, double*)>> inv_mass_application = this->callbacks->get_inv_mass_application();
		for (auto& pair : inv_mass_application) {
			const int dof = pair.first;
			auto& inv_mass_application_f = pair.second;

			const int begin = dofs_offsets[dof];
			const int end = dofs_offsets[dof + 1];
			const int n = end - begin;

			dof_count += n;
			inv_mass_application_f(this->residual.data() + begin, this->residual.data() + end);
		}

		// Check if all dofs were used
		if (dof_count != (int)this->residual.size()) {
			std::cout << "Stark error: NewtonsMethod::_compute_residual() found that not all dofs were used for ResidualType::Acceleration." << std::endl;
			exit(-1);
		}

		return this->residual;
	}
	else {
		std::cout << "Stark error: NewtonsMethod::_compute_residual() found an unknown residual type." << std::endl;
		exit(-1);
	}
}
double stark::core::NewtonsMethod::_compute_acceleration_correction(double du, double dt)
{
	return du / dt;  // Degrees of freedom are velocities in Stark. Therefore the correction du is also a velocity.
}
double stark::core::NewtonsMethod::_forcing_sequence(const Eigen::VectorXd& rhs)
{
	const double grad_norm = rhs.norm();
	const double cg_tol = std::min(0.1 /*TODO: arbritrary.*/, grad_norm * std::min(0.5, std::sqrt(grad_norm)));
	return cg_tol;
}

bool stark::core::NewtonsMethod::_solve_linear_system(Eigen::VectorXd& du, const symx::Assembled& assembled, double dt)
{
	// Right hand side
	Eigen::VectorXd rhs = -1.0 * (*assembled.grad);

	// Solve
	if (this->settings->newton.linear_system_solver == LinearSystemSolver::DirectLU) {
		this->logger->start_timing("directLU");
		std::vector<Eigen::Triplet<double>> triplets;
		assembled.hess->to_triplets(triplets);

		Eigen::SparseMatrix<double> s;
		s.resize(rhs.size(), rhs.size());
		s.setFromTriplets(triplets.begin(), triplets.end());
		s.makeCompressed();

		Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> lu;
		lu.analyzePattern(s);
		lu.factorize(s);
		du = lu.solve(rhs);
		this->logger->stop_timing_add("directLU");

		if (lu.info() != Eigen::ComputationInfo::Success) {
			this->console->add_error_msg("DirectLU couldn't find a solution.");
			return false;
		}
		else {
			return true;
		}
	}
	else if (this->settings->newton.linear_system_solver == LinearSystemSolver::CG) {
		this->logger->start_timing("CG");
		const int n_threads = this->settings->execution.n_threads;
		const double cg_tol = this->_forcing_sequence(rhs);
		const int max_iterations = std::max(1000, (int)(this->settings->newton.cg_max_iterations_multiplier * rhs.size())); // Very small sims will need to exceed ndofs iterations
		bsm::BlockedSparseMatrix<3, 3, double>& lhs = *assembled.hess;
		this->du.resize(rhs.size());
		this->du.setZero();
		
		lhs.set_preconditioner(bsm::Preconditioner::BlockDiagonal);
		lhs.prepare_preconditioning(n_threads);
		cg::Info info = cg::solve<double>(this->du.data(), rhs.data(), (int)rhs.size(), cg_tol, max_iterations,
			[&](double* b, const double* x, const int size) { lhs.spmxv_from_ptr(b, x, n_threads);  },
			[&](double* z, const double* r, const int size) { lhs.apply_preconditioning(z, r, n_threads); }
		, n_threads);
		this->cg_iterations_in_step += info.n_iterations;
		this->logger->stop_timing_add("CG");

		if (!info.converged) {
			this->console->add_error_msg(fmt::format("CG didn't converge to tolerance {:.2e} in max iterations {:d}.", cg_tol, max_iterations));
			return false;
		}
		else {
			return true;
		}
	}
	else {
		std::cout << "Stark error: NewtonsMethod::_solve_linear_system() found an unknown linear system solver." << std::endl;
		exit(-1);
	}
}
double stark::core::NewtonsMethod::_inplace_max_step_in_search_direction(const Eigen::VectorXd& du)
{
	// Max step in the search direction (e.g. CCD)
	double step = this->callbacks->run_max_allowed_step();

	// Reduce the step until the configuration is valid (e.g. penetration-free)
	this->global_energy->get_dofs(this->u0.data());
	while (true) {

		if (step < 0.01) {
			//this->console->add_error_msg(fmt::format("Max step in the search direction shorter than {:2e}", step));
			return 0.0;
		}

		this->u1 = this->u0 + step * this->du;
		this->global_energy->set_dofs(this->u1.data());

		this->logger->start_timing("is_intermidiate_state_valid");
		const bool is_valid_state = this->callbacks->run_is_intermidiate_state_valid();
		this->logger->stop_timing_add("is_intermidiate_state_valid");

		if (is_valid_state) {
			break;
		}

		step *= 0.5;
	}
	this->console->print(fmt::format("max step = {:.2e} | ", step), ConsoleVerbosity::NewtonIterations);
	return step;
}
double stark::core::NewtonsMethod::_inplace_backtracking_line_search(const Eigen::VectorXd& du, double E0, double E, double step_valid_configuration, double du_dot_grad)
{
	double step = step_valid_configuration;
	const double suitable_backtracking_energy = E0 + 1e-4 * du_dot_grad;
	this->logger->start_timing("line_search");
	int line_search_it = 1;
	while (E > suitable_backtracking_energy) {
		this->step_line_search_count++;

		// Print
		this->console->print(fmt::format("\n\t\t\t {:d}. step = {:.2e} | E/E_bt = {:.2e}", line_search_it, step, E / suitable_backtracking_energy), ConsoleVerbosity::NewtonIterations);

		// Exit
		if (line_search_it == this->settings->newton.max_line_search_iterations) {
			step = 0.0;
			break;
		}

		// Reduce step
		step *= this->settings->newton.line_search_multiplier;
		this->u1 = this->u0 + step * this->du;
		this->global_energy->set_dofs(this->u1.data());

		// Evaluate
		E = *this->_evaluate_E().E;

		// Counter
		line_search_it++;
	}
	this->logger->stop_timing_add("line_search");


	// Logging debug info
	//// This inspection is to find discontinuities in the energies.
	if (step == 0 && this->settings->debug.line_search_output) {
		const std::string label = std::to_string(this->debug_output_counter);

		this->line_search_debug_logger.append_to_series(label, fmt::format("{:.6e}", E0));
		this->line_search_debug_logger.append_to_series(label, fmt::format("{:.6e}", suitable_backtracking_energy / E0));
		this->line_search_debug_logger.append_to_series(label, fmt::format("{:.6e}", this->du.norm()));
		for (double fstep = -1.0; fstep < 2.0; fstep += 0.01) {
			if (this->debug_output_counter == 0) {
				this->line_search_debug_logger.append_to_series("normalized_step_length", fmt::format("{:.6e}", fstep * step_valid_configuration));
			}

			this->u1 = this->u0 + fstep * step_valid_configuration * this->du;
			this->global_energy->set_dofs(this->u1.data());
			this->callbacks->run_before_energy_evaluation();
			E = *this->global_energy->evaluate_E().E;
			this->callbacks->run_after_energy_evaluation();
			const double v = (E / E0);
			this->line_search_debug_logger.append_to_series(label, fmt::format("{:.6e}", v));
		}

		this->line_search_debug_logger.save_to_disk();
		this->debug_output_counter++;
	}

	return step;
}

