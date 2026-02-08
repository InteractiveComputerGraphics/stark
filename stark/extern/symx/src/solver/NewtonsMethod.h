#pragma once
#include <Eigen/Dense>
#include <memory>

#include "GlobalPotential.h"
#include "second_order/SecondOrderCompiledGlobal.h"
#include "solver_utils.h"
#include "Log.h"
#include <BlockedSparseMatrix/solve_pcg.h>

namespace symx
{
	class NewtonsMethod
	{
    private:
		/* Fields */
		std::shared_ptr<SecondOrderCompiledGlobal> compiled = nullptr;
		spGlobalPotential global_potential = nullptr;
		spContext context = nullptr;
		
		// Reusable work vectors
		Eigen::VectorXd du;
		Eigen::VectorXd step_du;
		Eigen::VectorXd rhs;
		Eigen::VectorXd grad;

		// PCG solver context (reusable buffers)
		bsm::PCGContext pcg_context;

		// Adaptive projection state (persists across Newton iterations)
		std::vector<uint8_t> ppn_active_dof_blocks_for_projection;  // PPN: which blocks to project
		int pdn_countdown = 0;        // ProjectOnDemand: remaining forced-projection iterations
		double ppn_threshold = -1.0;  // PPN: gradient threshold (-1 = not initialized)

        int total_line_search_iterations = 0;

		// Line search failure logging
		bool _ls_logging_mode = false;
		std::vector<std::vector<double>> _ls_energy_samples;  // Per Newton iteration: energy samples from -0.5*du to 1.5*du
		
    public:
        /* Fields */
		SolverCallbacks callbacks;
		NewtonSettings settings;
        Log log;

        /* Methods */
        NewtonsMethod(spGlobalPotential global_potential, spContext context);
		static std::shared_ptr<NewtonsMethod> create(spGlobalPotential global_potential, spContext context);
        SolverReturn solve();
        const Log& get_log() const { return this->log; }

    private:
		// Returns true if all elements are projected (can't project more)
		bool _project_and_assemble(spElementHessians element_hessians, ElementHessians::spBSM& hess, int ndofs);
		void _increase_projection(); // Search direction does not descend
		void _decrease_projection(); // Search direction descends

		bool _solve_linear_system(Eigen::VectorXd& du, const ElementHessians::spBSM& hess, const Eigen::VectorXd& grad);
		SolverReturn _solve_impl();  // Internal solve implementation (called by solve())
		SolverReturn _line_search_inplace(int& armijo_iterations, double E0, double du_dot_grad);
		void _print_return(SolverReturn result, int newton_iterations, int line_search_iterations) const;
		void _print(const std::string& msg, Verbosity verbosity) const;
	};
	using spNewtonsMethod = std::shared_ptr<NewtonsMethod>;
}
