#pragma once
#include <Eigen/Dense>
#include <memory>

#include "GlobalPotential.h"
#include "second_order/SecondOrderCompiledGlobal.h"
#include "solver_utils.h"
#include <BlockedSparseMatrix/solve_pcg.h>

namespace symx
{
	class NewtonsMethod
	{
	public:
		/* Definitions */
		struct SolveStats
		{
			int newton_iterations = 0;
			int cg_iterations = 0;
			int ls_cap_iterations = 0;
			int ls_max_iterations = 0;
			int ls_inv_iterations = 0;
			int ls_bt_iterations = 0;
			uint64_t n_hessians = 0;
			uint64_t n_projected_hessians = 0;
			double projected_hessians_ratio = 0.0;
		};

    private:
		/* Fields */
		std::shared_ptr<SecondOrderCompiledGlobal> compiled = nullptr;
		spGlobalPotential global_potential = nullptr;
		spContext context = nullptr;
		spOutputSink output = nullptr;
		spLogger logger = nullptr;
		
		// Reusable work vectors
		Eigen::VectorXd du;
		Eigen::VectorXd dofs_ls;
		Eigen::VectorXd dofs_before_ls;
		Eigen::VectorXd rhs;
		Eigen::VectorXd grad;
		Eigen::VectorXd initial_dofs;

		// PCG solver context (reusable buffers)
		bsm::PCGContext pcg_context;

		// Adaptive projection state (persists across Newton iterations)
		std::vector<uint8_t> ppn_active_dof_blocks_for_projection;  // PPN: which blocks to project
		int pdn_countdown = 0;        // ProjectOnDemand: remaining forced-projection iterations
		double ppn_threshold = -1.0;  // PPN: gradient threshold (-1 = not initialized)

		// Stats
        SolveStats stats;
		int last_cg_iterations = 0;
		
		// Line search failure logging
		bool _ls_logging_mode = false;
		std::vector<std::vector<double>> _ls_energy_samples;  // Per Newton iteration: energy samples from -0.5*du to 1.5*du
		
    public:
        /* Fields */
		spSolverCallbacks callbacks;
		NewtonSettings settings;

        /* Methods */
        NewtonsMethod(spGlobalPotential global_potential, spContext context, spSolverCallbacks callbacks = nullptr);
		static std::shared_ptr<NewtonsMethod> create(spGlobalPotential global_potential, spContext context, spSolverCallbacks callbacks = nullptr);
        SolverReturn solve();
		const SolveStats& get_last_solve_stats() const { return this->stats; }
		void print_summary(double total_time = -1.0) const;

    private:
		// Returns true if all elements are projected (can't project more)
		bool _project_and_assemble(spElementHessians element_hessians, ElementHessians::spBSM& hess, int ndofs);
		void _increase_projection(); // Search direction does not descend
		void _decrease_projection(); // Search direction descends

		bool _solve_linear_system(Eigen::VectorXd& du, const ElementHessians::spBSM& hess, const Eigen::VectorXd& grad);
		SolverReturn _line_search_inplace(double E0, double du_dot_grad, double du_max);
	};
	using spNewtonsMethod = std::shared_ptr<NewtonsMethod>;
}
