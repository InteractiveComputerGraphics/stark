#pragma once
#include "../Potential.h"
#include "../../compile/CompiledInLoop.h"
#include "../DeferredParallelTasks.h"
#include "Assembly.h"

namespace symx
{
    class SecondOrderCompiledPotential
    {
	private:
		/* Definitions */
		struct DoFInConn
		{
			int dof_set;  // nth set of degrees of freedom
			int conn_idx;  // nth entry in the local element connectivity
		};

        /* Fields */
        const spMWS<double> mws;
        std::vector<DoFInConn> dof_in_conn;
        CompiledInLoop<double> P;                 // Potential
        CompiledInLoop<double> P__dP_du;          // Potential, Gradient
        CompiledInLoop<double> P__dP_du__d2P_du2; // Potential, Gradient, Hessian
        CompiledInLoop<double> C;                 // Conditional
        std::vector<uint8_t> has_element_positive_condition;
        int32_t n_dofs = -1;
        bool cached = false;

    public:
        /* Methods */
        SecondOrderCompiledPotential(const Potential& potential, const std::vector<DataMap<double>>& dofs, const std::string &compilation_directory, DeferredParallelTasks& tasks);
        bool was_cached() const;
        void evaluate_P(Assembly& assembly);
		void evaluate_P__dP_du(Assembly& assembly);
		void evaluate_P__dP_du__local_d2P_du2(Assembly& assembly);

    private:
        void _evaluate_element_condition(int32_t n_threads);
    };
} // namespace symx
