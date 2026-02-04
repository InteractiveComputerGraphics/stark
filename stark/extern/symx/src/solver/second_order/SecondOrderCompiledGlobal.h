#pragma once
#include "SecondOrderCompiledPotential.h"
#include "../GlobalPotential.h"
#include "../Context.h"
#include "Assembly.h"

namespace symx
{
    // Compiles and evaluates a GlobalPotential for Newton-based optimization.
    // Handles code generation, caching, and parallel evaluation of E, grad, element Hessians.
    class SecondOrderCompiledGlobal
    {
    private:
        /* Fields */
        std::vector<std::unique_ptr<SecondOrderCompiledPotential>> compiled_potentials;
        spGlobalPotential global_potential;
        spContext context;
        Assembly assembly;

    public:
        /* Methods */
        SecondOrderCompiledGlobal(spGlobalPotential global_potential, spContext context);
        
        // Evaluation
        void evaluate_P(double& out_P);
        void evaluate_P__dP_du(double& out_P, Eigen::VectorXd& out_dP_du);
        spElementHessians evaluate_P__dP_du__local_d2P_du2(double& out_P, Eigen::VectorXd& out_dP_du);

        double test_derivatives_with_FD(const double h);
    };
} // namespace symx
