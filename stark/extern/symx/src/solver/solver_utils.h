#pragma once
#include <functional>
#include <algorithm>
#include <limits>

#include <Eigen/Dense>

#include "Context.h"

namespace symx
{
    enum class SolverReturn
    {
        Successful,
        Running,
        InvalidInitialState,
        TooManyIterations,
        TooManyArmijoIterations,
        LinearSystemSolveFailure,
        TooManyInvalidIntermediateIterations,
        StepDoesNotDescend,
        InvalidConvergedState,
    };

    static auto default_residual = [](Eigen::VectorXd& r) { return r.cwiseAbs().maxCoeff(); };
    class SolverCallbacks
    {
	private:
		/* Fields */
		std::vector<std::function<void()>> before_energy_evaluation;
		std::vector<std::function<bool()>> is_initial_state_valid;
		std::vector<std::function<bool()>> is_intermediate_state_valid;
		std::vector<std::function<void()>> on_intermediate_state_invalid;
		std::vector<std::function<void()>> on_armijo_fail;
		std::vector<std::function<bool()>> is_converged_state_valid;
        std::vector<std::function<double()>> max_allowed_step;
        std::function<double(Eigen::VectorXd&)> residual = default_residual;
        spContext context = nullptr;

		/* Methods */
		void _run(std::vector<std::function<void()>>& fs)
		{
			for (auto& f : fs) {
				f();
			}
		}
		bool _run_bool(std::vector<std::function<bool()>>& fs)
		{
			bool valid = true;
			for (auto& f : fs) {
				valid = valid && f();
			}
			return valid;
		}

	public:
		/* Methods */
        SolverCallbacks(spContext context) : context(context) {}
        static std::shared_ptr<SolverCallbacks> create(spContext context) { return std::make_shared<SolverCallbacks>(context); }

		// Add callbacks
		void add_before_energy_evaluation(std::function<void()> f) { this->before_energy_evaluation.push_back(f); }
		void add_is_initial_state_valid(std::function<bool()> f) { this->is_initial_state_valid.push_back(f); }
		void add_is_intermediate_state_valid(std::function<bool()> f) { this->is_intermediate_state_valid.push_back(f); }
		void add_on_intermediate_state_invalid(std::function<void()> f) { this->on_intermediate_state_invalid.push_back(f); }
		void add_on_armijo_fail(std::function<void()> f) { this->on_armijo_fail.push_back(f); }
		void add_is_converged_state_valid(std::function<bool()> f) { this->is_converged_state_valid.push_back(f); }
        void add_max_allowed_step(std::function<double()> f) { this->max_allowed_step.push_back(f); }

		// Run callbacks
		void run_before_energy_evaluation() { 
            auto _t = this->context->logger->time("before_energy_evaluation");
            this->_run(this->before_energy_evaluation); 
        }
		bool run_is_initial_state_valid() { 
            auto _t = this->context->logger->time("is_initial_state_valid");
            return this->_run_bool(this->is_initial_state_valid); 
        }
		bool run_is_intermediate_state_valid() { 
            auto _t = this->context->logger->time("is_intermediate_state_valid");
            return this->_run_bool(this->is_intermediate_state_valid); 
        }
		void run_on_intermediate_state_invalid() { 
            auto _t = this->context->logger->time("on_intermediate_state_invalid");
            this->_run(this->on_intermediate_state_invalid); 
        }
		void run_on_armijo_fail() { 
            auto _t = this->context->logger->time("on_armijo_fail");
            this->_run(this->on_armijo_fail); 
        }
		bool run_is_converged_state_valid() { 
            auto _t = this->context->logger->time("is_converged_state_valid");
            return this->_run_bool(this->is_converged_state_valid); 
        }
        double run_max_allowed_step()
        {
            auto _t = this->context->logger->time("max_allowed_step");
			double max_step = std::numeric_limits<double>::infinity();
			for (auto f : this->max_allowed_step) {
				max_step = std::min(max_step, f());
			}
			return max_step;
        }
		
		// Residual computation
		double compute_residual(Eigen::VectorXd& r) { return this->residual(r); }
	};
    using spSolverCallbacks = std::shared_ptr<SolverCallbacks>;

    enum class LinearSolver
    {
        DirectLU,
        BDPCG,  // Block Diagonal Preconditioned Conjugate Gradient
    };
    inline std::string to_string(symx::LinearSolver v)
    {
        switch (v)
        {
        case symx::LinearSolver::BDPCG: return "BDPCG"; break;
        case symx::LinearSolver::DirectLU: return "DirectLU"; break;
        default:
            std::cout << "symx::LinearSolver " << (int)v << " does not have a name. Exiting." << std::endl;
            exit(-1);
        }
    }

    enum class ProjectionToPD
    {
        Newton,           // Pure Newton: no projection
        ProjectedNewton,  // Always project all element Hessians to PD before assembly
        ProjectOnDemand,  // Project only when linear system fails or search direction does not descend
        Progressive,      // PPN: Progressively project based on gradient magnitude threshold
    };
    inline std::string to_string(symx::ProjectionToPD v)
    {
        switch (v)
        {
        case symx::ProjectionToPD::Newton: return "Newton"; break;
        case symx::ProjectionToPD::ProjectedNewton: return "ProjectedNewton"; break;
        case symx::ProjectionToPD::ProjectOnDemand: return "ProjectOnDemand"; break;
        case symx::ProjectionToPD::Progressive: return "Progressive"; break;
        default:
            std::cout << "symx::ProjectionToPD " << (int)v << " does not have a name. Exiting." << std::endl;
            exit(-1);
        }
    }

    struct SolverSettings
    {
        int max_iterations = 100;
        int min_iterations = 1;
        int max_backtracking_invalid_state_iterations = 8;
        int max_backtracking_armijo_iterations = 20;
        double step_cap = std::numeric_limits<double>::infinity();
        double residual_tolerance = 1e-6;
        double step_tolerance = std::numeric_limits<double>::epsilon();
        double line_search_armijo_beta = 1e-4;
        bool enable_armijo_bracktracking = true;
        bool print_line_search_upon_failure = false;
    };

    struct NewtonSettings : public SolverSettings
    {
        // Projection to PD settings
        ProjectionToPD projection_mode = ProjectionToPD::ProjectedNewton;  // Safe default
        double projection_eps = 1e-10;               // Minimum eigenvalue after projection
        bool project_to_pd_use_mirroring = false;   // Use mirroring instead of clamping for negative eigenvalues
        
        // ProjectOnDemand settings
        int project_on_demand_countdown = 4;        // Number of iterations to project after failure
        
        // Progressive (PPN) settings
        double ppn_tightening_factor = 0.5;         // Factor to tighten threshold when direction does not descend
        double ppn_release_factor = 2.0;            // Factor to relax threshold after successful step
        
        // Linear solver settings
        LinearSolver linear_solver = LinearSolver::BDPCG;
        double cg_max_iterations = 10000;
        double cg_abs_tolerance = 1e-12;
        double cg_rel_tolerance = 1e-4;
        bool cg_stop_on_indefiniteness = true;
        double epsilon_residual = 1e-10;            // Skip step if residual is below this (avoids CG instability)
    };

    inline std::string in_two_columns(const std::string& str1, const std::string& str2, size_t width)
    {
        std::string result = str1;

        if (str1.length() + str2.length() >= width) {
            result.append(".." + str2);
        }
        else {
            size_t n_dots = width - str1.length() - str2.length();
            for (size_t i = 0; i < n_dots; i++) {
                result.append(".");
            }
        }

        return result + str2;
    }
}
