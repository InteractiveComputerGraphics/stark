#pragma once
#include <functional>
#include <algorithm>
#include <limits>

#include <Eigen/Dense>

#include "OutputSink.h"

namespace symx
{
    enum class SolverReturn
    {
        Successful,
        Running,
        TooManyNewtonIterations,
        TooManyLineSearchIterations,
        LinearSystemFailure,
        InvalidIntermediateConfiguration,
        LineSearchDoesNotDescend,
        InvalidConvergedState,
    };

    static auto default_residual = [](Eigen::VectorXd& r) { return r.cwiseAbs().maxCoeff(); };
    class SolverCallbacks
    {
	private:
		/* Fields */
		std::vector<std::function<void()>> before_energy_evaluation;
		std::vector<std::function<void()>> after_energy_evaluation;
		std::vector<std::function<bool()>> is_initial_state_valid;
		std::vector<std::function<bool()>> is_intermediate_state_valid;
		std::vector<std::function<void()>> on_intermediate_state_invalid;
		std::vector<std::function<bool()>> is_converged_state_valid;
        std::vector<std::function<double()>> max_allowed_step;
        std::function<double(Eigen::VectorXd&)> residual = default_residual;

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
		// Add callbacks
		void add_before_energy_evaluation(std::function<void()> f) { this->before_energy_evaluation.push_back(f); }
		void add_after_energy_evaluation(std::function<void()> f) { this->after_energy_evaluation.push_back(f); }
		void add_is_initial_state_valid(std::function<bool()> f) { this->is_initial_state_valid.push_back(f); }
		void add_is_intermediate_state_valid(std::function<bool()> f) { this->is_intermediate_state_valid.push_back(f); }
		void add_on_intermediate_state_invalid(std::function<void()> f) { this->on_intermediate_state_invalid.push_back(f); }
		void add_is_converged_state_valid(std::function<bool()> f) { this->is_converged_state_valid.push_back(f); }
        void add_max_allowed_step(std::function<double()> f) { this->max_allowed_step.push_back(f); }

		// Run callbacks
		void run_before_energy_evaluation() { this->_run(this->before_energy_evaluation); }
		void run_after_energy_evaluation() { this->_run(this->after_energy_evaluation); }
		bool run_is_initial_state_valid() { return this->_run_bool(this->is_initial_state_valid); }
		bool run_is_intermediate_state_valid() { return this->_run_bool(this->is_intermediate_state_valid); }
		void run_on_intermediate_state_invalid() { this->_run(this->on_intermediate_state_invalid); }
		bool run_is_converged_state_valid() { return this->_run_bool(this->is_converged_state_valid); }
        double run_max_allowed_step()
        {
			double max_step = std::numeric_limits<double>::infinity();
			for (auto f : this->max_allowed_step) {
				max_step = std::min(max_step, f());
			}
			return max_step;
        }
		
		// Residual computation
		double compute_residual(Eigen::VectorXd& r) { return this->residual(r); }
	};

    enum class LinearSolver
    {
        DirectLU,
        BDPCG,  // Block Diagonal Preconditioned Conjugate Gradient
    };

    enum class ProjectionToPD
    {
        Newton,           // Pure Newton: no projection
        ProjectedNewton,  // Always project all element Hessians to PD before assembly
        ProjectOnDemand,  // Project only when linear system fails or search direction does not descend
        Progressive,      // PPN: Progressively project based on gradient magnitude threshold
    };

    // Backward compatibility alias
    using ProjectToPD = ProjectionToPD;

    struct SolverSettings
    {
        int max_iterations = 100;
        int min_iterations = 1;
        int max_line_search_iterations = 20;
        double step_cap = std::numeric_limits<double>::infinity();
        double residual_tolerance = 1e-6;
        double step_tolerance = std::numeric_limits<double>::epsilon();
        double line_search_armijo_beta = 1e-4;
        bool enable_armijo_bracktracking = true;
        symx::Verbosity verbosity = symx::Verbosity::Step;
        std::string output_prefix = "";
        bool print_line_search_upon_failure = false;
    };

    struct NewtonSettings : public SolverSettings
    {
        // Projection to PD settings
        ProjectionToPD projection_mode = ProjectionToPD::ProjectedNewton;  // Safe default
        double projection_eps = 1e-8;               // Minimum eigenvalue after projection
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

    inline std::string in_two_columns(const std::string& str1, const std::string& str2, size_t distance)
    {
        std::string result = str1;

        if (result.length() < distance) {
            result.append(distance - result.length(), '.');
        }
        else {
            result.replace(distance, result.length() - distance, result.length() - distance, '.');
        }

        return result + str2;
    }
}
