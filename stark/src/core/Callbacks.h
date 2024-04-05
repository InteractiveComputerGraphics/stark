#pragma once
#include <functional>
#include <algorithm>

#include <symx>

namespace stark::core
{
	/*
	* Stark allows the user to inject functionality in the main simulation loop (including the minimizer) by a system of callbacks.
	* Check the documentation to see where each callback is executed.
	*/
	class Callbacks
	{
	private:
		/* Fields */
		// Simulation loop
		std::vector<std::function<void()>> before_simulation;
		std::vector<std::function<void()>> before_time_step;
		std::vector<std::function<void()>> after_time_step;
		std::vector<std::function<void()>> before_energy_evaluation;
		std::vector<std::function<void()>> after_energy_evaluation;
		std::vector<std::function<void()>> on_time_step_accepted;
		std::vector<std::function<void()>> write_frame;

		// Validity
		std::vector<std::function<double()>> max_allowed_step;
		std::vector<std::function<bool()>> is_initial_state_valid;
		std::vector<std::function<bool()>> is_intermidiate_state_valid;
		std::vector<std::function<void()>> on_intermidiate_state_invalid;
		std::vector<std::function<bool()>> is_converged_state_valid;

		// Other
		std::unordered_map<int, std::function<void(double*, double*)>> inv_mass_application;

		/* Methods */
		void _run(std::vector<std::function<void()>>& fs)
		{
			for (auto f : fs) {
				f();
			}
		}
		bool _run_bool(std::vector<std::function<bool()>>& fs)
		{
			bool valid = true;
			for (auto f : fs) {
				valid = valid && f();
			}
			return valid;
		}

	public:
		/* Methods */
		// Add
		void add_before_simulation(std::function<void()> f) { this->before_simulation.push_back(f); };
		void add_before_time_step(std::function<void()> f) { this->before_time_step.push_back(f); };
		void add_after_time_step(std::function<void()> f) { this->after_time_step.push_back(f); };
		void add_before_energy_evaluation(std::function<void()> f) { this->before_energy_evaluation.push_back(f); };
		void add_after_energy_evaluation(std::function<void()> f) { this->after_energy_evaluation.push_back(f); };
		void add_on_time_step_accepted(std::function<void()> f) { this->on_time_step_accepted.push_back(f); };
		void add_write_frame(std::function<void()> f) { this->write_frame.push_back(f); };

		void add_max_allowed_step(std::function<double()> f) { this->max_allowed_step.push_back(f); };
		void add_is_initial_state_valid(std::function<bool()> f) { this->is_initial_state_valid.push_back(f); };
		void add_is_intermidiate_state_valid(std::function<bool()> f) { this->is_intermidiate_state_valid.push_back(f); };
		void add_on_intermidiate_state_invalid(std::function<void()> f) { this->on_intermidiate_state_invalid.push_back(f); };
		void add_is_converged_state_valid(std::function<bool()> f) { this->is_converged_state_valid.push_back(f); };

		void add_inv_mass_application(const symx::DoF& dof, std::function<void(double*, double*)> f) { this->inv_mass_application[dof.idx] = f; };

		// Get
		auto& get_inv_mass_application() { return this->inv_mass_application; };


		// Run
		void run_before_simulation() { this->_run(this->before_simulation); };
		void run_before_time_step() { this->_run(this->before_time_step); };
		void run_after_time_step() { this->_run(this->after_time_step); };
		void run_before_energy_evaluation() { this->_run(this->before_energy_evaluation); };
		void run_after_energy_evaluation() { this->_run(this->after_energy_evaluation); };
		void run_on_time_step_accepted() { this->_run(this->on_time_step_accepted); };
		void run_write_frame() { this->_run(this->write_frame); };

		double run_max_allowed_step()
		{
			double max_step = 1.0;
			for (auto f : this->max_allowed_step) {
				max_step = std::min(max_step, f());
			}
			return max_step;
		};
		bool run_is_initial_state_valid() { return this->_run_bool(this->is_initial_state_valid); };
		bool run_is_intermidiate_state_valid() { return this->_run_bool(this->is_intermidiate_state_valid); };
		void run_on_intermidiate_state_invalid() { this->_run(this->on_intermidiate_state_invalid); };
		bool run_is_converged_state_valid() { return this->_run_bool(this->is_converged_state_valid); };
	};
}
