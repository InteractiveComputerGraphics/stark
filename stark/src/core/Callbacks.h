#pragma once
#include <functional>
#include <algorithm>

#include <symx>

namespace stark::core
{
	/*
	* Callbacks will be executed in the order they are pushed.
	* However, it is safer to assume no particular order.
	*/
	struct Callbacks
	{
		std::vector<std::function<void()>> before_simulation;
		std::vector<std::function<symx::DoF(Eigen::VectorXd&)>> initial_guess;
		std::vector<std::function<void()>> before_time_step;
		std::vector<std::function<void()>> after_time_step;
		std::vector<std::function<void()>> before_energy_evaluation;
		std::vector<std::function<void()>> after_energy_evaluation;
		std::vector<std::function<double()>> max_allowed_step;
		std::vector<std::function<bool()>> is_intermidiate_state_valid;
		std::vector<std::function<bool()>> is_converged_state_valid;
		std::vector<std::function<void()>> on_time_step_accepted;
		std::vector<std::function<void()>> write_frame;
		
		void run_before_simulation() const { for (auto f : this->before_simulation) { f(); } };
		void run_before_time_step() const { for (auto f : this->before_time_step) { f(); } };
		void run_after_time_step() const { for (auto f : this->after_time_step) { f(); } };
		void run_before_energy_evaluation() const { for (auto f : this->before_energy_evaluation) { f(); } };
		void run_after_energy_evaluation() const { for (auto f : this->after_energy_evaluation) { f(); } };
		void run_on_time_step_accepted() const { for (auto f : this->on_time_step_accepted) { f(); } };

		void run_write_frame() const { for (auto f : this->write_frame) { f(); } };
		bool run_is_intermidiate_state_valid() const
		{ 
			bool valid = true;
			for (auto f : this->is_intermidiate_state_valid) {
				valid = valid && f();
			}
			return valid;
		};
		bool run_is_converged_state_valid() const
		{ 
			bool valid = true;
			for (auto f : this->is_converged_state_valid) {
				valid = valid && f();
			}
			return valid;
		};
		double run_max_allowed_step() const 
		{ 
			double max_step = 1.0;
			for (auto f : this->max_allowed_step) {
				max_step = std::min(max_step, f());
			}
			return max_step;
		};
	};
}
