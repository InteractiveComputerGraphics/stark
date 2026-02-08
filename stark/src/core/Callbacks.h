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
	public:
		// Callbacks for the symx's Newton solver
		symx::SolverCallbacks newton; 

		// Stark specific callbacks outside of Newton's Method
	private:
		/* Fields */
		std::vector<std::function<void()>> before_simulation;
		std::vector<std::function<void()>> before_time_step;
		std::vector<std::function<void()>> after_time_step;
		std::vector<std::function<void()>> on_time_step_accepted;
		std::vector<std::function<bool()>> should_continue_execution;
		std::vector<std::function<void()>> write_frame;

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
		void add_on_time_step_accepted(std::function<void()> f) { this->on_time_step_accepted.push_back(f); };
		void add_write_frame(std::function<void()> f) { this->write_frame.push_back(f); };
		void add_should_continue_execution(std::function<bool()> f) { this->should_continue_execution.push_back(f); };

		// Run
		void run_before_simulation() { this->_run(this->before_simulation); };
		void run_before_time_step() { this->_run(this->before_time_step); };
		void run_after_time_step() { this->_run(this->after_time_step); };
		void run_on_time_step_accepted() { this->_run(this->on_time_step_accepted); };
		void run_write_frame() { this->_run(this->write_frame); };
		bool run_should_continue_execution() { return this->_run_bool(this->should_continue_execution); };
	};
}
