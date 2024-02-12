#pragma once
#include <functional>
#include <list>
#include <deque>

namespace stark::models
{
	class EventDrivenScript
	{
	private:
		struct Event
		{
			std::function<bool()> run_when;
			std::function<void()> action;
			std::function<bool()> delete_when;
			//bool was_running = false;
			bool discard_at_first_check = false;
		};
		struct Action
		{
			std::function<bool()> run_until;
			std::function<void()> action;
		};

	public:
		enum class Permanence { PERMANENT, ONE_OFF };
		//struct IntervalEventForVariable
		//{
		//	double begin;
		//	Permanence permanence;
		//	std::function<void()> action;
		//};

		void add_independent_event(std::function<bool()> run_when, std::function<void()> action, Permanence permanence, std::function<bool()> delete_when = nullptr);
		void add_recurring_event(std::function<void()> action);
		void append_ordered_action(std::function<void()> action, std::function<bool()> run_until);
		//void add_exclusive_intervals_script(std::function<double()> variable, std::vector<IntervalEventForVariable> events);
		//void add_exclusive_intervals_script_for_variable(std::function<double()> variable, std::vector<IntervalEventForVariable> events);
		void run_a_cycle();
		void clear();

	private:
		std::list<Event> independent_events;
		std::deque<Action> ordered_actions;
	};
}