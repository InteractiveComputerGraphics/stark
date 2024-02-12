#pragma once
#include <functional>
#include <list>
#include <deque>
#include <vector>


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
			bool discard_at_first_check = false;
		};
		struct Action
		{
			std::function<bool()> run_until;
			std::function<void()> action;
		};

	public:
		enum class Permanence { PERMANENT, ONE_OFF };

		void add_independent_event(std::function<bool()> run_when, std::function<void()> action, Permanence permanence, std::function<bool()> delete_when = nullptr);
		void add_recurring_event(std::function<void()> action);
		int make_new_ordered_action_queue();
		void append_ordered_action(int queue_idx, std::function<void()> action, std::function<bool()> run_until);
		void run_a_cycle();
		void clear();

	private:
		std::list<Event> independent_events;
		std::vector<std::deque<Action>> ordered_actions;
	};
}