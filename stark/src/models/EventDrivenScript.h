#pragma once
#include <functional>
#include <list>
#include <deque>
#include <vector>
#include <string>


namespace stark::models
{
	struct EventInfo
	{
		const int id;
		int n_calls = 0;
		std::string str = "";
		EventInfo(int id) : id(id) {}
	};

	class EventDrivenScript
	{
	private:
		struct Event
		{
			EventInfo info;
			std::function<bool(EventInfo&)> run_when;
			std::function<void(EventInfo&)> action;
			std::function<bool(EventInfo&)> delete_when;
			bool discard_at_first_check = false;
			Event(int id, std::function<bool(EventInfo&)> run_when, std::function<void(EventInfo&)> action, std::function<bool(EventInfo&)> delete_when, bool discard_at_first_check) 
				: info(id), run_when(run_when), action(action), delete_when(delete_when), discard_at_first_check(discard_at_first_check) {}
		};
		struct Action
		{
			EventInfo info;
			std::function<bool(EventInfo&)> run_until;
			std::function<void(EventInfo&)> action;
			Action(int id, std::function<bool(EventInfo&)> run_until, std::function<void(EventInfo&)> action) 
				: info(id), run_until(run_until), action(action) {}
		};

	public:
		enum class Permanence { PERMANENT, ONE_OFF };

		int add_independent_event(std::function<bool(EventInfo&)> run_when, std::function<void(EventInfo&)> action, Permanence permanence, std::function<bool(EventInfo&)> delete_when = nullptr);
		int add_recurring_event(std::function<void(EventInfo&)> action);
		int make_new_ordered_action_queue();
		int append_ordered_action(int queue_idx, std::function<void(EventInfo&)> action, std::function<bool(EventInfo&)> run_until);
		void run_a_cycle();
		void clear();

	private:
		std::list<Event> independent_events;
		std::vector<std::deque<Action>> ordered_action_queues;  // Use case: each finger of a hand can be moved independently
		int event_counter = 0;
	};
}