#pragma once
#include <functional>
#include <list>

namespace stark::models
{
	class EventDrivenScript
	{
	public:
		enum class Permanence { PERMANENT, ONE_OFF };
		struct Event
		{
			std::function<bool()> condition;
			std::function<void()> action;
			std::function<bool()> delete_when;
			bool discard_at_first_check = false;
		};
		struct IntervalEvent
		{
			double begin;
			Permanence permanence;
			std::function<void()> action;
		};

		void add_event(std::function<bool()> condition, std::function<void()> action, Permanence permanence, std::function<bool()> delete_when = nullptr);
		void add_exclusive_intervals_script(std::function<double()> variable, std::vector<IntervalEvent> events);
		void run_a_cycle();
		void clear();

	private:
		std::list<Event> events;
	};
}