#include "EventDrivenScript.h"

void stark::models::EventDrivenScript::add_event(std::function<bool()> condition, std::function<void()> action, Permanence permanence, std::function<bool()> delete_when)
{
	Event event_;
	event_.condition = condition;
	event_.action = action;
	event_.delete_when = delete_when;
	if (permanence == Permanence::ONE_OFF) {
		event_.discard_at_first_check = true;
	}
	this->events.push_back(event_);
}

void stark::models::EventDrivenScript::add_exclusive_intervals_script(std::function<double()> variable, std::vector<IntervalEvent> events)
{
	// Add all events except the last one (which will run indefinitely)
	for (int i = 0; i < (int)events.size() - 1; i++) {
		const double begin = events[i].begin;
		const double end = events[i + 1].begin;
		const Permanence permanence = events[i].permanence;
		const std::function<void()> action = events[i].action;
		const std::function<bool()> condition = [variable, begin, end]() {
			const double v = variable();
			return begin <= v && v < end;
		};
		const std::function<bool()> delete_when = [variable, end]() {
			return variable() > end;
		};
		this->add_event(condition, action, permanence, delete_when);
	}

	// Last
	const double last_begin = events.back().begin;
	const Permanence permanence = events.back().permanence;
	const std::function<void()> action = events.back().action;
	const std::function<bool()> condition = [variable, last_begin]() {
		return last_begin <= variable();
		};
	this->add_event(condition, action, permanence);
}

void stark::models::EventDrivenScript::run_a_cycle()
{
	for (auto it = this->events.begin(); it != this->events.end();)	{
		if (it->delete_when && it->delete_when()) {
			it = this->events.erase(it);
		}
		else {
			if (it->condition()) {
				it->action();
				if (it->discard_at_first_check) {
					it = this->events.erase(it);
				}
				else {
					++it;
				}
			}
			else {
				++it;
			}
		}
	}
}

void stark::models::EventDrivenScript::clear()
{
	this->events.clear();
}
