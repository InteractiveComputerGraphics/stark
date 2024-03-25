#include "EventDrivenScript.h"

#include <iostream>

int stark::EventDrivenScript::add_independent_event(std::function<bool(EventInfo&)> run_when, std::function<void(EventInfo&)> action, Permanence permanence, std::function<bool(EventInfo&)> delete_when)
{
	Event event_(this->event_counter, run_when, action, delete_when, permanence == Permanence::ONE_OFF);
	this->independent_events.push_back(event_);
	return this->event_counter++;
}

int stark::EventDrivenScript::add_recurring_event(std::function<void(EventInfo&)> action)
{
	const std::function<bool(EventInfo&)> run_when = [](EventInfo&) { return true; };
	return this->add_independent_event(run_when, action, Permanence::PERMANENT);
}

void stark::EventDrivenScript::run_a_cycle(double time)
{
	// Independent events
	for (auto it = this->independent_events.begin(); it != this->independent_events.end();)	{
		if (it->delete_when && it->delete_when(it->info)) {
			it = this->independent_events.erase(it);
		}
		else {

			const bool run = it->run_when(it->info);
			if (run) {
				if (it->info.first_call()) {
					it->info.activate(time);
				}
				
				it->action(it->info);
				it->info.n_calls++;
				if (it->discard_at_first_check) {
					it = this->independent_events.erase(it);
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

void stark::EventDrivenScript::clear()
{
	this->independent_events.clear();
}
