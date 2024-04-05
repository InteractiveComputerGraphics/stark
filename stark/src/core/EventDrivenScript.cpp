#include "EventDrivenScript.h"

int stark::EventDrivenScript::add_event(std::function<void(EventInfo&)> action, std::function<bool(EventInfo&)> run_when, std::function<bool(EventInfo&)> delete_when)
{
	Event event_(this->event_counter, run_when, action, delete_when);
	this->events.push_back(event_);
	return this->event_counter++;
}

void stark::EventDrivenScript::run_a_cycle(double time)
{
	for (auto it = this->events.begin(); it != this->events.end();)	{
		
		// Should be deleted?
		if (it->delete_when && it->delete_when(it->info)) {
			it = this->events.erase(it);  // Returns iterator following the last removed element.
		}

		// Check if it should be run
		else {

			// Should be run?
			const bool run = it->run_when(it->info);

			// Run
			if (run) {

				// Activate if first call
				if (it->info.is_first_call()) {
					it->info.activate(time);
				}
				
				// Run
				it->action(it->info);
				it->info.increment_n_calls();
			}

			// Run or not, next.
			++it;
		}
	}
}

void stark::EventDrivenScript::clear()
{
	this->events.clear();
}
