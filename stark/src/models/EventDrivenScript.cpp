#include "EventDrivenScript.h"

#include <iostream>

int stark::models::EventDrivenScript::add_independent_event(std::function<bool(EventInfo&)> run_when, std::function<void(EventInfo&)> action, Permanence permanence, std::function<bool(EventInfo&)> delete_when)
{
	Event event_(this->event_counter, run_when, action, delete_when, permanence == Permanence::ONE_OFF);
	this->independent_events.push_back(event_);
	return this->event_counter++;
}

int stark::models::EventDrivenScript::add_recurring_event(std::function<void(EventInfo&)> action)
{
	const std::function<bool(EventInfo&)> run_when = [](EventInfo&) { return true; };
	return this->add_independent_event(run_when, action, Permanence::PERMANENT);
}

int stark::models::EventDrivenScript::make_new_ordered_action_queue()
{
	this->ordered_action_queues.push_back(std::deque<Action>());
	return (int)this->ordered_action_queues.size() - 1;
}

int stark::models::EventDrivenScript::append_ordered_action(int queue_idx, std::function<void(EventInfo&)> action, std::function<bool(EventInfo&)> run_until)
{
	// Error if queue does not exist
	if (queue_idx >= (int)this->ordered_action_queues.size()) {
		std::cout << "stark error: EventDrivenScript::append_ordered_action() queue_idx " << queue_idx << " does not exist" << std::endl;
		exit(-1);
	}

	Action action_(this->event_counter, run_until, action);
	this->ordered_action_queues[queue_idx].push_back(action_);
	return this->event_counter++;
}

void stark::models::EventDrivenScript::run_a_cycle()
{
	// Ordered action queues
	for (std::deque<Action>& ordered_action_queue : this->ordered_action_queues) {
		while (!ordered_action_queue.empty()) {
			auto it = ordered_action_queue.begin();
			const bool finished = it->run_until(it->info);
			if (finished) {
				ordered_action_queue.pop_front();
			}
			else {
				it->action(it->info);
				it->info.n_calls++;
				break;
			}
		}
	}

	// Independent events
	for (auto it = this->independent_events.begin(); it != this->independent_events.end();)	{
		if (it->delete_when && it->delete_when(it->info)) {
			it = this->independent_events.erase(it);
		}
		else {
			const bool run = it->run_when(it->info);
			if (run) {
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

void stark::models::EventDrivenScript::clear()
{
	this->independent_events.clear();
	this->ordered_action_queues.clear();
}
