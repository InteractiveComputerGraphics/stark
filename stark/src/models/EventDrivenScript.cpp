#include "EventDrivenScript.h"

#include <iostream>

void stark::models::EventDrivenScript::add_independent_event(std::function<bool()> run_when, std::function<void()> action, Permanence permanence, std::function<bool()> delete_when)
{
	Event event_;
	event_.run_when = run_when;
	event_.action = action;
	event_.delete_when = delete_when;
	event_.discard_at_first_check = permanence == Permanence::ONE_OFF;
	this->independent_events.push_back(event_);
}

void stark::models::EventDrivenScript::add_recurring_event(std::function<void()> action)
{
	const std::function<bool()> run_when = []() { return true; };
	this->add_independent_event(run_when, action, Permanence::PERMANENT);
}

int stark::models::EventDrivenScript::make_new_ordered_action_queue()
{
	this->ordered_actions.push_back(std::deque<Action>());
	return (int)this->ordered_actions.size() - 1;
}

void stark::models::EventDrivenScript::append_ordered_action(int queue_idx, std::function<void()> action, std::function<bool()> run_until)
{
	// Error if queue does not exist
	if (queue_idx >= (int)this->ordered_actions.size()) {
		std::cout << "stark error: EventDrivenScript::append_ordered_action() queue_idx " << queue_idx << " does not exist" << std::endl;
		exit(-1);
	}

	Action action_;
	action_.run_until = run_until;
	action_.action = action;
	this->ordered_actions[queue_idx].push_back(action_);
}

void stark::models::EventDrivenScript::run_a_cycle()
{
	// Ordered action
	for (std::deque<Action>& ordered_action_queue : this->ordered_actions) {
		while (!ordered_action_queue.empty()) {
			auto it = ordered_action_queue.begin();
			const bool finished = it->run_until();
			if (finished) {
				ordered_action_queue.pop_front();
			}
			else {
				it->action();
				break;
			}
		}
	}

	// Independent events
	for (auto it = this->independent_events.begin(); it != this->independent_events.end();)	{
		if (it->delete_when && it->delete_when()) {
			it = this->independent_events.erase(it);
		}
		else {
			const bool run = it->run_when();
			if (run) {
				it->action();
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
	this->ordered_actions.clear();
}
