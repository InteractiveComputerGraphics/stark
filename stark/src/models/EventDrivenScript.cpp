#include "EventDrivenScript.h"

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

void stark::models::EventDrivenScript::append_ordered_action(std::function<void()> action, std::function<bool()> run_until)
{
	Action action_;
	action_.run_until = run_until;
	action_.action = action;
	this->ordered_actions.push_back(action_);
}

//void stark::models::EventDrivenScript::add_exclusive_intervals_script_for_variable(std::function<double()> variable, std::vector<IntervalEventForVariable> events)
//{
//	// Add all events except the last one (which will run indefinitely)
//	for (int i = 0; i < (int)events.size() - 1; i++) {
//		const double begin = events[i].begin;
//		const double end = events[i + 1].begin;
//		const Permanence permanence = events[i].permanence;
//		const std::function<void()> action = events[i].action;
//		const std::function<bool()> condition = [variable, begin, end]() {
//			const double v = variable();
//			return begin <= v && v < end;
//		};
//		const std::function<bool()> delete_when = [variable, end]() {
//			return variable() > end;
//		};
//		this->add_event(condition, action, permanence, delete_when);
//	}
//
//	// Last
//	const double last_begin = events.back().begin;
//	const Permanence permanence = events.back().permanence;
//	const std::function<void()> action = events.back().action;
//	const std::function<bool()> condition = [variable, last_begin]() {
//		return last_begin <= variable();
//		};
//	this->add_event(condition, action, permanence);
//}

void stark::models::EventDrivenScript::run_a_cycle()
{
	// Ordered action
	while (!this->ordered_actions.empty()) {
		auto it = this->ordered_actions.begin();
		const bool finished = it->run_until();
		if (finished) {
			this->ordered_actions.pop_front();
		}
		else {
			it->action();
			break;
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
