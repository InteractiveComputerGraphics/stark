#pragma once
#include <functional>
#include <list>
#include <deque>
#include <vector>
#include <string>


namespace stark::models
{
	// TODO: Cleanup, public/private and document
	struct EventInfo
	{
	private:
		double begin_time = 0.0;
		int id;

	public:
		int n_calls = 0;
		std::string msg = "";
		std::vector<char> bin;
		EventInfo(int id) : id(id) {}
		void activate(double time) { this->begin_time = time; }
		bool first_call() const { return this->n_calls == 0; }
		bool has_data() const { return !this->bin.empty(); }
		double get_begin_time() const { return this->begin_time; }
		int get_id() const { return this->id; }

		// Serialize and store data
		template <typename T>
		void pack(const T& data) {
			this->bin.clear();
			this->bin.resize(sizeof(T));
			std::memcpy(this->bin.data(), &data, sizeof(T));
		}

		// Deserialize and return data
		template <typename T>
		T unpack() const {
			// Check size
			if (this->bin.size() != sizeof(T)) {
				std::cout << "stark error: EventInfo::unpack() size mismatch" << std::endl;
				exit(-1);
			}
			T data;
			std::memcpy(&data, this->bin.data(), sizeof(T));
			return data;
		}
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
			std::function<bool(EventInfo&)> stop_at;
			std::function<void(EventInfo&)> action;
			Action(int id, std::function<void(EventInfo&)> action, std::function<bool(EventInfo&)> stop_at)
				: info(id), action(action), stop_at(stop_at) {}
		};

	public:
		enum class Permanence { PERMANENT, ONE_OFF };

		int add_independent_event(std::function<bool(EventInfo&)> run_when, std::function<void(EventInfo&)> action, Permanence permanence, std::function<bool(EventInfo&)> delete_when = nullptr);
		int add_recurring_event(std::function<void(EventInfo&)> action);
		int make_new_ordered_action_queue();
		int append_ordered_action(int queue_idx, std::function<void(EventInfo&)> action, std::function<bool(EventInfo&)> stop_at);
		void run_a_cycle(double time);
		void clear();

	private:
		std::list<Event> independent_events;
		std::vector<std::deque<Action>> ordered_action_queues;  // Use case: each finger of a hand can be moved independently
		int event_counter = 0;
	};
}