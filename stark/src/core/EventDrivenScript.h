#pragma once
#include <functional>
#include <list>
#include <string>
#include <cstring>
#include <iostream>


namespace stark
{
	/*
	*	Holds information about an event, both script logic and custom user data.
	*	It can be used to pass data between the event and the user defined functions.
	*/ 
	struct EventInfo
	{
	public:
		/* Fields */
		std::string msg = "";

		/* Methods */
		EventInfo(int id) : id(id) {}
		void activate(double time) { this->begin_time = time; }
		int get_n_calls() const { return this->n_calls; }
		void increment_n_calls() { this->n_calls++; }
		bool is_first_call() const { return this->n_calls == 0; }
		bool has_data() const { return !this->bin.empty(); }
		double get_begin_time() const { return this->begin_time; }
		int get_id() const { return this->id; }

		/* Packing and unpacking data */
		template<typename T>
		void pack(const T& data) {
			this->bin.clear();
			this->bin.resize(sizeof(T));
			std::memcpy(this->bin.data(), &data, sizeof(T));
		}
		template<typename T>
		void pack(const T* begin, const T* end) {
			this->bin.clear();
			this->bin.resize((end - begin) * sizeof(T));
			std::memcpy(this->bin.data(), begin, (end - begin) * sizeof(T));
		}
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
		template <typename T>
		void unpack(T* begin, T* end) const {
			// Check size
			if (this->bin.size() != (end - begin) * sizeof(T)) {
				std::cout << "stark error: EventInfo::unpack() size mismatch" << std::endl;
				exit(-1);
			}
			std::memcpy(begin, this->bin.data(), (end - begin) * sizeof(T));
		}

	private:
		int id = 0;
		int n_calls = 0;
		double begin_time = 0.0;
		std::vector<char> bin;
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
			Event(int id, std::function<bool(EventInfo&)> run_when, std::function<void(EventInfo&)> action, std::function<bool(EventInfo&)> delete_when) 
				: info(id), run_when(run_when), action(action), delete_when(delete_when) {}
		};

	public:
		int add_event(std::function<void(EventInfo&)> action, std::function<bool(EventInfo&)> run_when = nullptr, std::function<bool(EventInfo&)> delete_when = nullptr);
		void run_a_cycle(double time);
		void clear();

	private:
		std::list<Event> events;
		int event_counter = 0;
	};
}
