#pragma once
#include <string>
#include <map>
#include <vector>

namespace stark::core
{
	class Logger
	{
	public:
		void start_timing(const std::string& label);
		void stop_timing_series(const std::string& label);
		void stop_timing_add(const std::string& label);
		void add_to_timer(const std::string& label, const double t);
		void add_to_counter(const std::string& label, const int v);
		void save_to_disk(const std::string& path);
		void save_to_disk();
		void set_path(const std::string& path);
		void set(const std::string& label, const double v);
		void set(const std::string& label, const int v);
		void add(const std::string& label, const double v);
		void add(const std::string& label, const int v);
		void append_to_series(const std::string& label, const std::string& v);
		void append_to_series(const std::string& label, const double v);
		void append_to_series(const std::string& label, const int v);

		const std::vector<std::string>& get_series(const std::string& label) const;
		double get_double(const std::string& label) const;
		int get_int(const std::string& label) const;
		std::string get_path() const;

	private:
		std::map<std::string, std::vector<std::string>> series;
		std::map<std::string, double> doubles;
		std::map<std::string, int> ints;
		std::map<std::string, double> t0;
		std::string path;
        std::vector<std::string> empty_series;
	};
}
