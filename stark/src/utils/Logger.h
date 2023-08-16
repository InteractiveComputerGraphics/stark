#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <omp.h>

#include <fmt/format.h>


namespace stark::utils
{
	class Logger
	{
	public:
		void start_timing(const std::string label);
		void stop_timing_series(const std::string label);
		void stop_timing_add(const std::string label);
		void add_to_timer(const std::string label, const double t);
		void add_to_counter(const std::string label, const int v);
		void save_to_disk(const std::string path);
		void save_to_disk();
		void set_path(const std::string path);
		void set(const std::string label, const double v);
		void set(const std::string label, const int v);
		void add(const std::string label, const double v);
		void add(const std::string label, const int v);
		void append_to_series(const std::string label, const std::string v);

		template<typename T>
		void append_to_series(const std::string label, const T v)
		{
			this->series[label].push_back(std::to_string(v));
		};

	//private:
		std::unordered_map<std::string, std::vector<std::string>> series;
		std::unordered_map<std::string, double> doubles;
		std::unordered_map<std::string, int> ints;
		std::unordered_map<std::string, double> t0;
		std::string path;
	};
}
