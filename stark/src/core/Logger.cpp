#include "Logger.h"

#include <iostream>
#include <fstream>
#include <omp.h>

void stark::core::Logger::start_timing(const std::string& label)
{
	this->t0[label] = omp_get_wtime();
}
void stark::core::Logger::stop_timing_series(const std::string& label)
{
	const double t1 = omp_get_wtime();
	auto it = this->t0.find(label);
	if (it == this->t0.end()) {
		std::cout << "Error: Label (" << label << ") not found in Timing.t0." << std::endl;
		exit(-1);
	}
	const double t0 = this->t0[label];
	this->series[label].push_back(std::to_string(t1 - t0));
}
void stark::core::Logger::stop_timing_add(const std::string& label)
{
	const double t1 = omp_get_wtime();
	if (this->t0.find(label) == this->t0.end()) {
		std::cout << "Error: Label (" << label << ") not found in Timing.t0." << std::endl;
		exit(-1);
	}
	if (this->doubles.find(label) == this->doubles.end()) {
		this->doubles[label] = 0.0;
	}
	const double t0 = this->t0[label];
	this->doubles[label] += t1 - t0;
}
void stark::core::Logger::add_to_timer(const std::string& label, const double t)
{
	if (this->doubles.find(label) == this->doubles.end()) {
		this->doubles[label] = 0.0;
	}
	this->doubles[label] += t;
}
void stark::core::Logger::add_to_counter(const std::string& label, const int v)
{
	if (this->ints.find(label) == this->ints.end()) {
		this->ints[label] = 0;
	}
	this->ints[label] += v;
}
void stark::core::Logger::set_path(const std::string& path)
{
	this->path = path;
}
void stark::core::Logger::set(const std::string& label, const double v)
{
	this->doubles[label] = v;
}
void stark::core::Logger::set(const std::string& label, const int v)
{
	this->ints[label] = v;
}
void stark::core::Logger::add(const std::string& label, const double v)
{
	this->doubles[label] += v;
}
void stark::core::Logger::add(const std::string& label, const int v)
{
	this->ints[label] += v;
}
void stark::core::Logger::append_to_series(const std::string& label, const std::string& v)
{
	this->series[label].push_back(v);
}
void stark::core::Logger::append_to_series(const std::string& label, const double v)
{
	this->series[label].push_back(std::to_string(v));
}
void stark::core::Logger::append_to_series(const std::string& label, const int v)
{
	this->series[label].push_back(std::to_string(v));
}
const std::vector<std::string>& stark::core::Logger::get_series(const std::string& label) const
{
	auto it = this->series.find(label);
	if (it == this->series.end()) {
		return this->empty_series;
	}
	return it->second;
}
double stark::core::Logger::get_double(const std::string& label) const
{
	auto it = this->doubles.find(label);
	if (it == this->doubles.end()) {
		return 0.0;
	}
	return it->second;
}
int stark::core::Logger::get_int(const std::string& label) const
{
	auto it = this->ints.find(label);
	if (it == this->ints.end()) {
		return 0;
	}
	return it->second;
}
std::string stark::core::Logger::get_path() const
{
	return this->path;
}
void stark::core::Logger::save_to_disk(const std::string& path)
{
	std::ofstream outfile(path);
	if (!outfile) {
		std::cout << "stark::core::Logger::save_to_disk error: Cannot open file " << path << std::endl;
		exit(-1);
	}

	for (auto& pair : this->series) {
		outfile << pair.first << ": ";
		for (std::string& v : pair.second) {
			outfile << v << ", ";
		}
		outfile << std::endl;
	}

	for (auto& pair : this->doubles) {
		outfile << pair.first << ": " << pair.second << std::endl;
	}

	for (auto& pair : this->ints) {
		outfile << pair.first << ": " << pair.second << std::endl;
	}

	outfile.close();
}
void stark::core::Logger::save_to_disk()
{
	if (this->path.empty()) {
		std::cout << "stark::core::Logger error: no path specified or set." << std::endl;
		exit(-1);
	}
	this->save_to_disk(this->path);
}
