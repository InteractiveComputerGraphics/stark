#include "Logger.h"

void stark::utils::Logger::start_timing(const std::string label)
{
	this->t0[label] = omp_get_wtime();
}
void stark::utils::Logger::stop_timing_series(const std::string label)
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
void stark::utils::Logger::stop_timing_add(const std::string label)
{
	const double t1 = omp_get_wtime();
	if (this->t0.find(label) == this->t0.end()) {
		std::cout << "Error: Label (" << label << ") not found in Timing.t0." << std::endl;
		exit(-1);
	}
	if (this->timers.find(label) == this->timers.end()) {
		this->timers[label] = 0.0;
	}
	const double t0 = this->t0[label];
	this->timers[label] += t1 - t0;
}
void stark::utils::Logger::add_to_timer(const std::string label, const double t)
{
	if (this->timers.find(label) == this->timers.end()) {
		this->timers[label] = 0.0;
	}
	this->timers[label] += t;
}
void stark::utils::Logger::set_path(const std::string path)
{
	this->path = path;
}
void stark::utils::Logger::save_to_disk(const std::string path)
{
	// Open the file
	std::ofstream outfile(path);
	if (!outfile) {
		std::cout << "stark::utils::Logger error: Cannot open a file " << path << std::endl;
		exit(-1);
	}

	for (auto& pair : this->series) {
		outfile << pair.first << ": ";
		for (std::string& v : pair.second) {
			outfile << v << ", ";
		}
		outfile << std::endl;
	}

	for (auto& pair : this->timers) {
		outfile << pair.first << ": " << pair.second << std::endl;
	}
	outfile.close();
}
void stark::utils::Logger::save_to_disk()
{
	if (this->path.empty()) {
		std::cout << "stark::utils::Logger error: no path specified or set." << std::endl;
		exit(-1);
	}
	this->save_to_disk(this->path);
}
