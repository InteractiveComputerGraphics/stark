#include "Console.h"

#include <iostream>
#include <fstream>


void stark::core::Console::initialize(const std::string path, const ConsoleVerbosity verbosity, const ConsoleOutputTo output_to)
{
	this->verbosity = verbosity;
	this->output_to = output_to;
	this->path = path;
	this->ofstream_ptr = std::make_unique<std::ofstream>(this->path);
	if (!(*this->ofstream_ptr)) {
		std::cout << "stark::core::Console error: Cannot open a file " << this->path << std::endl;
		exit(-1);
	}
}
void stark::core::Console::set_path(const std::string path)
{
	this->path = path;
}
std::string stark::core::Console::get_frame_path() const
{
	return this->path;
}
void stark::core::Console::set_verbosity(const ConsoleVerbosity verbosity)
{
	this->verbosity = verbosity;
}
void stark::core::Console::set_output_target(const ConsoleOutputTo output_to)
{
	this->output_to = output_to;
}
void stark::core::Console::print(const std::string& msg, const ConsoleVerbosity verbosity)
{
	this->_exit_if_no_path();

	std::lock_guard<std::mutex> guard(this->g_pages_mutex);

	// Write to file all verbosity levels
	if (this->output_to == ConsoleOutputTo::FileOnly || this->output_to == ConsoleOutputTo::FileAndConsole) {
		(*this->ofstream_ptr) << msg << std::flush;
	}

	// Write to console only if correct verbosity
	if (this->output_to == ConsoleOutputTo::ConsoleOnly || this->output_to == ConsoleOutputTo::FileAndConsole) {
		if (static_cast<int>(verbosity) <= static_cast<int>(this->verbosity)) {
			std::cout << msg << std::flush;
		}
	}
}

void stark::core::Console::add_error_msg(const std::string& msg)
{
	this->error_msg += msg;
}

void stark::core::Console::print_error_msg_and_clear(const ConsoleVerbosity verbosity)
{
	this->print(this->error_msg, verbosity);
	this->error_msg.clear();
}

void stark::core::Console::_exit_if_no_path()
{
	if (this->path.empty()) {
		std::cout << "stark::core::Console error: no path specified or set." << std::endl;
		exit(-1);
	}
}
