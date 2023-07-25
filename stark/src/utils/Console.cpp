#include "Console.h"

void stark::utils::Console::initialize(const std::string path, const Verbosity verbosity, const OutputTo output_to)
{
	this->verbosity = verbosity;
	this->output_to = output_to;
	this->path = path;
	this->ofstream_ptr = std::make_unique<std::ofstream>(path);
	if (!(*this->ofstream_ptr)) {
		std::cout << "stark::utils::Console error: Cannot open a file " << path << std::endl;
		exit(-1);
	}
}

void stark::utils::Console::set_path(const std::string path)
{
}
std::string stark::utils::Console::get_path() const
{
	return this->path;
}
void stark::utils::Console::set_verbosity(const Verbosity verbosity)
{
}
void stark::utils::Console::set_output_target(const OutputTo output_to)
{
}
void stark::utils::Console::print(const std::string str)
{
	this->_exit_if_no_path();

	std::lock_guard<std::mutex> guard(this->g_pages_mutex);

	if (this->output_to == OutputTo::FileOnly || this->output_to == OutputTo::FileAndConsole) {
		(*this->ofstream_ptr) << str << std::flush;
	}
	else if (this->output_to == OutputTo::ConsoleOnly || this->output_to == OutputTo::FileAndConsole) {
		std::cout << str << std::flush;
	}
}
void stark::utils::Console::print(const std::string& msg, const Verbosity verbosity)
{
	if (static_cast<int>(verbosity) <= static_cast<int>(this->verbosity)) {
		this->print(msg);
	}
}

void stark::utils::Console::_exit_if_no_path()
{
	if (this->path.empty()) {
		std::cout << "stark::utils::Console error: no path specified or set." << std::endl;
		exit(-1);
	}
}
