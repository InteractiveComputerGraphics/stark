#pragma once
#include <string>
#include <mutex>
#include <fstream>
#include <memory>

namespace stark::core
{
	enum class ConsoleVerbosity { NoOutput = 0, Frames = 1, TimeSteps = 2, NewtonIterations = 3 };
	enum class ConsoleOutputTo { ConsoleOnly, FileOnly, FileAndConsole, NoOutput };

	class Console
	{
	public:

	private:
		std::mutex g_pages_mutex;
		std::unique_ptr<std::ofstream> ofstream_ptr = nullptr;
		ConsoleOutputTo output_to = ConsoleOutputTo::FileAndConsole;
		ConsoleVerbosity verbosity = ConsoleVerbosity::TimeSteps;
		std::string path = "";
		std::string error_msg;

	public:
		void initialize(const std::string path, const ConsoleVerbosity verbosity, const ConsoleOutputTo output_to);

		void set_path(const std::string path);
		std::string get_frame_path() const;
		void set_verbosity(const ConsoleVerbosity verbosity);
		void set_output_target(const ConsoleOutputTo output_to);
		void print(const std::string& msg, const ConsoleVerbosity verbosity);
		void add_error_msg(const std::string& msg);
		void print_error_msg_and_clear(const ConsoleVerbosity verbosity);

	private:
		void _exit_if_no_path();
	};
}
