#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <mutex>

#include <fmt/format.h>


namespace stark
{
	enum class OutputTo { ConsoleOnly, FileOnly, FileAndConsole, NoOutput };
	enum class Verbosity { NoOutput = 0, Frames = 1, TimeSteps = 2, NewtonIterations = 3 };
}

namespace stark::utils
{
	class Console
	{
	public:

	private:
		std::mutex g_pages_mutex;
		std::unique_ptr<std::ofstream> ofstream_ptr = nullptr;
		OutputTo output_to = OutputTo::FileAndConsole;
		Verbosity verbosity = Verbosity::TimeSteps;
		std::string path = "";


	public:
		void initialize(const std::string path, const Verbosity verbosity, const OutputTo output_to);

		void set_path(const std::string path);
		std::string get_frame_path() const;
		void set_verbosity(const Verbosity verbosity);
		void set_output_target(const OutputTo output_to);
		void print(const std::string& msg, const Verbosity verbosity);

	private:
		void _exit_if_no_path();
	};
}
