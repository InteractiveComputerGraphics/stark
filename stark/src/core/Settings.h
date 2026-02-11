#pragma once
#include <string>
#include <limits>

#include <Eigen/Dense>
#include <symx>

namespace stark::core
{
	struct Settings
	{
		struct Output
		{
			std::string simulation_name = "";
			std::string output_directory = "";
			std::string codegen_directory = "";
			std::string time_stamp = "SET_TO_CREATION_TIME_BY_DEFAULT";
			int fps = 30;
			symx::Verbosity verbosity = symx::Verbosity::Medium;
			symx::OutputTo output_to = symx::OutputTo::PrintAndFile;
			bool enable_output = true;
		};
		struct Simulation
		{
			Eigen::Vector3d gravity = { 0.0, 0.0, -9.81 };
			bool init_frictional_contact = true;
			double max_time_step_size = 1.0/120.0;
			bool use_adaptive_time_step = true;
			double time_step_size_success_multiplier = 1.05;
			double time_step_size_lower_bound = 1e-6;
		};
		// struct symx::NewtonSettings {};
		struct Execution
		{
			double allowed_execution_time = std::numeric_limits<double>::max();
			double end_simulation_time = std::numeric_limits<double>::max();
			int end_frame = std::numeric_limits<int>::max();
			int n_threads = -1;
		};

		Output output;
		Simulation simulation;
		symx::NewtonSettings newton;
		Execution execution;

		/* Methods */
		Settings();
		std::string as_string() const;
	};
}
