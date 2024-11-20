#pragma once
#include <string>
#include <limits>

#include <Eigen/Dense>
#include <symx>

#include "Console.h"

namespace stark
{
	// User facing enums
	enum class ResidualType { Force, Acceleration };
	enum class LinearSystemSolver { CG, DirectLU };
	struct Residual { ResidualType type; double tolerance; };

	// Wrapper to set SymX compiler
	inline void set_compiler_command(const std::string& str)
	{
		symx::compiler_command = str;
	}
}

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
			ConsoleVerbosity console_verbosity = ConsoleVerbosity::TimeSteps;
			ConsoleOutputTo console_output_to = ConsoleOutputTo::FileAndConsole;
			bool enable_output = true;
		};
		struct Simulation
		{
			Eigen::Vector3d gravity = { 0.0, 0.0, -9.81 };
			bool init_frictional_contact = true;
			double max_time_step_size = 0.01;
			bool use_adaptive_time_step = true;
			double time_step_size_success_muliplier = 1.05;
			double time_step_size_lower_bound = 1e-6;
		};
		struct NewtonsMethod
		{
			Residual residual = { ResidualType::Acceleration, 1.0 };
			LinearSystemSolver linear_system_solver = LinearSystemSolver::CG;
			bool project_to_PD = true;

			int max_newton_iterations = 100;
			int max_line_search_iterations = 10;
			double line_search_multiplier = 0.5;
			double cg_max_iterations_multiplier = 1.0;
			double epsilon_residual = 1e-12;  // Does not apply the correction if the residual is below this value. Avoids numerical instability in CG.
		};
		struct Execution
		{
			double allowed_execution_time = std::numeric_limits<double>::max();
			double end_simulation_time = std::numeric_limits<double>::max();
			int end_frame = std::numeric_limits<int>::max();
			int n_threads = -1;
		};
		struct Debug
		{
			bool symx_check_for_NaNs = false;
			bool symx_suppress_compiler_output = true;
			bool symx_force_compilation = false;
			bool symx_force_load = false;
			bool symx_finite_difference_check = false;
			bool line_search_output = false;
		};

		Output output;
		Simulation simulation;
		NewtonsMethod newton;
		Execution execution;
		Debug debug;

		/* Methods */
		Settings();
		std::string as_string() const;
	};
}