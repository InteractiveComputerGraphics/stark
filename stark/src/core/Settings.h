#pragma once
#include <string>
#include <limits>

#include <Eigen/Dense>

#include "AdaptiveParameter.h"
#include "Console.h"

// User facing enums
namespace stark
{
	enum class ResidualType { Force, Acceleration };
	enum class LinearSystemSolver { CG, DirectLU };
	struct Residual { ResidualType type; double tolerance; };
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
			bool calculate_smooth_normals = false;
			bool enable_output = true;
		};
		struct Simulation
		{
			AdaptiveParameter adaptive_time_step;
			Eigen::Vector3d gravity = { 0.0, 0.0, -9.81 };
		};
		struct Contact
		{
			AdaptiveParameter adaptive_contact_stiffness;
			double dhat = -1.0;  // Can't have a good default value. If left at -1.0, the simulation will give an error
			double friction_stick_slide_threshold = -1.0; // Can't have a good default value. If left at -1.0, the simulation will give an error
			double edge_edge_cross_norm_sq_cutoff = 1e-30;
			double friction_displacement_perturbation = 1e-9;
			bool better_friction_mode = true;
			bool collisions_enabled = true;
			bool friction_enabled = true;
			bool triangle_point_enabled = true;
			bool edge_edge_enabled = true;
			bool enable_intersection_test = true;
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
			bool line_search_output = false;
		};

		Output output;
		Simulation simulation;
		NewtonsMethod newton;
		Contact contact;
		Execution execution;
		Debug debug;

		/* Methods */
		Settings();
		std::string as_string() const;
	};
}