#pragma once
#include <string>
#include <limits>

#include <Eigen/Dense>

#include "AdaptiveParameter.h"
#include "../utils/Console.h"

namespace stark
{
	struct Settings
	{
		struct Output
		{
			std::string simulation_name = "";
			std::string output_directory = "";
			std::string codegen_directory = "";
			int fps = 30;
			Verbosity console_verbosity = Verbosity::TimeSteps;
			OutputTo console_output_to = OutputTo::FileAndConsole;
			bool suppress_symx_compiler_output = true;
		};
		struct Stark
		{
			AdaptiveParameter adaptive_time_step;
			Eigen::Vector3d gravity = { 0.0, 0.0, -9.81 };
			double boundary_conditions_stiffness = 1e6;
		};
		struct Contact
		{
			AdaptiveParameter adaptive_contact_stiffness;
			double dhat = 1e-3;
			double friction_stick_slide_threshold = 0.1;
			double edge_edge_cross_norm_sq_cutoff = 1e-30;
			bool enable_intersection_test = true;
			bool collisions_enabled = true;
			bool triangle_point_enabled = true;
			bool edge_edge_enabled = true;
		};
		struct NewtonsMethod
		{
			double newton_tol = 1e-8;
			int max_newton_iterations = 10;
			int max_line_search_iterations = 10;
			double line_search_multiplier = 0.5;

			double cg_tol = 1e-12;
			double cg_max_iterations_multiplier = 1.0;

			bool use_direct_linear_solve = false;
			bool project_to_PD = false;
			bool debug_line_search_output = false;
		};
		struct Execution
		{
			double allowed_execution_time = std::numeric_limits<double>::max();
			double end_simulation_time = std::numeric_limits<double>::max();
			int end_frame = std::numeric_limits<int>::max();
			int n_threads = -1;
		};

		Output output;
		Stark simulation;
		NewtonsMethod newton;
		Contact contact;
		Execution execution;

		/* Methods */
		Settings();
		std::string as_string() const;
	};
}