#include <iostream>

#include <stark>
#include <catch2/catch_test_macros.hpp>

#include "../app/paths.h"


stark::Settings test_settings(std::string name)
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = name;
	settings.output.output_directory = OUTPUT_PATH + "/test_output";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.output.enable_output = true;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.gravity = { 0.0, 0.0, -10.0 };
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	return settings;
}


TEST_CASE("global_point_load", "[rb_constraints]") 
{
	stark::Settings settings = test_settings("global_point_load");
	stark::models::Simulation simulation(settings);

	TODO: It does not work with only one constraint

	auto box0 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 });
	//auto constraint = simulation.rigidbodies->add_constraint_global_point(box0, box0.get_translation());
	auto constraint = simulation.rigidbodies->add_constraint_global_point(box0, {1, 1, 1});
	simulation.stark.run(
		[&]()
		{
			//auto [C, f] = constraint.get_violation_in_m_and_force();
			//std::cout << C << " | " << f << std::endl;
		}
	);
}

