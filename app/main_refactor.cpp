#include <iostream>

#include <stark>

#include "paths.h"


int main()
{
	std::cout << "hola" << std::endl;

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_cloth";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_cloth";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;


	settings.debug.symx_check_for_NaNs = true;

	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 10;
	auto [vertices, triangles] = stark::utils::generate_triangular_grid({ -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	auto id = simulation.shells->add(vertices, triangles, stark::models::Shells::Material::towel());
	
	// BC
	auto bc = simulation.shells->create_prescribed_positions_group_with_transformation(id);
	bc->add_vertex(0);
	bc->add_vertex(n);

	// Run
	simulation.stark.run();
}