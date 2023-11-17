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
	settings.execution.end_simulation_time = 10.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;


	settings.debug.symx_check_for_NaNs = true;

	stark::models::Simulation simulation(settings);

	// Cloth
	const double l = 0.5;
	const int n = 50;
	auto [vertices, triangles] = stark::utils::generate_triangular_grid({ -l, -l }, { l, l }, { n, n });
	auto material = stark::models::Shells::Material::towel();
	//material.area_density = 3.0;
	//material.strain_poisson_ratio = 0.499;
	material.strain_limit = 999999.0;
	auto id = simulation.shells->add(vertices, triangles, material);
	
	// BC
	//bc->add_vertices_in_aabb({ -l, -l, 0.0 }, 0.001);
	//bc->add_vertices_in_aabb({ l, l, 0.0 }, 0.001);

	auto bcl = simulation.shells->create_prescribed_positions_group_with_transformation(id);
	bcl ->add_vertices_in_aabb({ -l, 0.0, 0.0 }, { 0.001, 1.0, 1.0 });
	//bcl->set_linear_velocity(-Eigen::Vector3d::UnitX());
	bcl->set_angular_velocity(-10.0*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());

	auto bcr = simulation.shells->create_prescribed_positions_group_with_transformation(id);
	bcr->add_vertices_in_aabb({ l, 0.0, 0.0 }, { 0.001, 1.0, 1.0 });
	//bcr->set_linear_velocity(Eigen::Vector3d::UnitX());
	bcr->set_angular_velocity(10.0*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());



	// Run
	simulation.stark.run();
}