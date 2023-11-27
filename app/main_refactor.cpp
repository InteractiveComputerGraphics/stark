#include <iostream>

#include <stark>

#include "paths.h"


void rb()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "rb";
	settings.output.output_directory = OUTPUT_PATH + "/rb";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;

	settings.debug.symx_check_for_NaNs = true;
	//settings.newton.project_to_PD = true;
	//settings.newton.use_direct_linear_solve = true;
	//settings.newton.max_line_search_iterations = 1000;

	stark::models::Simulation simulation(settings);

	// Objects
	auto box0 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
		.set_linear_damping(1.0)
		.enable_writing_transformation_sequence("box0");
	auto box1 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
		.set_linear_damping(1.0)
		.add_displacement({ 0.1, 0, 0 })
		.enable_writing_transformation_sequence("box1");

	// Constraints
	simulation.rigidbodies->add_constraint_anchor_point(box0, { -0.05, -0.05, -0.05 });
	simulation.rigidbodies->add_constraint_ball_joint(box0, box1, { 0.05, 0.05, 0.05 });

	// Misc
	simulation.rigidbodies->write_collision_meshes(true);

	// Run
	simulation.stark.run();
}
void net()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "net";
	settings.output.output_directory = OUTPUT_PATH + "/net";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;

	settings.debug.symx_check_for_NaNs = true;
	//settings.newton.project_to_PD = true;
	//settings.newton.use_direct_linear_solve = true;
	//settings.newton.max_line_search_iterations = 1000;

	stark::models::Simulation simulation(settings);

	// Cloth
	const double l = 0.5;
	const int n = 10;
	auto [vertices, triangles] = stark::utils::generate_triangular_grid({ -l, -l }, { l, l }, { n, n });
	auto edges = stark::utils::find_edges_from_simplices(triangles, (int)vertices.size());
	auto material = stark::models::OneDimensionalMaterial::sticky_goo();
	material.strain_damping = 0.1;
	auto id = simulation.lines->add(vertices, edges, material);

	// BC
	if (true) {
		auto bc = simulation.surfaces->create_prescribed_positions_group_with_transformation(id);
		bc->add_vertices_in_aabb({ -l, -l, 0.0 }, 0.001);
		bc->add_vertices_in_aabb({ l, l, 0.0 }, 0.001);
	}
	else {
		auto bcl = simulation.surfaces->create_prescribed_positions_group_with_transformation(id);
		bcl->add_vertices_in_aabb({ -l, 0.0, 0.0 }, { 0.001, 1.0, 1.0 });
		bcl->set_linear_velocity(-Eigen::Vector3d::UnitX());
		//bcl->set_angular_velocity(-10.0*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());

		auto bcr = simulation.surfaces->create_prescribed_positions_group_with_transformation(id);
		bcr->add_vertices_in_aabb({ l, 0.0, 0.0 }, { 0.001, 1.0, 1.0 });
		bcr->set_linear_velocity(Eigen::Vector3d::UnitX());
		//bcr->set_angular_velocity(10.0*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
	}

	// Run
	simulation.stark.run();
}
void hanging_cloth()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_cloth";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_cloth";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;

	settings.debug.symx_check_for_NaNs = true;
	//settings.newton.project_to_PD = true;
	//settings.newton.use_direct_linear_solve = true;
	//settings.newton.max_line_search_iterations = 1000;

	stark::models::Simulation simulation(settings);

	// Cloth
	const double l = 0.5;
	const int n = 50;
	auto [vertices, triangles] = stark::utils::generate_triangular_grid({ -l, -l }, { l, l }, { n, n });
	auto material = stark::models::SurfaceMaterial::towel();
	//material.area_density = 3.0;
	//material.strain_limit_stiffness = 1e7;
	//material.strain_limit = 99999.9;
	//material.strain_young_modulus = 1e3;
	auto id = simulation.surfaces->add(vertices, triangles, material);

	// BC
	if (true) {
		auto bc = simulation.surfaces->create_prescribed_positions_group_with_transformation(id);
		bc->add_vertices_in_aabb({ -l, -l, 0.0 }, 0.001);
		bc->add_vertices_in_aabb({ -l, l, 0.0 }, 0.001);
	}
	else {
		auto bcl = simulation.surfaces->create_prescribed_positions_group_with_transformation(id);
		bcl->add_vertices_in_aabb({ -l, 0.0, 0.0 }, { 0.001, 1.0, 1.0 });
		bcl->set_linear_velocity(-Eigen::Vector3d::UnitX());
		//bcl->set_angular_velocity(-10.0*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());

		auto bcr = simulation.surfaces->create_prescribed_positions_group_with_transformation(id);
		bcr->add_vertices_in_aabb({ l, 0.0, 0.0 }, { 0.001, 1.0, 1.0 });
		bcr->set_linear_velocity(Eigen::Vector3d::UnitX());
		//bcr->set_angular_velocity(10.0*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
	}

	// Run
	simulation.stark.run();
}
void rubber_block()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "rubber_block";
	settings.output.output_directory = OUTPUT_PATH + "/rubber_block";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.02, 0.02);
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;

	settings.debug.symx_check_for_NaNs = true;
	//settings.newton.project_to_PD = true;
	//settings.newton.use_direct_linear_solve = true;
	//settings.newton.max_line_search_iterations = 1000;

	stark::models::Simulation simulation(settings);

	// Declare objects
	const double w = 0.1;
	const double l = 0.1*w;
	const int nw = 30;
	const int nl = nw; nw* (l / w);
	auto [vertices, tets] = stark::utils::generate_tet_grid({ -l, -w, -w }, { l, w, w }, { nl, nw, nw });
	auto material = stark::models::VolumeMaterial::soft_rubber();
	//material.area_density = 3.0;
	//material.strain_limit_stiffness = 1e7;
	//material.strain_limit = 99999.9;
	//material.strain_young_modulus = 1e3;
	material.strain_poisson_ratio = 0.4;
	auto id = simulation.volumes->add(vertices, tets, material);

	// BC
	if (false) {
		auto bc = simulation.surfaces->create_prescribed_positions_group_with_transformation(id);
		bc->set_stiffness(1e6);
		bc->add_vertices_in_aabb({ -l, 0.0, 0.0 }, { 0.001, 2.0*w, 2.0*w });
	}
	else {
		auto bcl = simulation.surfaces->create_prescribed_positions_group_with_transformation(id);
		bcl->set_stiffness(1e6);
		bcl->add_vertices_in_aabb({ -l, 0.0, 0.0 }, { 0.001, 2.0*w, 2.0*w });
		bcl->set_linear_velocity(-0.05*Eigen::Vector3d::UnitX());
		//bcl->set_angular_velocity(-10.0*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());

		auto bcr = simulation.surfaces->create_prescribed_positions_group_with_transformation(id);
		bcr->set_stiffness(1e6);
		bcr->add_vertices_in_aabb({ l, 0.0, 0.0 }, { 0.001, 2.0*w, 2.0*w });
		bcr->set_linear_velocity(0.05*Eigen::Vector3d::UnitX());
		//bcr->set_angular_velocity(10.0*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
	}

	// Run
	simulation.stark.run();
}




int main()
{
	//hanging_cloth();
	//rubber_block();
	//net();
	rb();
}
