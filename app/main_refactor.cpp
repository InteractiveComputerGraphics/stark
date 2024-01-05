#include <iostream>

#include <stark>

#include "paths.h"
#include "rb_constraint_test_scenes.h"


void rb()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "rb";
	settings.output.output_directory = OUTPUT_PATH + "/rb";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.debug.symx_check_for_NaNs = true;

	stark::Simulation simulation(settings);

	// Objects
	stark::RigidBodyHandler box0 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
		.set_linear_damping(1.0)
		.enable_writing_transformation_sequence("box0");
	stark::RigidBodyHandler box1 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
		.set_linear_damping(1.0)
		.add_displacement({ 0.1, 0, 0 })
		.enable_writing_transformation_sequence("box1");

	// Constraints
	simulation.rigidbodies->add_constraint_global_point(box0, { -0.05, -0.05, -0.05 });
	simulation.rigidbodies->add_constraint_point(box0, box1, { 0.05, 0.05, 0.05 });

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
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.debug.symx_check_for_NaNs = true;
	stark::models::Simulation simulation(settings);

	// Net
	const double l = 0.5;
	const int n = 10;
	auto [vertices, triangles] = stark::utils::generate_triangular_grid({ -l, -l }, { l, l }, { n, n });
	auto edges = stark::utils::find_edges_from_simplices(triangles, (int)vertices.size());
	auto material = stark::models::MaterialLine::sticky_goo();
	material.strain_damping = 0.1;
	auto net = simulation.deformables->add_line(vertices, edges, material);

	// BC
	if (true) {
		auto bc = net.create_prescribed_positions_group_with_transformation();
		bc->add_vertices_in_aabb({ -l, -l, 0.0 }, 0.001);
		bc->add_vertices_in_aabb({ l, l, 0.0 }, 0.001);
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
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.01, 0.01);
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.debug.symx_check_for_NaNs = true;
	stark::models::Simulation simulation(settings);

	// Cloth
	const double l = 0.5;
	const int n = 50;
	auto [vertices, triangles] = stark::utils::generate_triangular_grid({ -l, -l }, { l, l }, { n, n });
	auto material = stark::MaterialSurface::towel();
	auto towel = simulation.deformables->add_surface(vertices, triangles, material);

	// BC
	auto bc = towel.create_prescribed_positions_group_with_transformation();
	bc->add_vertices_in_aabb({ -l, -l, 0.0 }, 0.001);
	bc->add_vertices_in_aabb({ -l, l, 0.0 }, 0.001);

	// Run
	simulation.stark.run();
}
void rubber_block()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "rubber_block";
	settings.output.output_directory = OUTPUT_PATH + "/rubber_block";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.005, 0.005);
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.debug.symx_check_for_NaNs = true;
	stark::models::Simulation simulation(settings);

	// Declare objects
	const double w = 0.1;
	const double l = 2.0*w;
	const int nw = 10;
	const int nl = nw * (l / w);
	auto [vertices, tets] = stark::utils::generate_tet_grid({ -l, -w, -w }, { l, w, w }, { nl, nw, nw });
	auto material = stark::models::MaterialVolume::soft_rubber();
	auto block = simulation.deformables->add_volume(vertices, tets, material);

	// BC
	auto bc = block.create_prescribed_positions_group_with_transformation();
	bc->set_stiffness(1e6);
	bc->add_vertices_in_aabb({ -l, 0.0, 0.0 }, { 0.001, 2.0*w, 2.0*w });

	// Run
	simulation.stark.run();
}


int main()
{
	//rb();
	//net();
	//hanging_cloth();
	rubber_block();

	//rb_constraints_all();
}
