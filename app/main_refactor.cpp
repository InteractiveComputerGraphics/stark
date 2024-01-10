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
void simple_collision()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "simple_collision";
	settings.output.output_directory = OUTPUT_PATH + "/simple_collision";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	//settings.execution.n_threads = 1;
	settings.execution.end_simulation_time = 5.0;


	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = false;
	settings.contact.dhat = 0.005;

	settings.debug.symx_check_for_NaNs = true;

	stark::Simulation simulation(settings);

	// Objects
	stark::RigidBodyHandler box0 = simulation.rigidbodies->add_box(1.0, { 0.5, 0.5, 0.1 });
	stark::RigidBodyHandler box1 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
		.add_displacement({ 0.1, 0, 0.2 });

	// Constraints
	simulation.rigidbodies->add_constraint_fix(box0);

	// Run
	simulation.stark.run();
}
void edge_edge_collision()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "edge_edge_collision";
	settings.output.output_directory = OUTPUT_PATH + "/edge_edge_collision";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	//settings.execution.n_threads = 1;
	settings.execution.end_simulation_time = 1.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.001, 0.001);

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = false;
	settings.contact.dhat = 0.03;
	settings.contact.adaptive_contact_stiffness.set(1e8, 1e8, 1e12);

	settings.debug.symx_check_for_NaNs = true;
	settings.debug.line_search_output = true;

	stark::Simulation simulation(settings);

	// Objects
	stark::RigidBodyHandler box0 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
		.set_rotation(45.0, Eigen::Vector3d::UnitY());
	stark::RigidBodyHandler box1 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
		.add_rotation(45.0, Eigen::Vector3d::UnitY())
		.add_rotation(90.0, Eigen::Vector3d::UnitZ())
		.add_displacement({ 0.01, 0, 0.165 });

	// Constraints
	simulation.rigidbodies->add_constraint_fix(box0);

	// Run
	simulation.stark.run();
}
void heavy_box_rigid_and_deformable()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "deformable";
	settings.output.output_directory = OUTPUT_PATH + "/heavy_box_rigid_and_deformable";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	//settings.simulation.adaptive_time_step.set(0.0, 0.001, 0.001);
	//settings.output.fps = 120.0;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = true;
	settings.contact.dhat = 0.01;

	settings.debug.symx_check_for_NaNs = true;

	stark::Simulation simulation(settings);

	// Add objects
	const double w = 0.3;
	const double w2 = 0.3 * w;
	const double rho = 1e3;
	const double rho2 = 1e4;

	//// Base
	const int n = 5;
	auto [vertices, tets] = stark::utils::generate_tet_grid({ -0.5*w, -0.5*w, -0.5*w }, { 0.5*w, 0.5*w, 0.5*w }, { n, n, n });
	auto material = stark::models::MaterialVolume::soft_rubber();
	material.density = rho;
	material.strain_young_modulus = 1e4;
	material.inertia_damping = 1.0;
	material.strain_damping = 1.0;
	material.strain_limit = 999.0;
	auto base = simulation.deformables->add_volume(vertices, tets, material);
	auto bc = base.create_prescribed_positions_group_with_transformation();
	bc->add_vertices_in_aabb({ 0.0, 0.0, -0.5*w }, { 1.0, 1.0, 0.01 });

	//// Top deformable
	if (false) {
		const int n = 1;
		auto [vertices, tets] = stark::utils::generate_tet_grid({ -0.5*w2, -0.5*w2, -0.5*w2 }, { 0.5*w2, 0.5*w2, 0.5*w2 }, { n, n, n });
		stark::utils::move(vertices, { 0.01, 0.0, 0.5*w + 0.5*w2 + 2.0*settings.contact.dhat });
		auto material = stark::models::MaterialVolume::soft_rubber();
		material.density = rho2;
		material.inertia_damping = 1.0;
		material.strain_damping = 1.0;
		material.strain_limit = 999.0;
		auto top = simulation.deformables->add_volume(vertices, tets, material);

		simulation.interactions->set_friction(base, top, 0.5);
		//simulation.interactions->disable_collision(base, top);
	}

	//// Top rigid
	else {
		auto top = simulation.rigidbodies->add_box(rho2*std::pow(w2, 3), w2)
			.set_translation({ 0.01, 0.0, 0.5*w + 0.5*w2 + 2.0*settings.contact.dhat });

		simulation.interactions->set_friction(base, top, 0.5);
		//simulation.interactions->disable_collision(base, top);
	}

	// Run
	simulation.stark.run();
}
void attachments()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "attachments";
	settings.output.output_directory = OUTPUT_PATH + "/attachments";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.005, 0.005);
	settings.debug.symx_check_for_NaNs = true;
	stark::models::Simulation simulation(settings);

	const double w = 0.1;
	const int n = 5;

	// Base block
	auto [vertices, tets] = stark::utils::generate_tet_grid({ -0.5 * w, -0.5 * w, -0.5 * w }, { 0.5 * w, 0.5 * w, 0.5 * w }, { n, n, n });
	auto material = stark::models::MaterialVolume::soft_rubber();
	auto block0 = simulation.deformables->add_volume(vertices, tets, material);

	// BC
	auto bc = block0.create_prescribed_positions_group_with_transformation();
	bc->set_stiffness(1e6);
	bc->add_vertices_in_aabb({ -0.5*w, 0.0, 0.0 }, { 0.001, 2.0 * w, 2.0 * w });

	// Second block
	stark::utils::move(vertices, { w, 0.0, -w });
	auto block1 = simulation.deformables->add_volume(vertices, tets, material);
	simulation.interactions->attach_by_distance(block0, block1);

	// Cloth
	auto [c_vertices, c_triangles] = stark::utils::generate_triangular_grid({ -0.5 * w, -0.5 * w }, { 0.5 * w, 0.5 * w }, { n, n });
	stark::utils::move(c_vertices, { 2.0*w, 0.0, -0.5*w });
	auto c_material = stark::MaterialSurface::towel();
	auto cloth = simulation.deformables->add_surface(c_vertices, c_triangles, c_material);
	simulation.interactions->attach_by_distance(block1, cloth);

	// Rigid body
	auto rigid = simulation.rigidbodies->add_box(1.0, w)
		.set_translation({3.0*w, 0.0, 0.0});
	simulation.interactions->attach_by_distance(rigid, cloth);

	// Run
	simulation.stark.run();
}


int main()
{
	//rb();
	//net();
	//hanging_cloth();
	//rubber_block();
	//simple_collision();
	//edge_edge_collision();
	//heavy_box_rigid_and_deformable();
	attachments();

	//rb_constraints_all();
}
