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
void laundry_cloth()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "8_cloth_1acc";
	settings.output.output_directory = OUTPUT_PATH + "/laundry_cloth";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;

	settings.execution.end_simulation_time = 0.21;
	settings.simulation.adaptive_time_step.set(0.0, 1.0 / 60.0, 1.0 / 60.0);
	settings.newton.residual = { stark::ResidualType::Acceleration, 1.0 };

	settings.contact.adaptive_contact_stiffness.set(1e4, 1e4, 1e12);
	settings.contact.friction_stick_slide_threshold = 0.01;
	settings.contact.dhat = 0.002;
	stark::models::Simulation simulation(settings);

	// Wall
	auto wall = simulation.rigidbodies->add_box(100.0, { 2.0, 0.5, 2.0 })
		.set_translation({ 0.0, -0.5, 0.0 });
	simulation.rigidbodies->add_constraint_fix(wall);
	wall.add_to_output_label("wall");

	// Drum
	const double target_w = 0.75 * 3.14;
	const double max_torque = 10.0;
	std::vector<Eigen::Vector3d> vertices_drum;
	std::vector<std::array<int, 3>> triangles_drum;
	stark::utils::load_obj(vertices_drum, triangles_drum, MODELS_PATH + "/laundry_drum_2.obj");
	stark::utils::rotate_deg(vertices_drum, -90.0, Eigen::Vector3d::UnitX());
	auto drum = simulation.rigidbodies->add_cylinder(1.0, 0.25, 0.25, vertices_drum, triangles_drum)
		.set_rotation(90.0, Eigen::Vector3d::UnitX());
	simulation.rigidbodies->add_constraint_motor(wall, drum, { 0, 0, 0 }, Eigen::Vector3d::UnitY(), target_w, max_torque, /*delay*/1.0);
	simulation.interactions->disable_collision(wall, drum);
	drum.add_to_output_label("drum");

	// Cloth
	const double friction = 1.0;
	const double scale = 0.34;
	const int n_cloths = 8; //8;
	const double spacing = 0.025;
	const int cloth_resolution = 60;
	std::vector<Eigen::Vector3d> vertices_cloth;
	std::vector<std::array<int, 3>> triangles_cloth;
	stark::utils::generate_triangular_grid(vertices_cloth, triangles_cloth, { -0.5 * scale, -0.5 * scale }, { 0.5 * scale, 0.5 * scale }, { cloth_resolution, cloth_resolution });
	stark::utils::rotate_deg(vertices_cloth, -85.0, Eigen::Vector3d::UnitX());
	stark::utils::move(vertices_cloth, { 0.0, -0.1, 0.0 });

	std::vector<stark::DeformableSurfaceHandler> cloths;
	for (size_t i = 0; i < n_cloths; i++) {
		cloths.push_back(
			simulation.deformables->add_surface(vertices_cloth, triangles_cloth, stark::MaterialSurface::towel())
		);
		stark::utils::move(vertices_cloth, { 0.0, spacing, 0.0 });
	}

	for (int i = 0; i < (int)cloths.size(); i++) {
		simulation.interactions->set_friction(drum, cloths[i], friction);
		for (int j = i + 1; j < (int)cloths.size(); j++) {
			simulation.interactions->set_friction(cloths[i], cloths[j], friction);
		}
	}

	// Run
	simulation.stark.run();
}
void laundry_soft_boxes()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "laundry_soft_boxes-adaptive";
	settings.output.output_directory = OUTPUT_PATH + "/laundry_soft_boxes";
	settings.output.codegen_directory = COMPILE_PATH;
	//settings.output.console_verbosity = stark::ConsoleVerbosity::NewtonIterations;

	settings.execution.end_simulation_time = 5.0;
	settings.simulation.adaptive_time_step.set(0.0, 1.0 / 60.0, 1.0 / 60.0);
	settings.simulation.adaptive_time_step.set(0.0, 1.0 / 60.0, 1.0 / 60.0);
	settings.newton.residual = { stark::ResidualType::Acceleration, 1.0 };

	settings.contact.adaptive_contact_stiffness.set(1e6, 1e6, 1e12);
	settings.contact.friction_stick_slide_threshold = 0.01;// 0.01;
	settings.contact.dhat = 0.002;
	settings.contact.friction_enabled = true;
	stark::models::Simulation simulation(settings);

	// Wall
	auto wall = simulation.rigidbodies->add_box(10.0, { 2.0, 0.5, 2.0 })
		.set_translation({ 0.0, -0.5, 0.0 });
	simulation.rigidbodies->add_constraint_fix(wall);
	wall.add_to_output_label("wall");

	// Drum
	const double target_w = 0.5 * 3.14;
	const double max_torque = 10.0;
	std::vector<Eigen::Vector3d> vertices_drum;
	std::vector<std::array<int, 3>> triangles_drum;
	stark::utils::load_obj(vertices_drum, triangles_drum, MODELS_PATH + "/laundry_drum_2.obj");
	stark::utils::rotate_deg(vertices_drum, -90.0, Eigen::Vector3d::UnitX());
	//auto drum = simulation.rigidbodies->add_cylinder(1.0, 0.25, 0.25, vertices_drum, triangles_drum)
	auto drum = simulation.rigidbodies->add_cylinder(1.0, 0.25, 0.25, 128)
		.set_rotation(90.0, Eigen::Vector3d::UnitX());
	simulation.rigidbodies->add_constraint_motor(wall, drum, { 0, 0, 0 }, Eigen::Vector3d::UnitY(), target_w, max_torque, /*delay*/0.01);
	simulation.interactions->disable_collision(wall, drum);
	drum.add_to_output_label("drum");

	// Cloth
	const int n_elems = 5;
	const double half_box_size = 0.05 / 2.0;

	std::vector<Eigen::Vector3d> vertices_box;
	std::vector<std::array<int, 4>> tets_box;
	stark::utils::generate_tet_grid(vertices_box, tets_box, { -half_box_size, -half_box_size, -half_box_size }, { half_box_size, half_box_size, half_box_size }, { n_elems, n_elems, n_elems });
	stark::utils::move(vertices_box, { -0.133, -0.0869, -0.148 });

	auto material_box = stark::models::MaterialVolume::soft_rubber();
	material_box.strain_young_modulus = 1e4;
	material_box.density *= 2.0;

	const double hx = 0.069;
	const double hy = 0.08;
	const double hz = 0.07;
	std::vector<stark::DeformableVolumeHandler> boxes;
	for (int i = 0; i < 5; i++) { /* 5 */
		for (int j = 0; j < 3; j++) { /* 3 */
			for (int k = 0; k < 2; k++) { /* 4 */
				std::vector<Eigen::Vector3d> vertices = vertices_box;
				stark::utils::move(vertices, { hx * (double)i, hy * (double)j, hz * (double)k });
				boxes.push_back(simulation.deformables->add_volume(vertices, tets_box, material_box));
			}
		}
	}

	// Friction
	simulation.stark.settings.contact.friction_enabled = true;
	const double friction = 2.0;
	for (int i = 0; i < (int)boxes.size(); i++) {
		simulation.interactions->set_friction(drum, boxes[i], friction);
		//for (int j = i + 1; j < (int)boxes.size(); j++) {
		//	simulation.interactions->set_friction(boxes[i], boxes[j], friction);
		//}
	}

	// Run
	simulation.stark.run();
}
void car()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "car_16ms_0.01ra";
	settings.output.output_directory = OUTPUT_PATH + "/car";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = true;
	settings.debug.symx_check_for_NaNs = true;

	// Better energy conservation = higher velocity?
	settings.simulation.adaptive_time_step.set(0.0, 1.0/60.0, 1.0/60.0);
	settings.newton.residual = { stark::ResidualType::Acceleration, 0.01 };
	settings.newton.project_to_PD = true;

	settings.newton.linear_system_solver = stark::LinearSystemSolver::DirectLU;
	settings.contact.dhat = 0.01;
	settings.contact.adaptive_contact_stiffness.set(1e8, 1e8, 1e12);
	stark::Simulation simulation(settings);


	// Car
	stark::VehicleFourWheels car(simulation, stark::VehicleFourWheels::Parametrization::sedan(), "car");

	// Environment
	stark::StaticPlaneHandler ground = simulation.interactions->add_static_plane({ 0.0, 0.0, -0.02 }, Eigen::Vector3d::UnitZ());
	car.set_wheels_friction(simulation, ground, 1.0);

	// Run
	bool braked = false;
	simulation.stark.run(
		[&]()
		{
			car.append_to_logger(simulation);

			const double t = simulation.stark.current_time;
			const double v = car.get_linear_velocity_in_km_per_h();
			if (!braked && t > 2.0) {
				if (t < 3.0) {
					car.set_target_velocity_in_km_per_h(900.0);
				}
				else {
					car.brake();
					braked = true;
					std::cout << "\nBRAKE" << std::endl;
				}
			}
		}
	);
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
	//rb_constraints_all();
	//attachments();
	//laundry_cloth();
	//laundry_soft_boxes();

	car();
}
