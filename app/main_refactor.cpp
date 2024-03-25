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
	simulation.run();
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
	stark::Simulation simulation(settings);

	// Net
	const double l = 0.5;
	const int n = 10;
	auto [vertices, triangles] = stark::utils::generate_triangular_grid({ -l, -l }, { l, l }, { n, n });
	auto edges = stark::utils::find_edges_from_simplices(triangles, (int)vertices.size());
	auto params = stark::DeformableLineParametrization::sticky_goo(stark::ElasticityOnly::False);
	params.strain_damping = 0.1;
	auto net = simulation.deformables->add_line(vertices, edges, params);

	// BC
	if (true) {
		auto bc = net.create_prescribed_positions_group_with_transformation();
		bc->add_vertices_in_aabb({ -l, -l, 0.0 }, 0.001);
		bc->add_vertices_in_aabb({ l, l, 0.0 }, 0.001);
	}

	// Run
	simulation.run();
}
void hanging_cloth()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_cloth";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_cloth";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.debug.symx_check_for_NaNs = true;

	settings.debug.symx_finite_difference_check = false;
	settings.newton.symx_cse_strategy = symx::CSE::Safe;
	stark::Simulation simulation(settings);

	// Cloth
	const double l = 0.5;
	const int n = 50;
	auto [vertices, triangles] = stark::utils::generate_triangular_grid({ -l, -l }, { l, l }, { n, n });
	auto params = stark::DeformableSurfaceParametrization::towel(stark::ElasticityOnly::False);
	auto towel = simulation.deformables->add_surface(vertices, triangles, params);

	// BC
	auto bc = towel.create_prescribed_positions_group_with_transformation();
	bc->add_vertices_in_aabb({ -l, -l, 0.0 }, 0.001);
	bc->add_vertices_in_aabb({ -l, l, 0.0 }, 0.001);

	// Run
	simulation.run();
}
void hanging_cloth_scaled()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_cloth_scaled";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_cloth_scaled";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.debug.symx_check_for_NaNs = true;
	stark::Simulation simulation(settings);

	// cloth
	auto params = stark::DeformableSurfaceParametrization::towel(stark::ElasticityOnly::False);
	params.inertia_damping = 1.0;
	params.strain_limit = 9999999.9;
	auto cloth = simulation.deformables->add_surface_grid({ 0.0, 0.0 }, 0.2, 20, params);

	// BC
	auto bc = cloth.create_prescribed_positions_group_with_transformation();
	bc->add_vertices_in_aabb({ 0.1, 0.1, 0.0 }, { 0.01, 0.01, 0.01 });
	bc->add_vertices_in_aabb({ 0.1, -0.1, 0.0 }, { 0.01, 0.01, 0.01 });

	// Run
	simulation.run(
		[&]()
		{
			const double t = simulation.get_time() / 5.0;
			cloth.set_scale(1.0 - 0.8 * t);
		}
	);
}
void hanging_box_scaled()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_box_scaled";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_box_scaled";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.debug.symx_check_for_NaNs = true;
	stark::Simulation simulation(settings);

	// Box
	auto params = stark::DeformableVolumeParametrization::soft_rubber(stark::ElasticityOnly::False);
	params.inertia_damping = 1.0;
	params.strain_limit = 9999999.9;
	auto box = simulation.deformables->add_volume_grid({ 0.0, 0.0, 0.0 }, 0.2, 7, params);

	// BC
	auto bc = box.create_prescribed_positions_group_with_transformation();
	bc->add_vertices_in_aabb({ 0.1, 0.1, 0.1 }, { 0.01, 0.01, 0.01 });
	bc->add_vertices_in_aabb({ 0.1, -0.1, 0.1 }, { 0.01, 0.01, 0.01 });

	// Run
	simulation.run(
		[&]()
		{
			const double t = simulation.get_time() / 5.0;
			box.set_scale(1.0 - 0.8 * t);
		}
	);
}
void hanging_cloth_and_reinforcement()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_cloth_and_reinforcement";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_cloth_and_reinforcement";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.contact.dhat = 0.002;

	settings.newton.project_to_PD = false;
	settings.debug.symx_check_for_NaNs = true;
	stark::Simulation simulation(settings);

	// Only cloth
	{
		auto params = stark::DeformableSurfaceParametrization::towel(stark::ElasticityOnly::False);
		params.inertia_damping = 1.0;
		params.strain_limit = 9999999.9;
		params.strain_young_modulus = 1e2;
		auto cloth = simulation.deformables->add_surface_grid({ 0.0, 0.0 }, 0.2, 20, params);

		// BC
		auto bc = cloth.create_prescribed_positions_group_with_transformation();
		bc->add_vertices_in_aabb({ 0.1, 0.1, 0.0 }, { 0.01, 0.01, 0.01 });
		bc->add_vertices_in_aabb({ 0.1, -0.1, 0.0 }, { 0.01, 0.01, 0.01 });
	}

	// Cloth + Reinforcement
	auto params = stark::DeformableSurfaceParametrization::towel(stark::ElasticityOnly::False);
	params.inertia_damping = 1.0;
	params.strain_limit = 9999999.9;
	params.strain_young_modulus = 1e2;

	params.use_reinforced_perimeter = true;
	params.reinforced_perimeter = stark::DeformableLineParametrization::sticky_goo(stark::ElasticityOnly::False);
	params.reinforced_perimeter.strain_limit = 9999999.9;
	params.reinforced_perimeter.strain_damping = 0.0;
	auto cloth = simulation.deformables->add_surface_grid({ 0.0, 0.0 }, 0.2, 20, params)
		.add_displacement({ 0.0, 0.3, 0.0 });

	// BC
	auto bc = cloth.create_prescribed_positions_group_with_transformation();
	bc->add_vertices_in_aabb({ 0.1, 0.4, 0.0 }, { 0.01, 0.01, 0.01 });
	bc->add_vertices_in_aabb({ 0.1, 0.2, 0.0 }, { 0.01, 0.01, 0.01 });

	// Run
	simulation.run(
		[&]()
		{
			const double t = simulation.get_time() / 5.0;
			auto reinforcement = cloth.get_reinforcement();
			reinforcement.set_young_modulus(1e5 * t);  // Increases skin bending stiffness over time
			reinforcement.set_scale(1.0 - 0.8*t);
		}
	);
}
void rubber_block_and_skin()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "rubber_block_and_skin";
	settings.output.output_directory = OUTPUT_PATH + "/rubber_block_and_skin";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.005, 0.005);
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.newton.project_to_PD = false;

	settings.debug.symx_check_for_NaNs = true;
	stark::Simulation simulation(settings);

	// Only volumetric soft cube
	{
		auto params = stark::DeformableVolumeParametrization::soft_rubber(stark::ElasticityOnly::False);
		params.inertia_damping = 1.0;
		params.strain_young_modulus = 5e3;
		auto block = simulation.deformables->add_volume_grid({ 0.0, 0.0, 0.1 }, 0.2, 5, params);

		// BC
		auto bc = block.create_prescribed_positions_group_with_transformation();
		bc->set_stiffness(1e6);
		bc->add_vertices_in_aabb({ 0.0, 0.0, 0.0 }, { 10.0, 10.0, 0.01 });
	}

	// Volumetric + Skin soft cube
	auto params = stark::DeformableVolumeParametrization::soft_rubber(stark::ElasticityOnly::False);
	params.inertia_damping = 1.0;
	params.strain_young_modulus = 5e3;

	params.use_skin = true;
	params.skin = stark::DeformableSurfaceParametrization::towel(stark::ElasticityOnly::False);
	params.skin.bending_stiffness = 1.0;
	auto block = simulation.deformables->add_volume_grid({ 0.0, 0.0, 0.1 }, 0.2, 5, params)
		.add_displacement({0.0, 0.3, 0.0});

	// BC
	auto bc = block.create_prescribed_positions_group_with_transformation();
	bc->set_stiffness(1e6);
	bc->add_vertices_in_aabb({ 0.0, 0.0, 0.0 }, { 10.0, 10.0, 0.01 });

	// Run
	simulation.run(
		[&]() 
		{
			block.get_skin().set_bending_stiffness(simulation.get_time() / 5.0);  // Increases skin bending stiffness over time
		}
	);
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
	simulation.run();
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
	simulation.run();
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
	settings.contact.friction_stick_slide_threshold = 0.1;

	settings.debug.symx_check_for_NaNs = true;
	settings.debug.symx_finite_difference_check = false;

	stark::Simulation simulation(settings);

	// Add objects
	const double w = 0.3;
	const double w2 = 0.3 * w;
	const double rho = 1e3;
	const double rho2 = 1e4;

	//// Base
	const int n = 10;
	auto [vertices, tets] = stark::utils::generate_tet_grid({ -0.5*w, -0.5*w, -0.5*w }, { 0.5*w, 0.5*w, 0.5*w }, { n, n, n });
	auto params = stark::DeformableVolumeParametrization::soft_rubber(stark::ElasticityOnly::False);
	params.density = rho;
	params.strain_young_modulus = 1e4;
	params.inertia_damping = 1.0;
	params.strain_damping = 1.0;
	params.strain_limit = 999.0;
	auto base = simulation.deformables->add_volume(vertices, tets, params);
	auto bc = base.create_prescribed_positions_group_with_transformation();
	bc->add_vertices_in_aabb({ 0.0, 0.0, -0.5*w }, { 1.0, 1.0, 0.01 });

	//// Top deformable
	if (false) {
		const int n = 1;
		auto [vertices, tets] = stark::utils::generate_tet_grid({ -0.5*w2, -0.5*w2, -0.5*w2 }, { 0.5*w2, 0.5*w2, 0.5*w2 }, { n, n, n });
		stark::utils::move(vertices, { 0.01, 0.0, 0.5*w + 0.5*w2 + 2.0*settings.contact.dhat });
		auto material = stark::DeformableVolumeParametrization::soft_rubber(stark::ElasticityOnly::False);
		params.density = rho2;
		params.inertia_damping = 1.0;
		params.strain_damping = 1.0;
		params.strain_limit = 999.0;
		auto top = simulation.deformables->add_volume(vertices, tets, params);

		simulation.interactions->set_friction(base, top, 0.5);
		//simulation.interactions->disable_collision(base, top);
	}

	//// Top rigid
	else {
		auto top = simulation.rigidbodies->add_box(rho2*std::pow(w2, 3), w2)
			.set_displacement({ 0.01, 0.0, 0.5*w + 0.5*w2 + 2.0*settings.contact.dhat });

		simulation.interactions->set_friction(base, top, 0.5);
		//simulation.interactions->disable_collision(base, top);
	}

	// Run
	simulation.run();
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
	stark::Simulation simulation(settings);

	const double w = 0.1;
	const int n = 5;

	// Base block
	auto [vertices, tets] = stark::utils::generate_tet_grid({ -0.5 * w, -0.5 * w, -0.5 * w }, { 0.5 * w, 0.5 * w, 0.5 * w }, { n, n, n });
	auto params = stark::DeformableVolumeParametrization::soft_rubber(stark::ElasticityOnly::False);
	auto block0 = simulation.deformables->add_volume(vertices, tets, params);

	// BC
	auto bc = block0.create_prescribed_positions_group_with_transformation();
	bc->set_stiffness(1e6);
	bc->add_vertices_in_aabb({ -0.5*w, 0.0, 0.0 }, { 0.001, 2.0 * w, 2.0 * w });

	// Second block
	stark::utils::move(vertices, { w, 0.0, -w });
	auto block1 = simulation.deformables->add_volume(vertices, tets, params);
	simulation.interactions->attach_by_distance(block0, block1);

	// Cloth
	auto [c_vertices, c_triangles] = stark::utils::generate_triangular_grid({ -0.5 * w, -0.5 * w }, { 0.5 * w, 0.5 * w }, { n, n });
	stark::utils::move(c_vertices, { 2.0*w, 0.0, -0.5*w });
	auto c_material = stark::DeformableSurfaceParametrization::towel(stark::ElasticityOnly::False);
	auto cloth = simulation.deformables->add_surface(c_vertices, c_triangles, c_material);
	simulation.interactions->attach_by_distance(block1, cloth);

	// Rigid body
	auto rigid = simulation.rigidbodies->add_box(1.0, w)
		.set_displacement({3.0*w, 0.0, 0.0});
	simulation.interactions->attach_by_distance(rigid, cloth);

	// Run
	simulation.run();
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
	stark::Simulation simulation(settings);

	// Wall
	auto wall = simulation.rigidbodies->add_box(100.0, { 2.0, 0.5, 2.0 })
		.set_displacement({ 0.0, -0.5, 0.0 });
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
			simulation.deformables->add_surface(vertices_cloth, triangles_cloth, stark::DeformableSurfaceParametrization::towel(stark::ElasticityOnly::False))
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
	simulation.run();
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
	stark::Simulation simulation(settings);

	// Wall
	auto wall = simulation.rigidbodies->add_box(10.0, { 2.0, 0.5, 2.0 })
		.set_displacement({ 0.0, -0.5, 0.0 });
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

	auto material_box = stark::DeformableVolumeParametrization::soft_rubber(stark::ElasticityOnly::False);
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
	const double friction = 2.0;
	for (int i = 0; i < (int)boxes.size(); i++) {
		simulation.interactions->set_friction(drum, boxes[i], friction);
		//for (int j = i + 1; j < (int)boxes.size(); j++) {
		//	simulation.interactions->set_friction(boxes[i], boxes[j], friction);
		//}
	}

	// Run
	simulation.run();
}
void car()
{
	/*
		Important:
			This simulation is highly dynamic. Tight convergence, small time steps, etc... matter.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "car";
	settings.output.output_directory = OUTPUT_PATH + "/car";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	
	settings.execution.end_simulation_time = 15.0;
	settings.newton.linear_system_solver = stark::LinearSystemSolver::CG;
	settings.newton.residual = { stark::ResidualType::Acceleration, 0.1 };
	//settings.debug.symx_check_for_NaNs = true;

	settings.simulation.adaptive_time_step.set(0.0, 0.005, 0.005);
	settings.newton.project_to_PD = true;
	settings.contact.dhat = 0.02;
	settings.contact.friction_stick_slide_threshold = 0.1;
	settings.contact.adaptive_contact_stiffness.set(1e8, 1e8, 1e12);

	auto simulation = std::make_shared<stark::Simulation>(settings);

	// Car
	//// Soft tires
	auto parametrization = stark::VehicleFourWheels::Parametrization::sedan();
	parametrization.chassis.collision_mesh = stark::utils::load_obj(MODELS_PATH + "/sedan_chassis_collision_mesh.obj");
	parametrization.wheels.tires.mesh = stark::utils::load_vtk<4>(MODELS_PATH + "/tire_sedan.vtk");
	stark::VehicleFourWheels car(simulation, parametrization, "sedan");

	////// Rigid tires
	//auto parametrization = stark::VehicleFourWheels::Parametrization::sedan();
	//parametrization.chassis.collision_mesh = stark::utils::load_obj(MODELS_PATH + "/sedan_chassis_collision_mesh.obj");
	//stark::VehicleFourWheels rigid(simulation, parametrization, "rigid");


	// Move the car

	// Environment
	stark::StaticPlaneHandler ground = simulation->interactions->add_static_plane({ 0.0, 0.0, -0.02 }, Eigen::Vector3d::UnitZ());
	//car.set_wheels_friction(ground, 1.25);
	car.set_wheels_friction(ground, 5.0);
	car.set_chassis_friction(ground, 0.5);

	//// Obstacle
	//if (false) {
	//	auto obstacle = simulation->rigidbodies->add_box(1000.0, 1.0)
	//		.set_rotation(45.0, Eigen::Vector3d::UnitX())
	//		.set_displacement({ 0.8, 50.0, 0.0 })
	//		.add_to_output_label("obstacle");
	//	simulation->rigidbodies->add_constraint_fix(obstacle)
	//		.set_stiffness(1e8)
	//		.set_tolerance_in_deg(60.0)
	//		.set_tolerance_in_m(1.0);
	//	simulation->interactions->disable_collision(ground, obstacle);
	//}

	// Script
	//// Velocity
	car.add_event_brake(0.0, 1.0);
	car.add_event_target_velocity_in_kmh(1.0, 5.0, 0.0, 50.0, stark::utils::BlendType::Instant);
	car.add_event_target_velocity_in_kmh(7.1, 99.9, 50.0, 100.0, stark::utils::BlendType::Instant);

	//// Steering
	car.add_event_global_steering_in_deg(0.0, 5.0, 0.0, 0.0, stark::utils::BlendType::Linear);
	car.add_event_global_steering_in_deg(5.0, 7.0, 0.0, 120.0, stark::utils::BlendType::Linear);
	car.add_event_global_steering_in_deg(7.0, 7.2, 120.0, 120.0, stark::utils::BlendType::Linear);
	car.add_event_global_steering_in_deg(7.2, 10.0, 120.0, 0.0, stark::utils::BlendType::Linear);
	car.add_event_global_steering_in_deg(10.0, 99.0, 0.0, 0.0, stark::utils::BlendType::Linear);

	// Run
	simulation->run();
}

int main()
{
	//rb();
	//net();
	//hanging_cloth();
	//rubber_block_and_skin();
	//hanging_cloth_and_reinforcement();
	//hanging_cloth_scaled();
	hanging_box_scaled();
	//simple_collision();
	//edge_edge_collision();
	//heavy_box_rigid_and_deformable();
	//rb_constraints_all();
	//attachments();
	//laundry_cloth();
	//laundry_soft_boxes();

	//car();
}
