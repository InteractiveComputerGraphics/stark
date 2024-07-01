#include "scenes_interactions.h"

#include "paths.h"

#include <Eigen/Dense>
#include <stark>


void interaction_cloth_rb()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "interaction_cloth_rb";
	settings.output.output_directory = BASE_PATH + "/interaction_cloth_rb";
	 settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 120;

	settings.execution.end_simulation_time = 2.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.005, 0.005);

	settings.contact.friction_enabled = true;
	settings.contact.adaptive_contact_stiffness.value = 1e6;
	settings.contact.dhat = 0.001;
	stark::models::Simulation sim(settings);

	// Slope
	const double rot_deg = 0.0;

	// Rigid bodies
	const double mu = 1.0;
	const int o1 = sim.rigid_bodies.add_box(10.0, { 0.2, 2.0, 0.1 }, { 0, 0, 0 });
	sim.rigid_bodies.add_rotation(o1, -rot_deg, Eigen::Vector3d::UnitX());
	sim.rigid_bodies.add_constraint_freeze(o1);

	// Cloth
	const double scale = 1.0;
	const int n = 1;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5*scale, -0.5*scale }, { 0.5*scale, 0.5*scale }, { n, n });
	stark::utils::rotate_deg(vertices, 75.0, Eigen::Vector3d::UnitX());
	stark::utils::move(vertices, {0.0, 0.0, 0.05 + 0.5*scale + 1.5*settings.contact.dhat });
	const int cloth_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	sim.cloth.set_bending_stiffness(cloth_id, 1e-3);
	
	sim.interactions.set_friction(o1, cloth_id, mu);

	// Run
	sim.stark.run();
}
void interaction_cloth_rb_bowl()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "interaction_cloth_rb_bowl";
	settings.output.output_directory = BASE_PATH + "/interaction_cloth_rb_bowl";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 5.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.005, 0.005);

	settings.contact.friction_enabled = true;
	//settings.contact.adaptive_contact_stiffness.value = 1e6;
	settings.contact.dhat = 0.002;
	stark::models::Simulation sim(settings);

	// Rigid bodies
	const double mass = 10.0;
	const int o1 = sim.rigid_bodies.add_sphere(mass, 0.5, { 0, 0, 0.25 });
	sim.rigid_bodies.add_constraint_freeze(o1);

	// Cloth
	const int n = 2;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.1, -0.1 }, { 0.1, 0.1 }, { n, n });
	stark::utils::move(vertices, { 0.2, 0.2, 0.0 });
	const int cloth_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);

	// Interactions
	sim.interactions.set_friction(o1, cloth_id, 1.0);

	// Run
	sim.stark.run();
}
void laundry_cloth()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "laundry_cloth";
	settings.output.output_directory = BASE_PATH + "/laundry_cloth";
	 settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 20.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.01, 0.01);
	settings.newton.max_newton_iterations = 20;
	settings.newton.max_line_search_iterations = 10;
	settings.newton.newton_tol = 1e-3;
	settings.newton.project_to_PD = true;

	settings.contact.adaptive_contact_stiffness.set(1e4, 1e4, 1e12);
	settings.contact.adaptive_contact_stiffness.success_multiplier = 0.8;
	settings.contact.adaptive_contact_stiffness.n_successful_iterations_to_increase = 50;
	settings.contact.friction_stick_slide_threshold = 0.01;
	settings.contact.dhat = 0.002;
	stark::models::Simulation sim(settings);

	// Wall
	const int wall = sim.rigid_bodies.add_box(10.0, { 2.0, 0.5, 2.0 }, { 0.0, -0.8, 0.0 });
	sim.rigid_bodies.add_constraint_freeze(wall);
	//sim.rigid_bodies.add_to_output_group("wall", wall);

	// Drum
	int drum = -1;
	{
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 3>> triangles;
		stark::utils::load_obj(vertices, triangles, MODELS_PATH + "/laundry_drum_2.obj");
		//stark::utils::scale(vertices, { 0.85, 0.5, 0.85 });
		drum = sim.rigid_bodies.add_cylinder(1.0, 0.25, 0.25, vertices, triangles);// , { 0, 0, 0 }, 90.0, { 1, 0, 0 });
		sim.rigid_bodies.add_constraint_motor(wall, drum, { 0, 0, 0 }, Eigen::Vector3d::UnitY(), 50.0, 0.75 * 3.14, /*delay*/0.01);
		sim.rigid_bodies.add_to_output_group("drum", drum);
		sim.rigid_bodies.disable_collisions(wall, drum);
	}

	// Cloth
	const double friction = 1.0;
	const double scale = 0.34;
	const int n_cloths = 1;// 8;
	const double spacing = 0.025;
	const int cloth_resolution = 100;// 100;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5 * scale, -0.5 * scale }, { 0.5 * scale, 0.5 * scale }, { cloth_resolution, cloth_resolution });
	stark::utils::rotate_deg(vertices, -85.0, Eigen::Vector3d::UnitX());
	stark::utils::move(vertices, { 0.0, -0.1, 0.0 });

	for (size_t i = 0; i < n_cloths; i++) {
		const int cloth_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);
		sim.cloth.set_bending_stiffness(cloth_id, 1e-6);
		sim.cloth.set_strain_parameters(cloth_id, 100.0, 0.3, 999999.9, 1.0);
		sim.interactions.set_friction(drum, cloth_id, friction);
		stark::utils::move(vertices, { 0.0, spacing, 0.0 });
	}

	for (int i = 0; i < sim.cloth.get_n_cloths(); i++) {
		for (int j = i + 1; j < sim.cloth.get_n_cloths(); j++) {
			sim.cloth.set_friction(i, j, friction);
		}
	}

	// Run
	sim.stark.run();
}
