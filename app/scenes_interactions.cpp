#include "scenes_interactions.h"

#include <Eigen/Dense>
#include <stark>


void interaction_cloth_rb()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "interaction_cloth_rb";
	settings.output.output_directory = "D:/sciebo/wd/stark/interaction_cloth_rb";
	settings.output.codegen_directory = "D:/sciebo/wd/stark/codegen";
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 120;

	settings.execution.end_simulation_time = 2.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.005, 0.005);

	settings.contact.friction_enabled = false;
	settings.contact.adaptive_contact_stiffness.value = 1e6;
	settings.contact.dhat = 0.001;
	stark::models::Simulation sim(settings);

	// Slope
	const double rot_deg = 0.0;

	// Rigid bodies
	const double mu = 1.0;
	const int o1 = sim.rigid_bodies.add_box(10.0, { 2.0, 2.0, 0.1 }, { 0, 0, 0 });
	sim.rigid_bodies.add_rotation(o1, -rot_deg, Eigen::Vector3d::UnitX());
	sim.rigid_bodies.set_friction(o1, mu);
	sim.rigid_bodies.add_constraint_freeze(o1);

	// Cloth
	const double scale = 1.0;
	const int n = 30;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5*scale, -0.5*scale }, { 0.5*scale, 0.5*scale }, { n, n });
	stark::utils::rotate_deg(vertices, 85.0, Eigen::Vector3d::UnitX());
	stark::utils::move(vertices, {0.0, 0.0, 0.05 + 0.5*scale + 1.5*settings.contact.dhat });
	const int cloth_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	sim.cloth.set_bending_stiffness(cloth_id, 1e-3);

	// Run
	sim.stark.run();
}

void interaction_cloth_rb_bowl()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "interaction_cloth_rb_bowl";
	settings.output.output_directory = "D:/sciebo/wd/stark/interaction_cloth_rb_bowl";
	settings.output.codegen_directory = "D:/sciebo/wd/stark/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.output.fps = 60;

	settings.execution.end_simulation_time = 1.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.005, 0.005);

	settings.contact.friction_enabled = false;
	settings.contact.adaptive_contact_stiffness.value = 1e6;
	settings.contact.dhat = 0.001;
	stark::models::Simulation sim(settings);

	// Rigid bodies
	const double mu = 1.0;
	const double mass = 10.0;
	const int o1 = sim.rigid_bodies.add_sphere(mass, 1.0, { 0, 0, 0.5 });
	sim.rigid_bodies.add_constraint_freeze(o1);

	// Cloth
	const int n = 5;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.1, -0.1 }, { 0.1, 0.1 }, { n, n });
	stark::utils::move(vertices, { 0.2, 0.2, 0.0 });
	const int cloth_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);

	// Run
	sim.stark.run();
}
