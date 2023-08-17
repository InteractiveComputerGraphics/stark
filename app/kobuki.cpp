#include "kobuki.h"

Kobuki make_kobuki(stark::models::Simulation& sim)
{
	// Input
	const double mass = 2.951;
	const double torque = 0.0666; // (Per-motor) Torque output at stall, that is, at max output due to an obstacle
	const double max_linear_velocity = 0.26;
	const double power_wheels_friction = 1.0;

	auto& rb = sim.rigid_bodies;

	// Body
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::load_obj(vertices, triangles, "D:/sciebo/wd/stark/res/kobuki_collision.obj");
	const int body = rb.add_cylinder(mass, 0.175, 0.07, vertices, triangles, { 0, 0, 0.057 });
	rb.add_to_output_group("body", body);

	// Wheels
	const int front = rb.add_cylinder(0.1, 0.0135, 0.012, { 0, 0.115, 0.0135 }, 90.0, Eigen::Vector3d::UnitY(), 32, 2);
	const int back = rb.add_cylinder(0.1, 0.0135, 0.012, { 0, -0.136, 0.0135 }, 90.0, Eigen::Vector3d::UnitY(), 32, 2);
	const int left = rb.add_cylinder(0.1, 0.035, 0.02, { -0.115, 0, 0.035 }, 90.0, Eigen::Vector3d::UnitY(), 64, 4);
	const int right = rb.add_cylinder(0.1, 0.035, 0.02, { 0.115, 0, 0.035 }, 90.0, Eigen::Vector3d::UnitY(), 64, 4);
	rb.add_to_output_group("wheels", { front, back, left, right });

	// Contraints
	rb.add_constraint_motor(body, left, { -0.115, 0, 0.035 }, -Eigen::Vector3d::UnitX(), torque, max_linear_velocity/0.035, 10.0);
	rb.add_constraint_motor(body, right, { 0.115, 0, 0.035 }, -Eigen::Vector3d::UnitX(), torque, max_linear_velocity/0.035, 10.0);
	rb.add_constraint_hinge_joint(body, front, { 0, 0.115, 0.0135 }, Eigen::Vector3d::UnitX());
	rb.add_constraint_hinge_joint(body, back, { 0, -0.136, 0.0135 }, Eigen::Vector3d::UnitX());

	// Disable pairwise collisions
	rb.disable_collisions(body, front);
	rb.disable_collisions(body, back);
	rb.disable_collisions(body, left);
	rb.disable_collisions(body, right);

	// Friction
	rb.set_friction(left, power_wheels_friction);
	rb.set_friction(right, power_wheels_friction);

	// Other
	//rb.add_constraint_freeze(body);


	return Kobuki();
}



void kobuki_test()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "kobuki_test";
	settings.output.output_directory = "D:/sciebo/wd/stark/kobuki_test";
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 60;

	settings.execution.end_simulation_time = 0.02;
	settings.simulation.adaptive_time_step.set(0.0, 0.001, 0.001);
	settings.simulation.boundary_conditions_stiffness = 1e8;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = true;
	settings.contact.adaptive_contact_stiffness.value = 1e8;
	settings.contact.dhat = 0.001;
	stark::models::Simulation sim(settings);

	// Kobuki
	Kobuki kobuki = make_kobuki(sim);

	// Floor
	const double floor_friction = 0.1;
	const int floor = sim.rigid_bodies.add_box(10.0, { 2, 3, 0.1 }, { 0, 1, -(0.05 + settings.contact.dhat) });
	sim.rigid_bodies.set_friction(floor, floor_friction);
	sim.rigid_bodies.add_constraint_freeze(floor);
	sim.rigid_bodies.add_to_output_group("floor", floor);

	// Run
	sim.stark.run();
}
