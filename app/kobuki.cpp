#include "kobuki.h"

Kobuki make_kobuki(stark::models::Simulation& sim)
{
	// Input
	const double mass = 2.8;
	const double torque = 10.0;
	const double max_velocity = 1.0;

	auto& rb = sim.rigid_bodies;

	// Body
	// TODO: more accurate collision mesh. Read VTK (also for towel)
	const int body = rb.add_cylinder(mass, 0.175, 0.07, { 0, 0, 0.05 }, 0.0, Eigen::Vector3d::UnitY(), 64);
	rb.add_to_output_group("body", body);

	// Wheels
	const int front = rb.add_cylinder(0.1, 0.0135, 0.012, { 0, 0.115, 0.0135 }, 90.0, Eigen::Vector3d::UnitY(), 32, 2);
	const int back = rb.add_cylinder(0.1, 0.0135, 0.012, { 0, -0.136, 0.0135 }, 90.0, Eigen::Vector3d::UnitY(), 32, 2);
	const int left = rb.add_cylinder(0.1, 0.035, 0.02, { -0.115, 0, 0.035 }, 90.0, Eigen::Vector3d::UnitY(), 64, 4);
	const int right = rb.add_cylinder(0.1, 0.035, 0.02, { 0.115, 0, 0.035 }, 90.0, Eigen::Vector3d::UnitY(), 64, 4);
	rb.add_to_output_group("wheels", { front, back, left, right });

	// Contraints
	rb.add_constraint_freeze(body);
	rb.add_constraint_motor(body, left, { -0.115, 0, 0.035 }, -Eigen::Vector3d::UnitX(), torque, max_velocity/0.035, 100.0);
	rb.add_constraint_motor(body, right, { 0.115, 0, 0.035 }, -Eigen::Vector3d::UnitX(), torque, max_velocity/0.035, 100.0);
	rb.add_constraint_hinge_joint(body, front, { 0, 0.115, 0.0135 }, Eigen::Vector3d::UnitX());
	rb.add_constraint_hinge_joint(body, back, { 0, -0.136, 0.0135 }, Eigen::Vector3d::UnitX());

	// TODO: Disable collisions

	return Kobuki();
}

void kobuki_test()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "kobuki_test";
	settings.output.output_directory = "D:/sciebo/wd/stark/kobuki_test";
	settings.output.codegen_directory = "D:/sciebo/wd/stark/codegen";
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 120;

	settings.execution.end_simulation_time = 1.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.001, 0.001);

	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.contact.adaptive_contact_stiffness.value = 1e8;
	settings.contact.dhat = 0.001;
	stark::models::Simulation sim(settings);

	// Kobuki
	Kobuki kobuki = make_kobuki(sim);

	// Run
	sim.stark.run();
}
