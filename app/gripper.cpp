#include "gripper.h"

#include "paths.h"


void make_franka_gripper(stark::models::Simulation& sim, const std::string right_finger_collision_path, const int cup = -1, const double cup_friction = 0.0)
{
	// Input

	auto& rb = sim.rigid_bodies;

	// Body
	const int hand = rb.add_box(0.5, { 0.202, 0.081, 0.037 });

	// Fingers
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::load_obj(vertices, triangles, right_finger_collision_path);
	const int right = rb.add_box(0.1, { 0.025, 0.054, 0.025 }, vertices, triangles, { 0.052445, 0.060518, 0.0 });
	stark::utils::scale(vertices, { -1.0, 1.0, 1.0 });
	const int left = rb.add_box(0.1, { 0.025, 0.054, 0.025 }, vertices, triangles, { -0.052445, 0.060518, 0.0 });
	sim.rigid_bodies.add_to_output_group("hand", {hand, right, left});

	// Contraints
	rb.add_constraint_freeze(hand);
	rb.add_constraint_freeze(right);
	rb.add_constraint_freeze(left);

	// Disable pairwise collisions
	rb.disable_collisions(hand, left);
	rb.disable_collisions(hand, right);

	// Friction
	if (cup >= 0) {
		rb.set_friction(hand, left, cup_friction);
		rb.set_friction(hand, right, cup_friction);
	}
}

void gripper_test()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "gripper_test";
	settings.output.output_directory = BASE_PATH + "/gripper_test";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 20.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.01, 0.01);
	settings.simulation.boundary_conditions_stiffness = 1e7;

	settings.newton.max_newton_iterations = 50;

	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.contact.friction_stick_slide_threshold = 0.005;
	settings.contact.adaptive_contact_stiffness.value = 1e5;
	settings.contact.dhat = 0.01;

	stark::models::Simulation sim(settings);

	// Floor
	const double floor_friction = 1.0;
	const int floor = sim.rigid_bodies.add_box(10.0, { 0.5, 0.5, 0.1 }, { 0, 0, -0.1 });
	sim.rigid_bodies.add_constraint_freeze(floor);
	sim.rigid_bodies.add_to_output_group("floor", floor);

	// Kobuki
	make_franka_gripper(sim, MODELS_PATH + "/franka_finger.obj", floor);

	// Towel
	//const int cloth_id = make_towel(sim);

	// Run
	sim.stark.run();
}
