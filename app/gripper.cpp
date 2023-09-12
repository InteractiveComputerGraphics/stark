#include "gripper.h"

#include "paths.h"


void make_franka_gripper(stark::models::Simulation& sim, const std::string right_finger_collision_path, const int cup = -1, const double cup_friction = 0.0)
{
	// Input
	// Manual: https://download.franka.de/documents/220010_Product%20Manual_Franka%20Hand_1.2_EN.pdf
	const double grasp_force = 9.84;
	auto& rb = sim.rigid_bodies;

	// Body
	const int hand = rb.add_box(0.5, { 0.202, 0.081, 0.037 });

	// Fingers
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::load_obj(vertices, triangles, right_finger_collision_path); // Inspiration: https://github.com/justagist/franka_panda_description/tree/master/meshes/visual
	const int right = rb.add_box(0.1, { 0.025, 0.054, 0.025 }, vertices, triangles, { 0.052445, 0.060518, 0.0 });
	stark::utils::scale(vertices, { -1.0, 1.0, 1.0 });
	const int left = rb.add_box(0.1, { 0.025, 0.054, 0.025 }, vertices, triangles, { -0.052445, 0.060518, 0.0 });
	//sim.rigid_bodies.add_to_output_group("hand", {hand, right, left});

	// Constraints
	rb.add_constraint_freeze(hand);
	rb.add_constraint_parallel_gripper(hand, right, left, Eigen::Vector3d::UnitX(), grasp_force, 0.1);

	// Disable pairwise collisions
	rb.disable_collisions(hand, left);
	rb.disable_collisions(hand, right);

	// Friction
	if (cup >= 0) {
		sim.interactions.set_friction(left, cup, cup_friction);
		sim.interactions.set_friction(right, cup, cup_friction);
	}
}

void gripper_box()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "gripper_box";
	settings.output.output_directory = BASE_PATH + "/gripper_box";
	settings.output.codegen_directory = COMPILE_PATH;

	settings.execution.end_simulation_time = 2.0;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = true;
	settings.contact.friction_stick_slide_threshold = 0.001;
	settings.contact.adaptive_contact_stiffness.value = 1e6;
	settings.contact.dhat = 0.001;

	stark::models::Simulation sim(settings);

	// Floor
	const double floor_friction = 1.0;
	const int floor = sim.rigid_bodies.add_box(10.0, { 0.5, 0.5, 0.1 }, { 0, 0.25, -0.1 });
	sim.rigid_bodies.add_constraint_freeze(floor);
	//sim.rigid_bodies.add_to_output_group("floor", floor);

	// Cup
	const int box = sim.rigid_bodies.add_box(0.5, { 0.044, 0.044, 0.044 }, { 0, 0.07, 0.0 });
	//sim.rigid_bodies.add_to_output_group("box", floor);

	// Kobuki
	make_franka_gripper(sim, MODELS_PATH + "/franka_finger.obj");

	// Run
	sim.stark.run();
}
void gripper_cup(const double bending_stiffness)
{
	stark::Settings settings = stark::Settings();
	//settings.output.simulation_name = "gripper_cup" + fmt::format("{:.1e}", bending_stiffness);
	settings.output.simulation_name = "gripper_cup_simple";
	settings.output.output_directory = BASE_PATH + "/gripper_cup";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 2.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.01, 0.01);
	settings.simulation.boundary_conditions_stiffness = 1e7;

	settings.newton.max_newton_iterations = 50;
	settings.newton.use_direct_linear_solve = true;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = true;
	settings.contact.friction_stick_slide_threshold = 0.001;
	settings.contact.adaptive_contact_stiffness.value = 1e6;
	settings.contact.dhat = 0.002;

	stark::models::Simulation sim(settings);

	// Floor
	const double floor_friction = 1.0;
	const int floor = sim.rigid_bodies.add_box(10.0, { 0.5, 0.5, 0.1 }, { 0, 0.25, -0.1 });
	sim.rigid_bodies.add_constraint_freeze(floor);
	//sim.rigid_bodies.add_to_output_group("floor", floor);

	// Cup
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	
	//stark::utils::Mesh m = stark::utils::make_cylinder(0.027, 0.10, 64, 16);
	//stark::utils::move(m.vertices, {0.0, 0.077, 0.005});
	//vertices = m.vertices;
	//triangles = m.triangles;

	stark::utils::load_obj(vertices, triangles, MODELS_PATH + "/cup_simple.obj");
	const int cup_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);

	sim.cloth.set_density(cup_id, 0.2);
	sim.cloth.set_strain_parameters(cup_id, 1e6, 0.45);  // Provides the resistance to be squeezed
	sim.cloth.set_bending_stiffness(cup_id, 1e-4);  // smooth consistent surface. Without it its just a bunch of triangles. With only it, numerically unstable
	sim.cloth.set_damping(2.0, 0.2, 0.2);
	sim.cloth.self_collisions_enabled = false;
	sim.cloth.cutoff_bending_angle_deg = 10.0;

	// Attach close vertices
	for (int i = 0; i < (int)vertices.size(); i++) {
		for (int j = 0; j < (int)vertices.size(); j++) {
			if ((vertices[i] - vertices[j]).norm() < 0.001) {
				sim.cloth.set_attached_vertices(cup_id, i, cup_id, j);
			}
		}
	}

	// Kobuki
	make_franka_gripper(sim, MODELS_PATH + "/franka_finger.obj", cup_id, 0.7);

	// Run
	sim.stark.run();
}

void gripper_suite()
{
	gripper_cup(1e-6);
	gripper_cup(2e-6);
	gripper_cup(5e-6);
	gripper_cup(8e-6);
	gripper_cup(1e-5);
	gripper_cup(2e-5);
	gripper_cup(5e-5);
	gripper_cup(8e-5);
	gripper_cup(1e-4);
}
