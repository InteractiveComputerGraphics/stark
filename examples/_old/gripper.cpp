#include "gripper.h"

#include "paths.h"

struct Gripper
{
	int hand = 0;
	int right = 0;
	int left = 0;
};
Gripper make_franka_gripper(stark::models::Simulation& sim, const std::string right_finger_collision_path, const int cup = -1, const int box = -1, const double cup_friction = 0.0)
{
	// Input
	// Manual: https://download.franka.de/documents/220010_Product%20Manual_Franka%20Hand_1.2_EN.pdf
	const double grasp_force = 1.0;
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
	sim.rigid_bodies.add_to_output_group("hand", {hand});
	sim.rigid_bodies.add_to_output_group("fingers", {right, left});

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
	if (box >= 0) {
		sim.rigid_bodies.set_friction(left, box, cup_friction);
		sim.rigid_bodies.set_friction(right, box, cup_friction);
	}
	return { hand, right, left };
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
	settings.output.simulation_name = "gripper";
	settings.output.output_directory = BASE_PATH + "/gripper_local_2g_1N";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	//settings.execution.end_simulation_time = 40.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.005, 0.005);
	settings.simulation.boundary_conditions_stiffness = 1e7;

	settings.newton.use_direct_linear_solve = true;
	settings.newton.newton_tol = 1e-4;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = true;
	settings.contact.friction_stick_slide_threshold = 0.0001;
	settings.contact.adaptive_contact_stiffness.set(1e6, 1e6, 1e12);
	settings.contact.adaptive_contact_stiffness.success_multiplier = 0.8;
	settings.contact.adaptive_contact_stiffness.n_successful_iterations_to_increase = 100;
	settings.contact.dhat = 0.0005;

	stark::models::Simulation sim(settings);

	// Floor
	const double floor_friction = 1.0;
	const int floor = sim.rigid_bodies.add_box(10.0, { 0.5, 0.5, 0.1 }, { 0, 0.25, -0.1 });
	sim.rigid_bodies.add_constraint_freeze(floor);
	sim.rigid_bodies.damping = 10.0;
	sim.rigid_bodies.add_to_output_group("floor", floor);

	// Cup
	int cup_id = -1;
	if (true) {
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 3>> triangles;

		//stark::utils::Mesh m = stark::utils::make_cylinder(0.027, 0.10, 64, 16);
		//stark::utils::move(m.vertices, {0.0, 0.077, 0.005});
		//vertices = m.vertices;
		//triangles = m.triangles;

		stark::utils::load_obj(vertices, triangles, MODELS_PATH + "/cup_simple_small_lip.obj");
		cup_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);

		sim.cloth.set_density(cup_id, 0.2);
		sim.cloth.set_strain_parameters(cup_id, 1e6, 0.45);  // Note that while PET plastic has a Young's modulus on the order of $E = \SI{1e9}{\pascal}$, this leads to extremely stiff behavior of the cup. Therefore, we adjust it to obtain a more realistic deformation under such load.
		sim.cloth.set_bending_stiffness(cup_id, 1e-4);  // smooth consistent surface. Without it its just a bunch of triangles. With only it, numerically unstable
		sim.cloth.set_damping(2.0, 0.2, 0.2);
		sim.cloth.self_collisions_enabled = false;
		sim.cloth.cutoff_bending_angle_deg = 10.0;

		// Attach close vertices
		for (int i = 0; i < (int)vertices.size(); i++) {
			for (int j = i + 1; j < (int)vertices.size(); j++) {
				if ((vertices[i] - vertices[j]).norm() < 0.0002) {
					sim.cloth.set_attached_vertices(cup_id, i, cup_id, j);
				}
			}
		}
	}
	int box_id = -1;
	if (false) {
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 3>> triangles;
		stark::utils::load_obj(vertices, triangles, MODELS_PATH + "/box_cup.obj");
		box_id = sim.rigid_bodies.add_box(0.2, { 0.044, 0.044, 0.095 }, vertices, triangles, { 0, 0.07, 0.0 });
		sim.rigid_bodies.add_to_output_group("box", { box_id });
	}

	// Gripper
	Gripper gripper = make_franka_gripper(sim, MODELS_PATH + "/franka_finger.obj", cup_id, box_id, 0.3);

	// Nirvana ball for sequence
	const int ball = sim.rigid_bodies.add_sphere(0.01, 0.003, { -0.01, 0.064, -2.0 }, 0.0, { 1, 0, 0 }, 1);
	sim.rigid_bodies.add_constraint_freeze(ball);
	sim.rigid_bodies.add_to_output_group("balls", { ball });

	// Run
	double time_next_drop = 4.0;
	const double drop_dt = 0.2;
	const double balls_mass = 0.005;
	const double balls_radius = 0.0054;
	sim.stark.run(
		[&]()
		{
			const double t = sim.stark.current_time;
			const double dt = sim.stark.settings.simulation.adaptive_time_step.value;

			if (t < 2.0) {
			}
			else if (t < 4.0) {
				sim.rigid_bodies.constraints.anchor_points.conn.clear();
				sim.rigid_bodies.constraints.anchor_points.loc.clear();
				sim.rigid_bodies.constraints.anchor_points.target.clear();

				sim.rigid_bodies.add_displacement(floor, {0, 0.1*dt, 0});

				sim.rigid_bodies.add_constraint_freeze(gripper.left);
				sim.rigid_bodies.add_constraint_freeze(gripper.hand);
				sim.rigid_bodies.add_constraint_freeze(floor);
			} 
			else {
				if (t < 5.0) {
					sim.stark.settings.simulation.adaptive_time_step.max = 0.001;
				}
				else {
					sim.stark.settings.simulation.adaptive_time_step.max = 0.01;
				}
				if (t > time_next_drop) {
					time_next_drop += drop_dt;
					//const int idx0 = sim.rigid_bodies.add_box(balls_mass, {0.005, 0.005, 0.005}, { -0.01, 0.064, 0.06 });
					//const int idx1 = sim.rigid_bodies.add_box(balls_mass, {0.005, 0.005, 0.005}, { -0.01, 0.074, 0.06 });
					//const int idx2 = sim.rigid_bodies.add_box(balls_mass, {0.005, 0.005, 0.005}, {  0.01, 0.064, 0.06 });
					//const int idx3 = sim.rigid_bodies.add_box(balls_mass, {0.005, 0.005, 0.005}, {  0.01, 0.074, 0.06 });
					const int idx0 = sim.rigid_bodies.add_sphere(balls_mass, balls_radius, { -0.015, 0.06, 0.06 }, 0.0, { 1, 0, 0 }, 1);
					const int idx1 = sim.rigid_bodies.add_sphere(balls_mass, balls_radius, { -0.015, 0.09, 0.06 }, 0.0, { 1, 0, 0 }, 1);
					const int idx2 = sim.rigid_bodies.add_sphere(balls_mass, balls_radius, {  0.015, 0.09, 0.06 }, 0.0, { 1, 0, 0 }, 1);
					const int idx3 = sim.rigid_bodies.add_sphere(balls_mass, balls_radius, {  0.015, 0.06, 0.06 }, 0.0, { 1, 0, 0 }, 1);
					sim.rigid_bodies.add_to_output_group("balls", { idx0, idx1, idx2, idx3 });
				}
			}
		}
	);
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
