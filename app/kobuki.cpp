#include "kobuki.h"

struct Towel
{
	const double l = 1.35;
	const double w = 0.75;
	const double mass = 0.484;
	const double density = 0.484;
	const double friction = 0.25;
};
Towel towel;

void make_kobuki(stark::models::Simulation& sim)
{
	// Input
	const double power_multiplier = 5.0;
	const double mass = 2.951;
	const double torque = power_multiplier*0.0666; // (Per-motor) Torque output at stall, that is, at max output due to an obstacle
	const double max_linear_velocity = power_multiplier*0.26;
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
	rb.add_constraint_motor(body, left, { -0.115, 0, 0.035 }, -Eigen::Vector3d::UnitX(), torque, max_linear_velocity/0.035, 1.0);
	rb.add_constraint_motor(body, right, { 0.115, 0, 0.035 }, -Eigen::Vector3d::UnitX(), torque, max_linear_velocity/0.035, 1.0);
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
}
int make_towel(stark::models::Simulation& sim, const int n_short_side = 30)
{
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	const std::array<int, 2> n = stark::utils::generate_triangular_grid(vertices, triangles, towel.w, towel.l, n_short_side);
	stark::utils::move(vertices, {0, 1.0, 0.5});

	const int cloth_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);
	sim.cloth.set_density(cloth_id, towel.density);
	sim.cloth.set_strain_parameters(cloth_id, 100.0, 0.3, 1.1, 1000.0);
	sim.cloth.set_bending_stiffness(cloth_id, 2e-5);
	sim.cloth.set_friction(cloth_id, towel.friction);
	sim.cloth.set_damping(0.2);

	//sim.cloth.set_vertex_target_position_as_initial(cloth_id, n[1]);
	//sim.cloth.set_vertex_target_position_as_initial(cloth_id, (int)vertices.size() - 1);

	return cloth_id;
}


void towel_parametrization()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "towel_final";
	settings.output.output_directory = "D:/sciebo/wd/stark/towel_parametrization";
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 15.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.01, 0.01);
	settings.simulation.boundary_conditions_stiffness = 1e7;

	settings.newton.max_newton_iterations = 50;

	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.contact.friction_stick_slide_threshold = 0.001;
	settings.contact.adaptive_contact_stiffness.value = 1e6;
	settings.contact.dhat = 0.0025;

	stark::models::Simulation sim(settings);

	// Floor
	int floor_id = -1;
	{
		const double floor_friction = 2.0;
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 3>> triangles;
		stark::utils::generate_triangular_grid(vertices, triangles, 3.0, 3.0, 1);
		floor_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);
		sim.cloth.freeze(floor_id);
		//sim.cloth.add_to_output_group("floor", floor);
	}

	// Towel
	int towel_id = -1;
	{
		const double towel_friction = 0.5;

		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 3>> triangles;
		const std::array<int, 2> n = stark::utils::generate_triangular_grid(vertices, triangles, towel.w, towel.l, 75);
		stark::utils::rotate_deg(vertices, -90.0, {1, 0, 0});
		stark::utils::move(vertices, { 0, 0, towel.l });

		towel_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);
		sim.cloth.set_density(towel_id, towel.density);
		sim.cloth.set_strain_parameters(towel_id, 100.0, 0.3, 0.1, 1.0);
		sim.cloth.set_bending_stiffness(towel_id, 5e-6);
		sim.cloth.set_friction(towel_id, towel_friction);
		sim.cloth.set_damping(2.0);
		sim.cloth.bending_damping = 0.1;
		sim.cloth.strain_damping = 0.1;
	}

	// Run
	sim.stark.run(
		[&]() 
		{
			const double t = sim.stark.current_time;
			sim.cloth.clear_vertex_target_position();
			sim.cloth.freeze(floor_id);
			if (t < 5.0) {
				sim.cloth.set_vertex_target_position_as_initial(towel_id, 0);
			}
		}
	);
}

void kobuki_test()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "kobuki_test";
	settings.output.output_directory = "D:/sciebo/wd/stark/kobuki_test";
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 20.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.01, 0.01);
	settings.simulation.boundary_conditions_stiffness = 1e7;

	settings.newton.max_newton_iterations = 50;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = true;
	settings.contact.friction_stick_slide_threshold = 0.01;
	settings.contact.adaptive_contact_stiffness.value = 1e6;
	settings.contact.dhat = 0.001;

	stark::models::Simulation sim(settings);

	// Kobuki
	make_kobuki(sim);

	// Floor
	const double floor_friction = 1.0;
	const int floor = sim.rigid_bodies.add_box(10.0, { 2, 3, 0.1 }, { 0, 1, -(0.05 + settings.contact.dhat) });
	sim.rigid_bodies.set_friction(floor, floor_friction);
	sim.rigid_bodies.add_constraint_freeze(floor);
	sim.rigid_bodies.add_to_output_group("floor", floor);

	// Towel
	const int cloth_id = make_towel(sim);

	// Run
	sim.stark.run();
}
