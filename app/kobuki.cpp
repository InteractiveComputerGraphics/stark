#include "kobuki.h"

#include <unordered_map>

#include "paths.h"

struct Towel
{
	const double l = 1.35;
	const double w = 0.75;
	const double mass = 0.484;
	const double density = 0.484;
	const double thickness = 0.04/8.0;  // 8 layers reach 5cm tall
	const double friction = 1.0;
};
Towel towel;

void make_kobuki(stark::models::Simulation& sim, const std::string kobuki_collision_path)
{
	// Input
	const double power_multiplier = 1.0;
	const double mass = 2.951;
	const double torque = power_multiplier*0.0666; // (Per-motor) Torque output at stall, that is, at max output due to an obstacle
	const double max_linear_velocity = power_multiplier*0.26;
	const double power_wheels_friction = 1.0;

	auto& rb = sim.rigid_bodies;

	// Body
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::load_obj(vertices, triangles, kobuki_collision_path);
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
	settings.output.codegen_directory = COMPILE_PATH;
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
void folding_towel()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "folding_towel_f1.0";
	settings.output.output_directory = "D:/sciebo/wd/stark/folding_towel";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 15.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.01, 0.01);

	settings.newton.max_newton_iterations = 50;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = true;
	settings.contact.friction_stick_slide_threshold = 0.001;
	settings.contact.adaptive_contact_stiffness.value = 1e4;
	settings.contact.dhat = towel.thickness;

	stark::models::Simulation sim(settings);

	// Towel
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	const int ny = 100;
	const int nx = 56;
	stark::utils::generate_triangular_grid(vertices, triangles, {-0.5*towel.w, -0.5*towel.l}, {0.5*towel.w, 0.5*towel.l}, {nx, ny});
	const int towel_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);
	//sim.cloth.set_bending_stiffness(towel_id, 1e-6);
	sim.cloth.set_friction(towel_id, 1.0);
	

	// Run
	sim.stark.run(
		[&]()
		{
			const double t = sim.stark.current_time;
			sim.cloth.clear_vertex_target_position();
			sim.cloth.clear_acceleration();

			if (t < 3.0) {
				sim.stark.settings.simulation.gravity = {0, 0, -9.81};
				for (const int i : stark::utils::vertices_in_AABB(sim.cloth.model.x1, {-1.0, -0.001, -1.0}, {1.0, 0.001, 1.0})) {
					sim.cloth.set_vertex_target_position(towel_id, i, sim.cloth.model.x1[i]);
				}
				if (t > 2.0) {
					const Eigen::Vector3d e0 = {-1.0, 0.0, -0.3485};
					const Eigen::Vector3d e1 = {1.0, 0.0, -0.3485};
					for (const int i : stark::utils::vertices_in_AABB(sim.cloth.model.x1, { -0.4, -0.02, -0.353 }, { 0.4, 0.02, -0.344 })) {
						const Eigen::Vector3d& p = sim.cloth.model.x1[i];
						const std::array<double, 2> bary = stark::models::barycentric_point_edge(p, e0, e1);
						const Eigen::Vector3d q = bary[0]*e0 + bary[1]*e1;
						const Eigen::Vector3d u = (q - p).normalized();
						sim.cloth.set_acceleration(towel_id, i, 10.0 * u);
					}
				}
			}
			else if (t < 6.0) {
				sim.stark.settings.simulation.gravity = {0, 9.81, 0};
				for (const int i : stark::utils::vertices_in_AABB(sim.cloth.model.x1, { -0.4, -0.02, -0.353 }, { 0.4, 0.02, -0.344 })) {
					sim.cloth.set_vertex_target_position(towel_id, i, sim.cloth.model.x1[i]);
				}
				if (t > 5.0) {
					const Eigen::Vector3d e0 = { 0.0, -1.0, -0.345 };
					const Eigen::Vector3d e1 = { 0.0, 1.0, -0.345 };
					for (const int i : stark::utils::vertices_in_AABB(sim.cloth.model.x1, { -0.005, -0.04, -0.364 }, { 0.005, 0.4, -0.320 })) {
						const Eigen::Vector3d& p = sim.cloth.model.x1[i];
						const std::array<double, 2> bary = stark::models::barycentric_point_edge(p, e0, e1);
						const Eigen::Vector3d q = bary[0] * e0 + bary[1] * e1;
						const Eigen::Vector3d u = (q - p).normalized();
						sim.cloth.set_acceleration(towel_id, i, 10.0 * u);
					}
				}
			}
			else if (t < 9.0) {
				sim.stark.settings.simulation.gravity = {0, 0, 9.81};
				for (const int i : stark::utils::vertices_in_AABB(sim.cloth.model.x1, { -0.005, -0.04, -0.364 }, { 0.005, 0.4, -0.320 })) {
					sim.cloth.set_vertex_target_position(towel_id, i, sim.cloth.model.x1[i]);
				}
			}
			else {
				sim.stark.settings.simulation.gravity = { 9.81, 0, 0 };
				const int floor_id = 1;
				if (sim.cloth.get_n_cloths() == 1) {
					std::vector<Eigen::Vector3d> vertices;
					std::vector<std::array<int, 3>> triangles;
					stark::utils::generate_triangular_grid(vertices, triangles, 1.0, 1.0, 1);
					stark::utils::scale(vertices, 0.45);
					stark::utils::rotate_deg(vertices, 90, Eigen::Vector3d::UnitY());
					stark::utils::move(vertices, {0.11, 0.173, -0.157});
					sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);
					sim.cloth.set_damping(10.0, 0.5, 0.5);
					sim.cloth.set_friction(floor_id, 1.0);
				}
				sim.cloth.freeze(floor_id);
				//settings.contact.friction_enabled = true;
			}
		}
	);
}
void rolling_towel()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "rolling_towel";
	settings.output.output_directory = OUTPUT_PATH + "/rolling_towel";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 100.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.01, 0.01);

	settings.newton.max_newton_iterations = 50;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = false;
	settings.contact.friction_stick_slide_threshold = 0.001;
	settings.contact.adaptive_contact_stiffness.value = 1e4;
	settings.contact.dhat = towel.thickness;

	stark::models::Simulation sim(settings);

	// Towel
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	const int ny = 100;
	const int nx = 56;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5 * towel.w, -0.5 * towel.l }, { 0.5 * towel.w, 0.5 * towel.l }, { nx, ny });
	stark::utils::rotate_deg(vertices, 90.0, { 1, 0, 0 });
	stark::utils::move(vertices, {0.0, 0.0, -0.5*towel.l});
	const int towel_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);

	// Run
	sim.stark.run(
		[&]()
		{
			const double t = sim.stark.current_time;
			const double w = 5.0;
			sim.cloth.clear_vertex_target_position();

			for (const int i : stark::utils::vertices_in_AABB(sim.cloth.model.X, { -1.0, -1.0, -0.01 }, { 1.0, 1.0, 0.01 })) {
				sim.cloth.set_vertex_target_position_as_initial(towel_id, i);
			}
			for (const int i : stark::utils::vertices_in_AABB(sim.cloth.model.X, { -1.0, -1.0, -0.02 }, { 1.0, 1.0, -0.01 })) {
				const double x = sim.cloth.model.x1[i].x();
				const double r = std::abs(sim.cloth.model.X[i].z());
				sim.cloth.set_vertex_target_position(towel_id, i, {x, r*std::sin(w*t), -r*std::cos(w*t)});
			}
		}
	);
}
void kobuki_test()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "kobuki_test";
	settings.output.output_directory = "D:/sciebo/wd/stark/kobuki_test";
	settings.output.codegen_directory = COMPILE_PATH;
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
	make_kobuki(sim, MODELS_PATH + "/kobuki_collision.obj");

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
void kobuki_v_towel(const std::string output_directory, const std::string name, const std::string mesh_path, const std::string kobuki_collision_path, const double floor_friction, const double rotation)
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = name;
	settings.output.output_directory = output_directory;
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 10.0;
	//settings.execution.end_simulation_time = 0.02;
	settings.simulation.adaptive_time_step.set(0.0, 0.01, 0.01);

	settings.newton.max_newton_iterations = 50;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = true;
	settings.contact.friction_stick_slide_threshold = 0.001;
	settings.contact.adaptive_contact_stiffness.value = 1e4;
	settings.contact.dhat = 0.85*towel.thickness;

	stark::models::Simulation sim(settings);

	// Kobuki
	make_kobuki(sim, kobuki_collision_path);

	// Floor
	const int floor = sim.rigid_bodies.add_box(10.0, { 2, 3, 0.1 }, { 0, 1, -(0.05 + settings.contact.dhat) });
	sim.rigid_bodies.set_friction(floor, floor_friction);
	sim.rigid_bodies.add_constraint_freeze(floor);
	//sim.rigid_bodies.add_to_output_group("floor", floor);

	// Towel
	const double towel_friction = 1.0;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::load_obj(vertices, triangles, mesh_path);
	stark::utils::rotate_deg(vertices, rotation, Eigen::Vector3d::UnitZ());
	stark::utils::move(vertices, { 0.0, 0.5, 0.0 });
	const int towel_id = sim.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Towel);
	sim.cloth.set_friction(towel_id, towel_friction);

	// Run
	sim.stark.run();
}
void kobuki_v_towel_suite()
{
	std::vector<std::pair<std::string, double>> friction_types;
	friction_types.push_back({"001", 0.01});
	friction_types.push_back({"005", 0.05});
	friction_types.push_back({"010", 0.1});
	friction_types.push_back({"025", 0.25});
	friction_types.push_back({"050", 0.5});
	friction_types.push_back({"075", 0.75});
	friction_types.push_back({"100", 1.0});
	friction_types.push_back({"150", 1.5});
	friction_types.push_back({"200", 2.0});
	//std::vector<std::string> floor_types = {"carpet", "polished"};
	std::vector<std::string> folds = {"1fold", "2folds", "3fold"};

	//const std::string base = "C:/Users/jose/sciebo/paper_project_box/icra24";
	const std::string base = "D:/sciebo/paper_project_box/icra24";
	//const std::string base = "/local-hdd/jfernandez/sciebo/icra24";
	const std::string out = base + "/scenes/kobuki_v_towel/v0";
	const std::string mesh_3_folds = base + "/models/towel_3folds.obj";
	const std::string mesh_2_folds = base + "/models/towel_2folds.obj";
	const std::string mesh_1_fold = base + "/models/towel_1fold.obj";
	const std::string mesh_flat = base + "/models/towel_flat.obj";
	const std::string mesh_rolled_rolled_side = base + "/models/towel_rolled_rolledside.obj";
	const std::string mesh_rolled_open_side = base + "/models/towel_rolled_openside.obj";
	const std::string mesh_rolled_side = base + "/models/towel_rolled_side.obj";
	const std::string kobuki_collision_path = base + "/models/kobuki_collision.obj";

	for (const auto pair : friction_types) {
		const std::string label = pair.first;
		const double friction = pair.second;
		kobuki_v_towel(out + "/" + label, "flat", mesh_flat, kobuki_collision_path, friction, 0);
		kobuki_v_towel(out + "/" + label, "1fold",  mesh_1_fold, kobuki_collision_path, friction, 0);
		kobuki_v_towel(out + "/" + label, "2folds",  mesh_2_folds, kobuki_collision_path, friction, 0);

		kobuki_v_towel(out + "/" + label, "3folds_1foldside", mesh_3_folds, kobuki_collision_path, friction, 0);
		kobuki_v_towel(out + "/" + label, "3folds_2foldside", mesh_3_folds, kobuki_collision_path, friction, -90);
		kobuki_v_towel(out + "/" + label, "3folds_openside",  mesh_3_folds, kobuki_collision_path, friction, 180);

		kobuki_v_towel(out + "/" + label, "rolled_rolledside", mesh_rolled_rolled_side, kobuki_collision_path, friction, 0);
		kobuki_v_towel(out + "/" + label, "rolled_openside", mesh_rolled_open_side, kobuki_collision_path, friction, 0);
		kobuki_v_towel(out + "/" + label, "rolled_side", mesh_rolled_side, kobuki_collision_path, friction, 0);
	}
}
