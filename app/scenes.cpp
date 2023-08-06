#include "scenes.h"


void hanging_cloth()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_cloth";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.execution.end_simulation_time = 10.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 70;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	const int cloth_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	simulation.cloth.set_damping(0.2);

	// Run
	simulation.stark.run(
		[&]()
		{
			const double t = simulation.stark.current_time;
			const double vel = 0.05;
			if (t < 4.0) {
				simulation.cloth.set_vertex_target_position(cloth_id, 0, { vertices[0].x(), vertices[0].y() + t * vel, vertices[0].z() });
				simulation.cloth.set_vertex_target_position(cloth_id, n, { vertices[n].x(), vertices[n].y() - t * vel, vertices[n].z() });
			}
		}
	);
}
void collision_cloth_test()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "collision_cloth_test";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 2.0;
	
	settings.newton.debug_line_search_output = true;
	settings.newton.use_direct_linear_solve = false;
	settings.newton.project_to_PD = false;
	
	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = false;
	settings.contact.edge_edge_enabled = true;
	settings.contact.enable_intersection_test = true;
	settings.contact.dhat = 0.001;
	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 1;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	const int base_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	for (int i = 0; i < (int)vertices.size(); i++) {
		simulation.cloth.set_vertex_target_position_as_initial(base_id, i);
	}

	stark::utils::rotate_deg(vertices, 45.0, {1, 1, 0.25});
	stark::utils::move(vertices, {0, 0, 1.0});
	const int fall_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);

	simulation.cloth.set_damping(0.2);

	// Run
	simulation.stark.run();
}
void collision_cloth_edge_edge_tests()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "collision_cloth_edge_edge_tests";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 10.0;
	settings.execution.n_threads = 1;
	settings.simulation.gravity = { 0, 0, 0 };
	settings.simulation.boundary_conditions_stiffness = 1e2;

	settings.newton.debug_line_search_output = true;
	settings.newton.use_direct_linear_solve = false;
	settings.newton.project_to_PD = false;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = false;
	settings.contact.edge_edge_enabled = true;
	settings.contact.triangle_point_enabled = false;
	settings.contact.enable_intersection_test = true;
	settings.contact.dhat = 0.1;
	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 1;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	const int large_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	for (int i = 0; i < (int)vertices.size(); i++) {
		simulation.cloth.set_vertex_target_position_as_initial(large_id, i);
	}
	stark::utils::scale(vertices, { 0.5, 0.5, 1.0 });
	stark::utils::rotate_deg(vertices, 90.0, Eigen::Vector3d::UnitX());
	stark::utils::move(vertices, { 0.76, 0.51, -0.4 });
	const int small_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	for (int i = 0; i < (int)vertices.size(); i++) {
		simulation.cloth.set_vertex_target_position_as_initial(small_id, i);
	}

	// Run
	simulation.stark.run(
		[&]()
		{
			const double t = simulation.stark.current_time;
			const double vel = 0.1;
			for (int i = 0; i < (int)vertices.size(); i++) {
				simulation.cloth.set_vertex_target_position(small_id, i, vertices[i] + Eigen::Vector3d::UnitZ()*vel*t);
			}
		}
	);
}
void collision_cloth_parallel_edge_test_rotation()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "collision_cloth_parallel_edge_test";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 2.0;
	settings.execution.n_threads = 1;
	settings.simulation.gravity = {0, 0, 0};

	settings.newton.debug_line_search_output = true;
	settings.newton.use_direct_linear_solve = false;
	settings.newton.project_to_PD = false;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = false;
	settings.contact.edge_edge_enabled = true;
	settings.contact.triangle_point_enabled = false;
	settings.contact.enable_intersection_test = true;
	settings.contact.dhat = 0.1;
	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 1;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	const int large_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	stark::utils::scale(vertices, {0.5, 0.5, 1.0});
	stark::utils::move(vertices, { 0.8, 0.0, 0.0 });
	//stark::utils::rotate_deg(vertices, 5.0, Eigen::Vector3d::UnitX());
	const int small_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);

	// Run
	simulation.stark.run(
		[&]()
		{
			const double t = simulation.stark.current_time;
			const double v = 0.02;

			const auto& mesh = simulation.cloth.model.mesh;
			simulation.cloth.clear_vertex_target_position();
			for (int i = 0; i < (int)vertices.size(); i++) {
				simulation.cloth.set_vertex_target_position(large_id, i, mesh.vertices[i]);
			}
			for (int i = 0; i < (int)vertices.size(); i++) {
				const Eigen::Vector3d& p = mesh.vertices[mesh.get_global_vertex_idx(small_id, i)];
				//if (p.x() > 1.0) {
					if (p.y() < 0.0) {
						simulation.cloth.set_vertex_target_position(small_id, i, { p.x(), p.y(), p.z() + v * t });
					}
					else {
						simulation.cloth.set_vertex_target_position(small_id, i, { p.x(), p.y(), p.z() - v * t });
					}
				//}
			}
		}
	);
}
void collision_cloth_parallel_edge_test_shear()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "collision_cloth_parallel_edge_test";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 2.0;
	settings.execution.n_threads = 1;
	settings.simulation.gravity = { 0, 0, 0 };

	settings.newton.debug_line_search_output = true;
	settings.newton.use_direct_linear_solve = false;
	settings.newton.project_to_PD = false;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = false;
	settings.contact.edge_edge_enabled = true;
	settings.contact.triangle_point_enabled = false;
	settings.contact.enable_intersection_test = true;
	settings.contact.dhat = 0.1;
	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 1;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	const int large_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	stark::utils::rotate_deg(vertices, 5.0, Eigen::Vector3d::UnitZ());
	stark::utils::scale(vertices, { 0.5, 0.5, 1.0 });
	stark::utils::move(vertices, { 0.8, 0.0, 0.0 });
	const int small_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);

	// Run
	simulation.stark.run(
		[&]()
		{
			const double t = simulation.stark.current_time;
			const double v = 0.02;

			const auto& mesh = simulation.cloth.model.mesh;
			simulation.cloth.clear_vertex_target_position();
			for (int i = 0; i < (int)vertices.size(); i++) {
				simulation.cloth.set_vertex_target_position(large_id, i, mesh.vertices[i]);
			}
			for (int i = 0; i < (int)vertices.size(); i++) {
				const Eigen::Vector3d& p = mesh.vertices[mesh.get_global_vertex_idx(small_id, i)];
				//if (p.x() > 1.0) {
				if (p.y() < 0.0) {
					simulation.cloth.set_vertex_target_position(small_id, i, { p.x() - v * t, p.y(), p.z() });
				}
				else {
					simulation.cloth.set_vertex_target_position(small_id, i, { p.x() + v * t, p.y(), p.z() });
				}
				//}
			}
		}
	);
}
void collision_cloth_parallel_edge_test_slide()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "collision_cloth_parallel_edge_test";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 15.0;
	settings.execution.n_threads = 1;
	settings.simulation.gravity = { 0, 0, 0 };

	settings.newton.debug_line_search_output = true;
	settings.newton.use_direct_linear_solve = false;
	settings.newton.project_to_PD = false;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = false;
	settings.contact.edge_edge_enabled = true;
	settings.contact.triangle_point_enabled = false;
	settings.contact.enable_intersection_test = true;
	settings.contact.dhat = 0.1;
	stark::models::Simulation simulation(settings);

	const bool shear = true;
	const bool rotate = true;

	// Cloth
	const int n = 1;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	const int large_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);

	if (shear) { stark::utils::rotate_deg(vertices, 5.0, Eigen::Vector3d::UnitZ()); }
	if (rotate) { stark::utils::rotate_deg(vertices, 10.0, Eigen::Vector3d::UnitX()); }
	stark::utils::scale(vertices, { 0.5, 0.5, 1.0 });
	stark::utils::move(vertices, { 0.8, -0.8, 0.0 });
	const int small_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);

	// Run
	simulation.stark.run(
		[&]()
		{
			const double t = simulation.stark.current_time;
			const double v = 0.1;
			const double vr = 0.01;
			const double vs = 0.0025;

			const auto& mesh = simulation.cloth.model.mesh;
			simulation.cloth.clear_vertex_target_position();
			for (int i = 0; i < (int)vertices.size(); i++) {
				simulation.cloth.set_vertex_target_position(large_id, i, mesh.vertices[i]);
			}
			for (int i = 0; i < (int)vertices.size(); i++) {
				Eigen::Vector3d p = mesh.vertices[mesh.get_global_vertex_idx(small_id, i)];
				const Eigen::Vector3d p_ = mesh.vertices[mesh.get_global_vertex_idx(large_id, i)];

				// Translation
				p.y() += v * t;

				// Shear
				if (shear) {
					if (p_.y() < 0.0) {
						p.x() -= vs * t;
					}
					else {
						p.x() += vs * t;
					}
				}

				// Rotation
				if (rotate) {
					if (p_.y() < 0.0) {
						p.z() += vr * t;
					}
					else {
						p.z() -= vr * t;
					}
				}
				simulation.cloth.set_vertex_target_position(small_id, i, p);
			}
		}
	);
}
void cloth_friction_slope_test()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "cloth_friction_slope_test";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 5.0;
	//settings.execution.n_threads = 1;

	settings.newton.debug_line_search_output = true;
	settings.newton.use_direct_linear_solve = false;
	settings.newton.project_to_PD = false;
	settings.newton.max_newton_iterations = 200;

	settings.contact.collisions_enabled = true;
	settings.contact.triangle_point_enabled = true;
	settings.contact.edge_edge_enabled = true;
	settings.contact.enable_intersection_test = true;
	settings.contact.friction_enabled = true;
	settings.contact.friction_stick_slide_threshold = 1.0;
	settings.contact.dhat = 0.01;
	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 2;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	stark::utils::rotate_deg(vertices, 30.0, Eigen::Vector3d::UnitX());
	const int large_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	for (int i = 0; i < (int)vertices.size(); i++) {
		simulation.cloth.set_vertex_target_position_as_initial(large_id, i);
	}

	stark::utils::scale(vertices, { 0.5, 0.5, 0.4 });
	stark::utils::move(vertices, { 0.01, 0.02, 0.2 });
	stark::utils::rotate_deg(vertices, 30.0, Eigen::Vector3d::UnitZ());
	const int small_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	//simulation.cloth.set_vertex_target_position_as_initial(small_id, 0);

	const double mu = 0.006;
	simulation.cloth.set_friction(large_id, mu);
	simulation.cloth.set_friction(small_id, mu);

	// Run
	simulation.stark.run();
}
void cloth_wrap()
{
	// Simulation
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "wrap";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.execution.end_simulation_time = 25.0;
	//settings.execution.n_threads = 1;
	settings.simulation.gravity = { 0, 0, 0 };

	settings.newton.debug_line_search_output = false;
	settings.newton.use_direct_linear_solve = false;
	settings.newton.project_to_PD = false;

	settings.contact.collisions_enabled = true;
	settings.contact.friction_enabled = false;
	settings.contact.edge_edge_enabled = true;
	settings.contact.triangle_point_enabled = true;
	settings.contact.enable_intersection_test = true;
	settings.contact.dhat = 0.001;
	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 10;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n }, true);
	const int cloth_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	simulation.cloth.is_strain_limiting_active = false;
	simulation.cloth.set_damping(1.0);

	// Run
	simulation.stark.run(
		[&]()
		{
			const double t = simulation.stark.current_time;
			const double w = 2.0 * 3.1416 / 8.0;  // One turn every 8 seconds
			const int n_turns = 3;

			const double angle_rad = w * t;
			const double max_angle_rad = 2.0*3.1416*(double)n_turns;

			if (t < 18.0) {
				simulation.cloth.clear_vertex_target_position();
				for (int i = 0; i < (int)vertices.size(); i++) {
					if (vertices[i].x() < -0.49999) {
						const double r = vertices[i].y();
						simulation.cloth.set_vertex_target_position(cloth_id, i, { vertices[i].x(), r * std::cos(w * t), r * std::sin(w * t) });
					}
				}
				for (int i = 0; i < (int)vertices.size(); i++) {
					if (vertices[i].x() > 0.49999) {
						const double r = vertices[i].y();
						simulation.cloth.set_vertex_target_position(cloth_id, i, { vertices[i].x(), r * std::cos(-w * t), r * std::sin(-w * t) });
					}
				}
			}
		}
	);
}
