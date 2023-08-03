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
	settings.contact.edge_edge_enabled = true;
	settings.contact.triangle_point_enabled = false;
	settings.contact.enable_intersection_test = true;
	settings.contact.dhat = 0.1;
	settings.contact.edge_edge_cross_norm_sq_threshold = 1e-6;
	settings.contact.edge_edge_cross_norm_sq_cutoff = 1e-20;
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
void collision_cloth_parallel_edge_test()
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
	settings.contact.edge_edge_enabled = true;
	settings.contact.triangle_point_enabled = false;
	settings.contact.enable_intersection_test = true;
	settings.contact.dhat = 0.1;
	settings.contact.edge_edge_cross_norm_sq_threshold = 1e-12;
	settings.contact.edge_edge_cross_norm_sq_cutoff = 1e-32;
	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 1;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	const int large_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	stark::utils::scale(vertices, {0.5, 0.5, 1.0});
	stark::utils::move(vertices, { 0.8, 0.0, 0.0 });
	stark::utils::rotate_deg(vertices, 5.0, Eigen::Vector3d::UnitX());
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
void cloth_wrap()
{
	// Simulation
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "wrap";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 20.0;
	//settings.execution.n_threads = 1;
	settings.simulation.gravity = { 0, 0, 0 };

	settings.newton.debug_line_search_output = true;
	settings.newton.use_direct_linear_solve = false;
	settings.newton.project_to_PD = false;

	settings.contact.collisions_enabled = true;
	settings.contact.edge_edge_enabled = true;
	settings.contact.triangle_point_enabled = true;
	settings.contact.enable_intersection_test = true;
	settings.contact.dhat = 0.01;
	settings.contact.edge_edge_cross_norm_sq_cutoff = 1e-32;
	settings.contact.edge_edge_cross_norm_sq_threshold = 1e-6;
	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 10;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	const int cloth_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	simulation.cloth.is_strain_limiting_active = false;
	simulation.cloth.set_damping(1.0);

	// Run
	simulation.stark.run(
		[&]()
		{
			const double t = simulation.stark.current_time;
			const double w = 0.5;

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
	);
}
