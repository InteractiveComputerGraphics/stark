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
	stark::utils::scale(vertices, {0.5, 0.5, 1.0});
	stark::utils::move(vertices, { 0.8, 0.0, 0.0 });
	stark::utils::rotate_deg(vertices, 10.0, Eigen::Vector3d::UnitX());
	const int small_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	//for (int i = 0; i < (int)vertices.size(); i++) {
	//	simulation.cloth.set_vertex_target_position_as_initial(small_id, i);
	//}

	// Run
	simulation.stark.run();
}
