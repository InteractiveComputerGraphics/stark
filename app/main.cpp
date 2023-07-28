#include <iostream>

#include <Eigen/Dense>

#include <stark>

int main()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "test";
	settings.output.output_directory = "../output/test";
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 1.0;
	//settings.execution.n_threads = 16;
	settings.newton.use_direct_linear_solve = true;
	settings.contact.collisions_enabled = false;

	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 1;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	
	const int cloth_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	simulation.cloth.set_vertex_target_position_as_initial(cloth_id, 0);
	simulation.cloth.set_vertex_target_position_as_initial(cloth_id, n);
	//simulation.cloth.set_vertex_target_position_as_initial(cloth_id, (n + 1) * (n + 1) - n - 1);
	//simulation.cloth.set_vertex_target_position_as_initial(cloth_id, (n + 1) * (n + 1) - 1);

	simulation.stark.run();

	return 0;
}
