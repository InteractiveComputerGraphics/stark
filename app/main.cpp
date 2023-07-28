#include <iostream>

#include <Eigen/Dense>

#include <stark>

int main()
{


	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "test";
	settings.output.output_directory = "../output/test";
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.execution.end_simulation_time = 10.0;
	//settings.execution.n_threads = 1;
	//settings.newton.use_direct_linear_solve = false;
	settings.contact.collisions_enabled = false;

	stark::models::Simulation simulation(settings);

	// Cloth
	const int n = 70;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	stark::utils::generate_triangular_grid(vertices, triangles, { -0.5, -0.5 }, { 0.5, 0.5 }, { n, n });
	
	const int cloth_id = simulation.cloth.add(vertices, triangles, stark::models::Cloth::MaterialPreset::Cotton);
	simulation.cloth.set_damping(0.2);
	//simulation.cloth.set_vertex_target_position_as_initial(cloth_id, 0);
	//simulation.cloth.set_vertex_target_position_as_initial(cloth_id, n);
	//simulation.cloth.set_vertex_target_position_as_initial(cloth_id, (n + 1) * (n + 1) - n - 1);
	//simulation.cloth.set_vertex_target_position_as_initial(cloth_id, (n + 1) * (n + 1) - 1);
	//simulation.cloth.is_strain_limiting_active = false;

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

	return 0;
}
