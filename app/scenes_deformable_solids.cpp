#include "scenes_deformable_solids.h"

#include "paths.h"

#include <Eigen/Dense>
#include <stark>


void andreas_cantilever()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "andreas_cantilever";
	settings.output.output_directory = BASE_PATH + "/andreas_cantilever";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.simulation.adaptive_time_step.set(1.0/60.0, 1.0/60.0, 1.0/60.0);
	settings.execution.end_simulation_time = 1.5;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;

	settings.newton.newton_tol = 1e-4;
	settings.newton.project_to_PD = true;
	//settings.execution.n_threads = 1;

	stark::models::Simulation simulation(settings);

	// Cantilever
	const int n = 10;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 4>> tets;
	stark::utils::generate_tet_grid(vertices, tets, { 0.0, -0.5, -0.5 }, { 2.0, 0.5, 0.5 }, { 2*n, n, n });
	const int id = simulation.deformables.add(vertices, tets);
	simulation.deformables.set_density(id, 1000.0);
	simulation.deformables.set_strain_parameters(id, 0.4e6, 0.4);

	for (int i = 0; i < (int)vertices.size(); i++) {
		if (vertices[i].x() < 1e-6) {
			simulation.deformables.set_vertex_target_position_as_initial(id, i);
		}
	}

	// Run
	simulation.stark.run();
}
