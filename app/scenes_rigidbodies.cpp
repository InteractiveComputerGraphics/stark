#include "scenes_rigidbodies.h"

#include <Eigen/Dense>
#include <stark>


void rb_ball_joint()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "rb_ball_joint";
	settings.output.output_directory = "../output/" + settings.output.simulation_name;
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 10.0;
	stark::models::Simulation sim(settings);

	// Rigid bodies
	const double density = 1000.0;
	stark::utils::Mesh sphere = stark::utils::make_sphere();
	stark::utils::scale(sphere.vertices, 0.05);
	const int s1 = sim.rigid_bodies.add(sphere.vertices, sphere.triangles, density);

	stark::utils::move(sphere.vertices, {0.2, 0.0, 0.0});
	const int s2 = sim.rigid_bodies.add(sphere.vertices, sphere.triangles, density);

	// Run
	sim.stark.run();
}
