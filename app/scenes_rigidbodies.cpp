#include "scenes_rigidbodies.h"

#include <Eigen/Dense>
#include <stark>


void rb_ball_joint()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "ball_joint";
	settings.output.output_directory = "../output/rigid_body_joints";
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 10.0;
	stark::models::Simulation sim(settings);

	// Rigid bodies
	const double density = 1000.0;

	stark::utils::Mesh sphere = stark::utils::make_sphere(0.05);
	const int o1 = sim.rigid_bodies.add(sphere.vertices, sphere.triangles, density);

	stark::utils::Mesh cube = stark::utils::make_box(0.05);
	stark::utils::move(cube.vertices, {0.2, 0.0, 0.0});
	const int o2 = sim.rigid_bodies.add(cube.vertices, cube.triangles, density);

	// Constraints
	sim.rigid_bodies.add_constraint_ball_joint(o1, o2, {0, 0, 0});
	sim.rigid_bodies.add_constraint_freeze(o1);

	// Run
	sim.stark.run();
}

void rb_slider()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "slider";
	settings.output.output_directory = "../output/rigid_body_joints";
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.execution.end_simulation_time = 10.0;
	stark::models::Simulation sim(settings);

	// Rigid bodies
	const double density = 1000.0;

	stark::utils::Mesh cube = stark::utils::make_box(0.05);
	const int o1 = sim.rigid_bodies.add(cube.vertices, cube.triangles, density);

	stark::utils::Mesh cylinder = stark::utils::make_cylinder(0.025, 0.05);
	stark::utils::move(cylinder.vertices, {0, 0, -0.2});
	//stark::utils::rotate_deg(cylinder.vertices, -45.0, Eigen::Vector3d::UnitY());
	const int o2 = sim.rigid_bodies.add(cylinder.vertices, cylinder.triangles, density);

	// Constraints
	sim.rigid_bodies.add_constraint_freeze(o1);
	sim.rigid_bodies.add_constraint_slider(o1, o2, sim.rigid_bodies.t1[o1], sim.rigid_bodies.t1[o2], 1000.0, 1.0);

	// Run
	sim.stark.run();
}
