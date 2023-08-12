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
	const double mass = 1.0;
	const int o1 = sim.rigid_bodies.add_sphere(mass, 0.1);
	const int o2 = sim.rigid_bodies.add_torus(mass, 0.1, 0.03, { 0.6, 0.0, 0.0 }, 45, { 0, 1, 0 });

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
	const double mass = 0.5;
	const double scale = 0.1;
	const int o1 = sim.rigid_bodies.add_box(mass, {scale, scale, scale});
	const int o2 = sim.rigid_bodies.add_cylinder(mass, 0.5*scale, scale, {0, 0, -0.2});
	sim.rigid_bodies.add_rotation(o2, -45.0, { 0, 1, 0 });

	// Constraints
	sim.rigid_bodies.add_constraint_freeze(o1);
	sim.rigid_bodies.add_constraint_slider(o1, o2, sim.rigid_bodies.t1[o1], sim.rigid_bodies.t1[o2], 2.0, 0.0);

	// Run
	sim.stark.run();
}
