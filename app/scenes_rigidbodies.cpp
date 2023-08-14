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
	
	settings.execution.end_simulation_time = 5.0;
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
void rb_contacts_floor_test()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "contacts_floor_test";
	settings.output.output_directory = "../output/rigid_body_contacts";
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::NewtonIterations;
	settings.output.fps = 120;

	settings.execution.end_simulation_time = 1.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.001, 0.001);

	settings.contact.friction_enabled = true;
	settings.contact.adaptive_contact_stiffness.value = 1e8;
	settings.contact.dhat = 0.001;
	stark::models::Simulation sim(settings);
 
	// Slope
	const double drop_height = 0.01;
	const double rot_deg = 45.0;

	// Rigid bodies
	const double mu = 1.0;
	const double mass = 1.0;
	const double scale = 0.1;
	const int o1 = sim.rigid_bodies.add_box(mass, { 1.0, 1.0, scale }, { 0, 0, 0 });
	const int o2 = sim.rigid_bodies.add_box(mass, {scale, scale, scale}, { 0.1, -0.2, scale + drop_height + 1.5*settings.contact.dhat });
	sim.rigid_bodies.add_rotation(o1, -rot_deg, Eigen::Vector3d::UnitX());
	sim.rigid_bodies.add_rotation(o2, -rot_deg, Eigen::Vector3d::UnitX());
	sim.rigid_bodies.set_friction(o1, mu);
	sim.rigid_bodies.set_friction(o2, mu);

	// Constraints
	sim.rigid_bodies.add_constraint_freeze(o1);

	// Run
	sim.stark.run();
}
void laundry()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "laundry";
	settings.output.output_directory = "../output/laundry";
	settings.output.codegen_directory = "../output/codegen";
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.output.fps = 30;

	settings.execution.end_simulation_time = 20.0;
	settings.simulation.adaptive_time_step.set(0.0, 0.001, 0.001);
	settings.newton.max_newton_iterations = 200;

	settings.contact.adaptive_contact_stiffness.value = 1e8;
	settings.contact.dhat = 0.002;
	settings.contact.edge_edge_enabled = false;
	stark::models::Simulation sim(settings);

	sim.rigid_bodies.set_damping(0.25);

	// Wall
	const int wall = sim.rigid_bodies.add_box(100.0, { 2.0, 0.5, 2.0 }, {0.0, -0.6, 0.0});
	sim.rigid_bodies.add_constraint_freeze(wall);

	// Drum
	const double torque = 15.0;
	const int drum = sim.rigid_bodies.add_cylinder(1.0, 0.75, 0.5, {0, 0, 0}, 90.0, {1, 0, 0}, 64);
	sim.rigid_bodies.add_constraint_motor(wall, drum, {0, 0, 0}, Eigen::Vector3d::UnitY(), 50.0, 3.14);

	// Objects
	const double mu = 1.0;
	const double mass = 0.1;
	const double scale = 0.1;

	const double d = 1.5*scale;
	for (double x = -0.4; x < 0.4; x += d) {
		for (double y = -0.15; y < 0.16; y += d) {
			for (double z = -0.4; z < -0.3; z += d) {
				const int idx = sim.rigid_bodies.add_sphere(mass, 0.5 * scale, {x, y, z});
				sim.rigid_bodies.set_friction(idx, mu);
			}
		}
	}
	 
	//const int idx = sim.rigid_bodies.add_sphere(mass, 0.5 * scale, { 0.0, 0.0, -0.6 });
	//sim.rigid_bodies.set_friction(idx, mu);

	// Run
	sim.stark.run();
}
