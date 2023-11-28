#include "rb_constraint_test_scenes.h"

#include "paths.h"

#include <functional>

void template_sim(std::string name, std::function<void(stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)> callback)
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = name;
	settings.output.output_directory = OUTPUT_PATH + "/rb_constraints";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::Verbosity::TimeSteps;
	settings.execution.end_simulation_time = 10.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	settings.debug.symx_check_for_NaNs = true;
	//settings.newton.project_to_PD = true;
	settings.newton.use_direct_linear_solve = false;
	stark::models::Simulation simulation(settings);

	// Object
	auto box0 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 });

	// Constraint
	simulation.rigidbodies->add_constraint_fixed(box0);

	callback(simulation, box0);

	// Run
	simulation.stark.run();
}


void rb_constraints_ball_joint()
{
	template_sim("ball_joint", 
		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0) 
		{
			auto prev = box0;
			const int N = 10;
			for (int i = 1; i < N; i++) {
				auto curr = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
					.set_translation({0.1*i, 0.0, 0.0});
				if (i % 2) {
					sim.rigidbodies->add_constraint_ball_joint(prev, curr, {0.05 + (i-1)*0.1, -0.05, -0.05});
				}
				else {
					sim.rigidbodies->add_constraint_ball_joint(prev, curr, {0.05 + (i-1)*0.1, 0.05, 0.05});
				}
				prev = curr;
			}
		}
	);
}
