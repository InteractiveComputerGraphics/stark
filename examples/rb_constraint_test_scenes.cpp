#include "rb_constraint_test_scenes.h"
#include <functional>

#include <stark>
#include "paths.h"

stark::RigidBodyHandler make_box(stark::Simulation& simulation)
{
	const double mass = 1.0;
	const double size = 0.1;
	auto [vertices, triangles, box] = simulation.presets->rigidbodies->add_box("box", mass, size);
	return box.rigidbody;
}

void template_sim(std::string name, std::function<void(stark::Simulation& simulation, stark::RigidBodyHandler& box0)> callback)
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = name;
	settings.output.output_directory = OUTPUT_PATH + "/rb_constraints";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::Frames;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Object
	auto box0 = make_box(simulation);

	// Constraint
	simulation.rigidbodies->add_constraint_fix(box0);

	// Fill the rest of the sim
	callback(simulation, box0);

	// Run
	simulation.run();
}

void rb_constraints_point()
{
	template_sim("ball_joint", 
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0) 
		{
			auto prev = box0;
			const int N = 10;
			for (int i = 1; i < N; i++) {
				auto curr = make_box(simulation);
				curr.set_translation({0.1*i, 0.0, 0.0});
				if (i % 2) {
					simulation.rigidbodies->add_constraint_point(prev, curr, {0.05 + (i-1)*0.1, -0.05, -0.05});
				}
				else {
					simulation.rigidbodies->add_constraint_point(prev, curr, {0.05 + (i-1)*0.1, 0.05, 0.05});
				}
				prev = curr;
			}
		}
	);
}
void rb_constraints_point_on_axis()
{
	template_sim("point_on_axis",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto prev = box0;
			const int N = 10;
			for (int i = 1; i < N; i++) {
				auto curr = make_box(simulation);
				curr.set_translation({ 0.1 * i, 0.0, 0.0 });
				if (i % 2) {
					simulation.rigidbodies->add_constraint_point(prev, curr, { 0.05 + (i - 1) * 0.1, -0.05, -0.05 });
				}
				else {
					simulation.rigidbodies->add_constraint_point(prev, curr, { 0.05 + (i - 1) * 0.1, 0.05, 0.05 });
				}
				prev = curr;
			}

			simulation.rigidbodies->add_constraint_point_on_axis(box0, prev, prev.get_translation(), Eigen::Vector3d::UnitX());
		}
	);
}
void rb_constraints_direction()
{
	template_sim("relative_direction_lock", 
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0) 
		{
			auto prev = box0;
			const int N = 10;
			for (int i = 1; i < N; i++) {
				auto curr = make_box(simulation);
				curr.set_translation({0.1*i, 0.0, 0.0});
				if (i % 2) {
					simulation.rigidbodies->add_constraint_point(prev, curr, {0.05 + (i-1)*0.1, -0.05, -0.05});
				}
				else {
					simulation.rigidbodies->add_constraint_point(prev, curr, {0.05 + (i-1)*0.1, 0.05, 0.05});
				}
				if (i != 1) {
					simulation.rigidbodies->add_constraint_direction(prev, curr, Eigen::Vector3d::UnitY());
				}
				prev = curr;
			}
		}
	);
}
void rb_constraints_distance_limits()
{
	template_sim("distance_limits",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.5, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_distance_limits(box0, box1, box0.get_translation(), box1.get_translation() - 0.05*Eigen::Vector3d::Ones(), 0.44, 0.8);
		}
	);
}
void rb_constraints_distance()
{
	template_sim("distance",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({0.5, 0.0, 0.0});
			simulation.rigidbodies->add_constraint_distance(box0, box1, box0.get_translation(), box1.get_translation() - 0.05*Eigen::Vector3d::Ones());
		}
	);
}
void rb_constraints_angle_limit()
{
	template_sim("angle_limits",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.5, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_distance_limits(box0, box1, box0.get_translation(), box1.get_translation() - 0.05*Eigen::Vector3d::Ones(), 0.44, 0.8);
			simulation.rigidbodies->add_constraint_angle_limit(box0, box1, Eigen::Vector3d::UnitZ(), 20.0);
		}
	);
}
void rb_constraints_spring()
{
	template_sim("spring",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.5, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_spring(box0, box1, box0.get_translation(), box1.get_translation() - 0.05*Eigen::Vector3d::Ones(), 5.0, 1.0);
		}
	);
}

void rb_constraints_attachment()
{
	template_sim("attachment",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.1, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_attachment(box0, box1);
		}
	);
}
void rb_constraints_point_with_angle_limit()
{
	template_sim("point_with_angle_limit",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.1, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_point_with_angle_limit(box0, box1, {0.05, 0.0, 0.0}, Eigen::Vector3d::UnitX(), 30.0);
		}
	);
}
void rb_constraints_hinge()
{
	template_sim("hinge",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.1, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_hinge(box0, box1, { 0.05, 0.0, -0.05 }, Eigen::Vector3d::UnitY());
		}
	);
}
void rb_constraints_hinge_with_limits()
{
	template_sim("hinge_with_limits",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.1, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_hinge_with_angle_limit(box0, box1, { 0.05, 0.0, -0.05 }, Eigen::Vector3d::UnitY(), 30);
		}
	);
}
void rb_constraints_spring_with_limits()
{
	template_sim("spring_with_limits",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.1, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_spring_with_limits(box0, box1, box0.get_translation(), box1.get_translation(), 20.0, 0.09, 0.8);
		}
	);
}
void rb_constraints_slider()
{
	template_sim("slider",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.1, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_spring(box0, box1, box0.get_translation(), box1.get_translation(), 0.2);
			simulation.rigidbodies->add_constraint_slider(box0, box1, { 0.05, -0.05, 0.05 }, { 1, 0, -1 });
		}
	);
}
void rb_constraints_prismatic_slider()
{
	template_sim("prismatic_slider",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.1, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_spring(box0, box1, box0.get_translation(), box1.get_translation(), 0.2);
			simulation.rigidbodies->add_constraint_prismatic_slider(box0, box1, { 0.05, -0.05, 0.05 }, { 1, 0, -1 });
		}
	);
}
void rb_constraints_prismatic_press()
{
	template_sim("prismatic_press",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.0, 0.0, -0.5 });
			simulation.rigidbodies->add_constraint_prismatic_press(box0, box1, box0.get_translation(), Eigen::Vector3d::UnitZ(), 0.2, 20.0);
			simulation.rigidbodies->add_constraint_spring(box0, box1, box0.get_translation(), box1.get_translation(), 8.0, 0.5);
		}
	);
}
void rb_constraints_motor()
{
	template_sim("motor",
		[](stark::Simulation& simulation, stark::RigidBodyHandler& box0)
		{
			auto box1 = make_box(simulation);
			box1.set_translation({ 0.2, 0.0, 0.0 });
			simulation.rigidbodies->add_constraint_motor(box0, box1, box0.get_translation(), Eigen::Vector3d::UnitY(), -1.0, 10.0);
		}
	);
}

void rb_constraints_all()
{
	rb_constraints_point();
	rb_constraints_point_on_axis();
	rb_constraints_distance();
	rb_constraints_distance_limits();
	rb_constraints_direction();
	rb_constraints_angle_limit();
	rb_constraints_spring();

	rb_constraints_attachment();
	rb_constraints_point_with_angle_limit();
	rb_constraints_hinge();
	rb_constraints_hinge_with_limits();
	rb_constraints_spring_with_limits();
	rb_constraints_slider();
	rb_constraints_prismatic_slider();
	rb_constraints_prismatic_press();
	rb_constraints_motor();
}
