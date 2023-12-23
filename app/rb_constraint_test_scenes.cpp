#include "rb_constraint_test_scenes.h"

#include "paths.h"

#include <functional>

void template_sim(std::string name, std::function<void(stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)> callback)
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = name;
	settings.output.output_directory = OUTPUT_PATH + "/rb_constraints";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::TimeSteps;
	settings.execution.end_simulation_time = 5.0;
	settings.contact.collisions_enabled = false;
	settings.contact.friction_enabled = false;
	//settings.debug.symx_check_for_NaNs = true;
	//settings.newton.project_to_PD = true;
	//settings.newton.use_direct_linear_solve = true;
	settings.debug.symx_force_compilation = false;
	stark::models::Simulation simulation(settings);

	// Object
	auto box0 = simulation.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 });

	// Constraint
	simulation.rigidbodies->add_constraint_global_point(box0, box0.get_translation());
	simulation.rigidbodies->add_constraint_global_direction(box0, Eigen::Vector3d::UnitZ());
	simulation.rigidbodies->add_constraint_global_direction(box0, Eigen::Vector3d::UnitY());

	// Fill the rest of the sim
	callback(simulation, box0);

	// Run
	simulation.stark.run();
}

void rb_constraints_point()
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
					sim.rigidbodies->add_constraint_point(prev, curr, {0.05 + (i-1)*0.1, -0.05, -0.05});
				}
				else {
					sim.rigidbodies->add_constraint_point(prev, curr, {0.05 + (i-1)*0.1, 0.05, 0.05});
				}
				prev = curr;
			}
		}
	);
}
void rb_constraints_relative_direction_lock()
{
	template_sim("relative_direction_lock", 
		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0) 
		{
			auto prev = box0;
			const int N = 10;
			for (int i = 1; i < N; i++) {
				auto curr = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
					.set_translation({0.1*i, 0.0, 0.0});
				if (i % 2) {
					sim.rigidbodies->add_constraint_point(prev, curr, {0.05 + (i-1)*0.1, -0.05, -0.05});
				}
				else {
					sim.rigidbodies->add_constraint_point(prev, curr, {0.05 + (i-1)*0.1, 0.05, 0.05});
				}
				if (i != 1) {
					sim.rigidbodies->add_constraint_direction(prev, curr, Eigen::Vector3d::UnitY());
				}
				prev = curr;
			}
		}
	);
}
void rb_constraints_point_on_axis()
{
	template_sim("point_on_axis",
		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
		{
			auto prev = box0;
			const int N = 10;
			for (int i = 1; i < N; i++) {
				auto curr = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
					.set_translation({ 0.1 * i, 0.0, 0.0 });
				if (i % 2) {
					sim.rigidbodies->add_constraint_point(prev, curr, { 0.05 + (i - 1) * 0.1, -0.05, -0.05 });
				}
				else {
					sim.rigidbodies->add_constraint_point(prev, curr, { 0.05 + (i - 1) * 0.1, 0.05, 0.05 });
				}
				prev = curr;
			}

			sim.rigidbodies->add_constraint_point_on_axis(box0, prev, prev.get_translation(), Eigen::Vector3d::UnitX());
		}
	);
}
void rb_constraints_distance_limits()
{
	template_sim("distance_limits",
		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
		{
			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
				.set_translation({ 0.5, 0.0, 0.0 });
			sim.rigidbodies->add_constraint_distance_limits(box0, box1, box0.get_translation(), box1.get_translation() - 0.05*Eigen::Vector3d::Ones(), 0.44, 0.8);
		}
	);
}
void rb_constraints_angle_limits()
{
	template_sim("angle_limits",
		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
		{
			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
				.set_translation({ 0.5, 0.0, 0.0 });
			sim.rigidbodies->add_constraint_distance_limits(box0, box1, box0.get_translation(), box1.get_translation() - 0.05*Eigen::Vector3d::Ones(), 0.44, 0.8);
			sim.rigidbodies->add_constraint_angle_limit(box0, box1, Eigen::Vector3d::UnitZ(), 20.0);
		}
	);
}
//void rb_constraints_spring()
//{
//	template_sim("spring",
//		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
//		{
//			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
//				.set_translation({ 0.5, 0.0, 0.0 });
//			sim.rigidbodies->add_constraint_spring(box0, box1, box0.get_translation(), box1.get_translation() - 0.05*Eigen::Vector3d::Ones(), 5.0, 1.0);
//		}
//	);
//}
//void rb_constraints_hinge()
//{
//	template_sim("hinge",
//		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
//		{
//			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
//				.set_translation({ 0.1, 0.0, 0.0 });
//			sim.rigidbodies->add_constraint_hinge(box0, box1, { 0.05, 0.0, -0.05 }, Eigen::Vector3d::UnitY());
//		}
//	);
//}
//void rb_constraints_hinge_with_limits()
//{
//	template_sim("hinge_with_limits",
//		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
//		{
//			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
//				.set_translation({ 0.1, 0.0, 0.0 });
//			sim.rigidbodies->add_constraint_hinge_with_angle_limit(box0, box1, { 0.05, 0.0, -0.05 }, Eigen::Vector3d::UnitY(), 30);
//		}
//	);
//}
//void rb_constraints_slider()
//{
//	template_sim("slider",
//		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
//		{
//			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
//				.set_translation({ 0.1, 0.0, 0.0 });
//			sim.rigidbodies->add_constraint_spring(box0, box1, box0.get_translation(), box1.get_translation(), 0.2);
//			sim.rigidbodies->add_constraint_slider(box0, box1, { 0.05, -0.05, 0.05 }, { 1, 0, -1 });
//		}
//	);
//}
//void rb_constraints_prismatic_slider()
//{
//	template_sim("prismatic_slider",
//		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
//		{
//			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
//				.set_translation({ 0.1, 0.0, 0.0 });
//			sim.rigidbodies->add_constraint_spring(box0, box1, box0.get_translation(), box1.get_translation(), 0.2);
//			sim.rigidbodies->add_constraint_prismatic_slider(box0, box1, { 0.05, -0.05, 0.05 }, { 1, 0, -1 });
//		}
//	);
//}
//void rb_constraints_spring_with_limits()
//{
//	template_sim("spring_with_limits",
//		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
//		{
//			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
//				.set_translation({ 0.1, 0.0, 0.0 });
//			sim.rigidbodies->add_spring_with_limits(box0, box1, box0.get_translation(), box1.get_translation(), 0.2, 0.09, 0.8);
//		}
//	);
//}
//void rb_constraints_prismatic_press()
//{
//	template_sim("prismatic_press",
//		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
//		{
//			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
//				.set_translation({ 0.0, 0.0, -0.5 });
//			sim.rigidbodies->add_prismatic_press(box0, box1, box0.get_translation(), Eigen::Vector3d::UnitZ(), 0.2, 20.0);
//			sim.rigidbodies->add_constraint_spring(box0, box1, box0.get_translation(), box1.get_translation(), 8.0, 0.5);
//		}
//	);
//}
//void rb_constraints_motor()
//{
//	template_sim("motor",
//		[](stark::models::Simulation& sim, stark::models::RigidBodyHandler& box0)
//		{
//			auto box1 = sim.rigidbodies->add_box(1.0, { 0.1, 0.1, 0.1 })
//				.set_translation({ 0.2, 0.0, 0.0 });
//			sim.rigidbodies->add_motor(box0, box1, box0.get_translation(), Eigen::Vector3d::UnitY(), -1.0, 10.0);
//		}
//	);
//}

void rb_constraints_all()
{
	rb_constraints_point();
	rb_constraints_relative_direction_lock();
	rb_constraints_point_on_axis();
	rb_constraints_distance_limits();
	rb_constraints_angle_limits();
	//rb_constraints_spring();
	//rb_constraints_hinge();
	//rb_constraints_hinge_with_limits();
	//rb_constraints_slider();
	//rb_constraints_prismatic_slider();
	//rb_constraints_spring_with_limits();
	//rb_constraints_prismatic_press();
	//rb_constraints_motor();
}
