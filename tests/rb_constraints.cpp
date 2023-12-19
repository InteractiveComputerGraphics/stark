#include <iostream>
#include <random>

#include <stark>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "../app/paths.h"

#define ENABLE_THESE_TESTS true


using namespace Catch::Matchers;

double rng() {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution<double> distr(0.0, 100.0);
	return distr(gen);
}
const double MASS = rng();
const double PERTURBATION = rng() + 10.0;  // Too small force/acceleration will take too long to reach constraint limits


#if ENABLE_THESE_TESTS
stark::Settings test_settings(std::string name)
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = name;
	settings.output.output_directory = OUTPUT_PATH + "/test_output";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.output.console_verbosity = stark::ConsoleVerbosity::NoOutput;
	settings.output.enable_output = false;
	settings.execution.end_simulation_time = 3.0;
	settings.simulation.gravity = {0, 0, 0};
	settings.contact.collisions_enabled = false;
	return settings;
}
TEST_CASE("inertia", "[rb_constraints]") 
{
	stark::Settings settings = test_settings("inertia");
	settings.simulation.gravity[0] = PERTURBATION;
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	auto constraint = simulation.rigidbodies->add_constraint_global_point(box0, box0.get_translation());
	simulation.stark.run();
	auto [C, f] = constraint.get_violation_in_m_and_force();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_m()));
	REQUIRE_THAT(f[0], WithinRel(-PERTURBATION*MASS, 1e-3));
	REQUIRE_THAT(f[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(f[2], WithinAbs(0.0, 1e-3));
}

TEST_CASE("global_point", "[rb_constraints]") 
{
	stark::Settings settings = test_settings("global_point");
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	auto constraint = simulation.rigidbodies->add_constraint_global_point(box0, box0.get_translation());
	box0.add_force_at_centroid({PERTURBATION, 0, 0});
	simulation.stark.run();
	auto [C, f] = constraint.get_violation_in_m_and_force();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_m()));
	REQUIRE_THAT(f[0], WithinRel(-PERTURBATION, 1e-3));
	REQUIRE_THAT(f[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(f[2], WithinAbs(0.0, 1e-3));
}

TEST_CASE("global_direction", "[rb_constraints]") 
{
	stark::Settings settings = test_settings("global_direction");
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	auto constraint = simulation.rigidbodies->add_constraint_global_direction(box0, Eigen::Vector3d::UnitZ());
	box0.add_torque({PERTURBATION, 0, 0});
	simulation.stark.run();
	auto [C, t] = constraint.get_violation_in_deg_and_torque();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_deg()));
	REQUIRE_THAT(t[0], WithinRel(-PERTURBATION, 1e-3));
	REQUIRE_THAT(t[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(t[2], WithinAbs(0.0, 1e-3));
}

TEST_CASE("point", "[rb_constraints]")
{
	stark::Settings settings = test_settings("point");
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	simulation.rigidbodies->add_constraint_fix(box0);
	auto box1 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 }).set_translation({0.1, 0.0, 0.0});
	auto constraint = simulation.rigidbodies->add_constraint_point(box0, box1, {0.05, 0.0, 0.0});

	box1.add_force_at_centroid({ PERTURBATION, 0, 0 });
	simulation.stark.run();
	auto [C, f] = constraint.get_violation_in_m_and_force();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_m()));
	REQUIRE_THAT(f[0], WithinRel(-PERTURBATION, 1e-3));
	REQUIRE_THAT(f[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(f[2], WithinAbs(0.0, 1e-3));
}

TEST_CASE("point_on_axis", "[rb_constraints]")
{
	stark::Settings settings = test_settings("point_on_axis");
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	simulation.rigidbodies->add_constraint_fix(box0);
	auto box1 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 }).set_translation({0.1, 0.0, 0.0});
	auto constraint = simulation.rigidbodies->add_constraint_point_on_axis(box0, box1, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ());

	box1.add_force_at_centroid({ PERTURBATION, 0, 0 });
	simulation.stark.run();
	auto [C, f] = constraint.get_violation_in_m_and_force();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_m()));
	REQUIRE_THAT(f[0], WithinRel(-PERTURBATION, 1e-3));
	REQUIRE_THAT(f[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(f[2], WithinAbs(0.0, 1e-3));
}

TEST_CASE("distance", "[rb_constraints]")
{
	stark::Settings settings = test_settings("distance");
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	simulation.rigidbodies->add_constraint_fix(box0);
	auto box1 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 }).set_translation({1.0, 0.0, 0.0});
	auto constraint = simulation.rigidbodies->add_constraint_distance(box0, box1, box0.get_translation(), box1.get_translation());

	box1.add_force_at_centroid({ PERTURBATION, 0, 0 });
	simulation.stark.run();
	auto [C, f] = constraint.get_violation_in_m_and_force();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_m()));
	REQUIRE_THAT(f[0], WithinRel(-PERTURBATION, 1e-3));
	REQUIRE_THAT(f[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(f[2], WithinAbs(0.0, 1e-3));
}

TEST_CASE("distance_limits_max", "[rb_constraints]")
{
	stark::Settings settings = test_settings("distance_limits_max");
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	simulation.rigidbodies->add_constraint_fix(box0);
	auto box1 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 }).set_translation({1.0, 0.0, 0.0});
	auto constraint = simulation.rigidbodies->add_constraint_distance_limits(box0, box1, box0.get_translation(), box1.get_translation(), 0.99, 1.01);

	box1.add_force_at_centroid({ PERTURBATION, 0, 0 });
	simulation.stark.run();
	auto [C, f] = constraint.get_violation_in_m_and_force();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_m()));
	REQUIRE_THAT(f[0], WithinRel(-PERTURBATION, 1e-3));
	REQUIRE_THAT(f[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(f[2], WithinAbs(0.0, 1e-3));
}

TEST_CASE("distance_limits_min", "[rb_constraints]")
{
	stark::Settings settings = test_settings("distance_limits_min");
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	simulation.rigidbodies->add_constraint_fix(box0);
	auto box1 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 }).set_translation({1.0, 0.0, 0.0});
	auto constraint = simulation.rigidbodies->add_constraint_distance_limits(box0, box1, box0.get_translation(), box1.get_translation(), 0.99, 1.01);

	box1.add_force_at_centroid({ -PERTURBATION, 0, 0 });
	simulation.stark.run();
	auto [C, f] = constraint.get_violation_in_m_and_force();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_m()));
	REQUIRE_THAT(f[0], WithinRel(PERTURBATION, 1e-3));
	REQUIRE_THAT(f[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(f[2], WithinAbs(0.0, 1e-3));
}

TEST_CASE("direction", "[rb_constraints]")
{
	stark::Settings settings = test_settings("direction");
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	simulation.rigidbodies->add_constraint_fix(box0);
	auto box1 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 }).set_translation({0.0, 0.0, 0.1});
	auto constraint = simulation.rigidbodies->add_constraint_direction(box0, box1, Eigen::Vector3d::UnitZ());

	box1.add_torque({ -PERTURBATION, 0, 0 });
	simulation.stark.run();
	auto [C, t] = constraint.get_violation_in_deg_and_torque();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_deg()));
	REQUIRE_THAT(t[0], WithinRel(PERTURBATION, 1e-3));
	REQUIRE_THAT(t[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(t[2], WithinAbs(0.0, 1e-3));
}

TEST_CASE("angle_limit", "[rb_constraints]")
{
	stark::Settings settings = test_settings("angle_limit");
	stark::models::Simulation simulation(settings);

	auto box0 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 });
	simulation.rigidbodies->add_constraint_fix(box0);
	auto box1 = simulation.rigidbodies->add_box(MASS, { 0.1, 0.1, 0.1 }).set_translation({0.0, 0.0, 0.1});
	auto constraint = simulation.rigidbodies->add_constraint_angle_limit(box0, box1, Eigen::Vector3d::UnitZ(), 25.0);

	box1.add_torque({ -PERTURBATION, 0, 0 });
	simulation.stark.run();
	auto [C, t] = constraint.get_violation_in_deg_and_torque();
	REQUIRE_THAT(C, WithinAbs(0.0, constraint.get_tolerance_in_deg()));
	REQUIRE_THAT(t[0], WithinRel(PERTURBATION, 1e-3));
	REQUIRE_THAT(t[1], WithinAbs(0.0, 1e-3));
	REQUIRE_THAT(t[2], WithinAbs(0.0, 1e-3));
}

#endif // ENABLE_THESE_TESTS

