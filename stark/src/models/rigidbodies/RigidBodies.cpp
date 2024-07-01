#include "RigidBodies.h"
#include <functional>


stark::RigidBodies::RigidBodies(core::Stark& stark, spRigidBodyDynamics rb)
	: rb(rb), output(stark, rb)
{
	this->inertia = std::make_shared<EnergyRigidBodyInertia>(stark, rb);
	this->constraints = std::make_shared<EnergyRigidBodyConstraints>(stark, rb);
}

void stark::RigidBodies::set_default_constraint_stiffness(double stiffness)
{
	this->default_stiffness = stiffness;
}
void stark::RigidBodies::set_default_constraint_distance_tolerance(double tolerance_in_m)
{
	this->default_tolerance_in_m = tolerance_in_m;
}
void stark::RigidBodies::set_default_constraint_angle_tolerance(double tolerance_in_deg)
{
	this->default_tolerance_in_deg = tolerance_in_deg;
}
double stark::RigidBodies::get_default_constraint_stiffness() const
{
	return this->default_stiffness;
}
double stark::RigidBodies::get_default_constraint_distance_tolerance() const
{
	return this->default_tolerance_in_m;
}
double stark::RigidBodies::get_default_constraint_angle_tolerance() const
{
	return this->default_tolerance_in_deg;
}
stark::RigidBodyHandler stark::RigidBodies::add(const double mass, const Eigen::Matrix3d& inertia_local)
{
	const int rb_idx = this->rb->add();
	this->inertia->add(rb_idx, mass, inertia_local, 0.0, 0.0);
	return RigidBodyHandler(this->rb.get(), this->inertia.get(), rb_idx);
}

stark::RBCGlobalPointHandler stark::RigidBodies::add_constraint_global_point(const RigidBodyHandler& body, const Eigen::Vector3d& p_glob)
{
	body.exit_if_not_valid("RigidBodies::add_constraint_global_point");
	const int idx = this->constraints->global_points->add(
		body.get_idx(), 
		body.transform_global_to_local_point(p_glob),
		p_glob, 
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCGlobalPointHandler(body, this->constraints->global_points, idx);
}
stark::RBCGlobalDirectionHandler stark::RigidBodies::add_constraint_global_direction(const RigidBodyHandler& body, const Eigen::Vector3d& d_glob)
{
	body.exit_if_not_valid("RigidBodies::add_constraint_global_direction");
	const int idx = this->constraints->global_directions->add(
		body.get_idx(),
		body.transform_global_to_local_direction(d_glob),
		d_glob,
		this->default_stiffness,
		this->default_tolerance_in_deg
	);
	return RBCGlobalDirectionHandler(body, this->constraints->global_directions, idx);
}
stark::RBCPointHandler stark::RigidBodies::add_constraint_point(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_point");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_point");
	const int idx = this->constraints->points->add(
		body_a.get_idx(),
		body_b.get_idx(),
		body_a.transform_global_to_local_point(p_glob),
		body_b.transform_global_to_local_point(p_glob),
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCPointHandler(body_a, body_b, this->constraints->points, idx);
}
stark::RBCPointOnAxisHandler stark::RigidBodies::add_constraint_point_on_axis(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_point_on_axis");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_point_on_axis");
	const int idx = this->constraints->point_on_axes->add(
		body_a.get_idx(),
		body_b.get_idx(),
		body_a.transform_global_to_local_point(p_glob),
		body_a.transform_global_to_local_direction(d_glob),
		body_b.transform_global_to_local_point(p_glob),
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCPointOnAxisHandler(body_a, body_b, this->constraints->point_on_axes, idx);
}
stark::RBCDistanceHandler stark::RigidBodies::add_constraint_distance(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_distance");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_distance");
	const int idx = this->constraints->distances->add(
		body_a.get_idx(),
		body_b.get_idx(),
		body_a.transform_global_to_local_point(a_glob),
		body_b.transform_global_to_local_point(b_glob),
		(a_glob - b_glob).norm(),
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCDistanceHandler(body_a, body_b, this->constraints->distances, idx);
}
stark::RBCDistanceLimitHandler stark::RigidBodies::add_constraint_distance_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double min_distance, double max_distance)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_distance_limits");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_distance_limits");

	const double d = (a_glob - b_glob).norm();
	if (d < min_distance || max_distance < d) {
		std::cout << "stark error: RigidBodies::add_constraint_distance_limits() got a rest distance out of limits for bodies " << body_a.get_label() << " and " << body_b.get_label() << std::endl;
		exit(-1);
	}

	const int idx = this->constraints->distance_limits->add(
		body_a.get_idx(),
		body_b.get_idx(),
		body_a.transform_global_to_local_point(a_glob),
		body_b.transform_global_to_local_point(b_glob),
		min_distance,
		max_distance,
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCDistanceLimitHandler(body_a, body_b, this->constraints->distance_limits, idx);
}
stark::RBCDirectionHandler stark::RigidBodies::add_constraint_direction(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_direction");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_direction");
	const int idx = this->constraints->directions->add(
		body_a.get_idx(),
		body_b.get_idx(),
		body_a.transform_global_to_local_direction(d_glob),
		body_b.transform_global_to_local_direction(d_glob),
		this->default_stiffness,
		this->default_tolerance_in_deg
	);
	return RBCDirectionHandler(body_a, body_b, this->constraints->directions, idx);
}
stark::RBCAngleLimitHandler stark::RigidBodies::add_constraint_angle_limit(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double admissible_angle_deg)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_angle_limit");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_angle_limit");
	const int idx = this->constraints->angle_limits->add(
		body_a.get_idx(),
		body_b.get_idx(),
		body_a.transform_global_to_local_direction(d_glob),
		body_b.transform_global_to_local_direction(d_glob),
		RigidBodyConstraints::AngleLimits::opening_distance_of_angle(admissible_angle_deg),
		this->default_stiffness,
		this->default_tolerance_in_deg
	);
	return RBCAngleLimitHandler(body_a, body_b, this->constraints->angle_limits, idx);
}
stark::RBCDampedSpringHandler stark::RigidBodies::add_constraint_spring(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double stiffness, double damping)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_spring");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_spring");
	const int idx = this->constraints->damped_springs->add(
		body_a.get_idx(),
		body_b.get_idx(),
		body_a.transform_global_to_local_point(a_glob),
		body_b.transform_global_to_local_point(b_glob),
		(a_glob - b_glob).norm(),
		stiffness,
		damping
	);
	return RBCDampedSpringHandler(body_a, body_b, this->constraints->damped_springs, idx);
}
stark::RBCLinearVelocityHandler stark::RigidBodies::add_constraint_linear_velocity(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_v, double max_force, double delay)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_linear_velocity");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_linear_velocity");

	// Check force is positive
	if (max_force < 0.0) {
		std::cout << "stark error: RigidBodies::add_constraint_linear_velocity() got a negative force for bodies " << body_a.get_label() << " and " << body_b.get_label() << std::endl;
		exit(-1);
	}

	const int idx = this->constraints->linear_velocity->add(
		body_a.get_idx(),
		body_b.get_idx(),
		body_a.transform_global_to_local_direction(d_glob),
		target_v,
		max_force,
		delay
	);
	return RBCLinearVelocityHandler(body_a, body_b, this->constraints->linear_velocity, idx);
}
stark::RBCAngularVelocityHandler stark::RigidBodies::add_constraint_angular_velocity(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_w, double max_abs_torque, double delay)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_angular_velocity");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_angular_velocity");

	// Check torque is positive
	if (max_abs_torque < 0.0) {
		std::cout << "stark error: RigidBodies::add_constraint_angular_velocity() got a negative torque for bodies " << body_a.get_label() << " and " << body_b.get_label() << std::endl;
		exit(-1);
	}

	const int idx = this->constraints->angular_velocity->add(
		body_a.get_idx(),
		body_b.get_idx(),
		body_a.transform_global_to_local_direction(d_glob),
		target_w,
		max_abs_torque,
		delay
	);
	return RBCAngularVelocityHandler(body_a, body_b, this->constraints->angular_velocity, idx);
}
stark::RBCFixHandler stark::RigidBodies::add_constraint_fix(const RigidBodyHandler& body)
{
	body.exit_if_not_valid("RigidBodies::add_constraint_fix");
	auto anchor_point = this->add_constraint_global_point(body, body.get_translation());
	auto z_lock = this->add_constraint_global_direction(body, Eigen::Vector3d::UnitZ());
	auto x_lock = this->add_constraint_global_direction(body, Eigen::Vector3d::UnitX());
	return RBCFixHandler(body, anchor_point, z_lock, x_lock);
}
stark::RBCAttachmentHandler stark::RigidBodies::add_constraint_attachment(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_attachment");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_attachment");
	auto point = this->add_constraint_point(body_a, body_b, 0.5*(body_a.get_translation() + body_b.get_translation()));
	auto z_lock = this->add_constraint_direction(body_a, body_b, Eigen::Vector3d::UnitZ());
	auto x_lock = this->add_constraint_direction(body_a, body_b, Eigen::Vector3d::UnitX());
	return RBCAttachmentHandler(body_a, body_b, point, z_lock, x_lock);
}
stark::RBCPointWithAngleLimitHandler stark::RigidBodies::add_constraint_point_with_angle_limit(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, const double admissible_angle_deg)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_point_with_angle_limit");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_point_with_angle_limit");
	auto point = this->add_constraint_point(body_a, body_b, 0.5 * (body_a.get_translation() + body_b.get_translation()));
	auto angle_limit = this->add_constraint_angle_limit(body_a, body_b, d_glob, admissible_angle_deg);
	return RBCPointWithAngleLimitHandler(body_a, body_b, point, angle_limit);
}
stark::RBCHingeJointHandler stark::RigidBodies::add_constraint_hinge(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_hinge");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_hinge");
	auto point = this->add_constraint_point(body_a, body_b, p_glob);
	auto direction = this->add_constraint_direction(body_a, body_b, d_glob);
	return RBCHingeJointHandler(body_a, body_b, point, direction);
}
stark::RBCHingeJointWithAngleLimitHandler stark::RigidBodies::add_constraint_hinge_with_angle_limit(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double admissible_angle_deg)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_hinge_with_angle_limit");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_hinge_with_angle_limit");
	const Eigen::Vector3d u = (d_glob.dot(Eigen::Vector3d::UnitX()) < 0.5) ? d_glob.cross(Eigen::Vector3d::UnitX()) : d_glob.cross(Eigen::Vector3d::UnitY());

	auto hinge_joint = this->add_constraint_hinge(body_a, body_b, p_glob, d_glob);
	auto angle_limits = this->add_constraint_angle_limit(body_a, body_b, u, admissible_angle_deg);
	return RBCHingeJointWithAngleLimitHandler(body_a, body_b, hinge_joint, angle_limits);
}
stark::RBCSliderHandler stark::RigidBodies::add_constraint_slider(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_slider");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_slider");
	auto point_on_axes = this->add_constraint_point_on_axis(body_a, body_b, p_glob, d_glob);
	auto dir_lock = this->add_constraint_direction(body_a, body_b, d_glob);
	return RBCSliderHandler(body_a, body_b, point_on_axes, dir_lock);
}
stark::RBCPrismaticSliderHandler stark::RigidBodies::add_constraint_prismatic_slider(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_prismatic_slider");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_prismatic_slider");
	const Eigen::Vector3d u = (d_glob.dot(Eigen::Vector3d::UnitX()) < 0.5) ? d_glob.cross(Eigen::Vector3d::UnitX()) : d_glob.cross(Eigen::Vector3d::UnitY());

	auto slider = this->add_constraint_slider(body_a, body_b, p_glob, d_glob);
	auto dir_lock = this->add_constraint_direction(body_a, body_b, u);
	return RBCPrismaticSliderHandler(body_a, body_b, slider, dir_lock);
}
stark::RBCSpringWithLimitsHandler stark::RigidBodies::add_constraint_spring_with_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double stiffness, double min_length, double max_length, double damping)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_spring_with_limits");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_spring_with_limits");
	auto spring = this->add_constraint_spring(body_a, body_b, a_glob, b_glob, stiffness, damping);
	auto distance_limits = this->add_constraint_distance_limits(body_a, body_b, a_glob, b_glob, min_length, max_length);
	return RBCSpringWithLimitsHandler(body_a, body_b, spring, distance_limits);
}
stark::RBCPrismaticPressHandler stark::RigidBodies::add_constraint_prismatic_press(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double target_v, double max_force, double delay)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_prismatic_press");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_prismatic_press");
	auto prismatic_slider = this->add_constraint_prismatic_slider(body_a, body_b, p_glob, d_glob);
	auto linear_velocity = this->add_constraint_linear_velocity(body_a, body_b, d_glob, target_v, max_force, delay);
	return RBCPrismaticPressHandler(body_a, body_b, prismatic_slider, linear_velocity);
}
stark::RBCMotorHandler stark::RigidBodies::add_constraint_motor(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double target_w, double max_torque, double delay)
{
	body_a.exit_if_not_valid("RigidBodies::add_constraint_motor");
	body_b.exit_if_not_valid("RigidBodies::add_constraint_motor");
	auto hinge_joint = this->add_constraint_hinge(body_a, body_b, p_glob, d_glob);
	auto angular_velocity = this->add_constraint_angular_velocity(body_a, body_b, d_glob, target_w, max_torque, delay);
	return RBCMotorHandler(body_a, body_b, hinge_joint, angular_velocity);
}
