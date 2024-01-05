#include "RigidBodies.h"
#include <functional>

#include "inertia_tensors.h"
#include "../../utils/mesh_utils.h"

stark::models::RigidBodies::RigidBodies(stark::core::Stark& stark, spRigidBodyDynamics dyn, spEnergyFrictionalContact contact)
{
	this->rb = std::make_shared<RigidBodiesInternal>(stark, dyn, contact);
}
void stark::models::RigidBodies::write_render_meshes(bool boolean)
{
	this->rb->write_render_mesh = boolean;
}
void stark::models::RigidBodies::write_collision_meshes(bool boolean)
{
	this->rb->write_collision_mesh = boolean;
}

void stark::models::RigidBodies::set_default_constraint_stiffness(double stiffness)
{
	this->default_stiffness = stiffness;
}
void stark::models::RigidBodies::set_default_constraint_distance_tolerance(double tolerance_in_m)
{
	this->default_tolerance_in_m = this->default_tolerance_in_m;
}
void stark::models::RigidBodies::set_default_constraint_angle_tolerance(double tolerance_in_deg)
{
	this->default_tolerance_in_deg = tolerance_in_deg;
}
double stark::models::RigidBodies::get_default_constraint_stiffness() const
{
	return this->default_stiffness;
}
double stark::models::RigidBodies::get_default_constraint_distance_tolerance() const
{
	return this->default_tolerance_in_m;
}
double stark::models::RigidBodies::get_default_constraint_angle_tolerance() const
{
	return this->default_tolerance_in_deg;
}

stark::models::RigidBodyHandler stark::models::RigidBodies::add(const double mass, const Eigen::Matrix3d& inertia_local)
{
	const int idx = this->rb->dyn->add();
	this->rb->inertia->add(mass, inertia_local, 0.0, 0.0);
	this->rb->collision_meshes.push_back(utils::Mesh<3>());
	this->rb->render_meshes.push_back(utils::Mesh<3>());
	return RigidBodyHandler(this->rb, idx);
}
stark::models::RigidBodyHandler stark::models::RigidBodies::add_sphere(const double mass, const double radius, const int subdivisions)
{
	const Eigen::Matrix3d inertia_local = inertia_tensor_sphere(mass, radius);
	auto body = this->add(mass, inertia_local);
	
	const utils::Mesh m = utils::make_sphere(radius, subdivisions);
	body.set_collision_mesh(m.vertices, m.conn);
	body.set_render_mesh(m.vertices, m.conn);
	
	return body;
}
stark::models::RigidBodyHandler stark::models::RigidBodies::add_box(const double mass, const Eigen::Vector3d& size)
{
	const Eigen::Matrix3d inertia_local = inertia_tensor_box(mass, size);
	auto body = this->add(mass, inertia_local);

	const utils::Mesh m = utils::make_box(size);
	body.set_collision_mesh(m.vertices, m.conn);
	body.set_render_mesh(m.vertices, m.conn);

	return body;
}
stark::models::RigidBodyHandler stark::models::RigidBodies::add_box(const double mass, const double size)
{
	return this->add_box(mass, { size, size, size });
}
stark::models::RigidBodyHandler stark::models::RigidBodies::add_cylinder(const double mass, const double radius, const double full_height, const int slices, const int stacks)
{
	const Eigen::Matrix3d inertia_local = inertia_tensor_cylinder(mass, radius, full_height);
	auto body = this->add(mass, inertia_local);

	const utils::Mesh m = utils::make_cylinder(radius, full_height, slices, stacks);
	body.set_collision_mesh(m.vertices, m.conn);
	body.set_render_mesh(m.vertices, m.conn);

	return body;
}
stark::models::RigidBodyHandler stark::models::RigidBodies::add_torus(const double mass, const double outer_radius, const double inner_radius, const int slices, const int stacks)
{
	const Eigen::Matrix3d inertia_local = inertia_tensor_torus(mass, outer_radius, inner_radius);
	auto body = this->add(mass, inertia_local);

	const utils::Mesh m = utils::make_torus(outer_radius, inner_radius, slices, stacks);
	body.set_collision_mesh(m.vertices, m.conn);
	body.set_render_mesh(m.vertices, m.conn);

	return body;
}

stark::models::RBCGlobalPointHandler stark::models::RigidBodies::add_constraint_global_point(const RigidBodyHandler& body, const Eigen::Vector3d& p_glob)
{
	const int idx = this->rb->constraints->global_points->add(
		body.index(), 
		body.global_to_local_point(p_glob),
		p_glob, 
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCGlobalPointHandler(body, this->rb->constraints->global_points, idx);
}
stark::models::RBCGlobalDirectionHandler stark::models::RigidBodies::add_constraint_global_direction(const RigidBodyHandler& body, const Eigen::Vector3d& d_glob)
{
	const int idx = this->rb->constraints->global_directions->add(
		body.index(),
		body.global_to_local_direction(d_glob),
		d_glob,
		this->default_stiffness,
		this->default_tolerance_in_deg
	);
	return RBCGlobalDirectionHandler(body, this->rb->constraints->global_directions, idx);
}
stark::models::RBCPointHandler stark::models::RigidBodies::add_constraint_point(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob)
{
	const int idx = this->rb->constraints->points->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_point(p_glob),
		body_b.global_to_local_point(p_glob),
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCPointHandler(body_a, body_b, this->rb->constraints->points, idx);
}
stark::models::RBCPointOnAxisHandler stark::models::RigidBodies::add_constraint_point_on_axis(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob)
{
	const int idx = this->rb->constraints->point_on_axes->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_point(p_glob),
		body_a.global_to_local_direction(d_glob),
		body_b.global_to_local_point(p_glob),
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCPointOnAxisHandler(body_a, body_b, this->rb->constraints->point_on_axes, idx);
}
stark::models::RBCDistanceHandler stark::models::RigidBodies::add_constraint_distance(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob)
{
	const int idx = this->rb->constraints->distances->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_point(a_glob),
		body_b.global_to_local_point(b_glob),
		(a_glob - b_glob).norm(),
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCDistanceHandler(body_a, body_b, this->rb->constraints->distances, idx);
}
stark::models::RBCDistanceLimitHandler stark::models::RigidBodies::add_constraint_distance_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double min_distance, double max_distance)
{
	const double d = (a_glob - b_glob).norm();
	if (d < min_distance || max_distance < d) {
		std::cout << "stark error: RigidBodies::add_constraint_distance_limits() got a rest distance out of limits for bodies " << body_a.get_label() << " and " << body_b.get_label() << std::endl;
		exit(-1);
	}

	const int idx = this->rb->constraints->distance_limits->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_point(a_glob),
		body_b.global_to_local_point(b_glob),
		min_distance,
		max_distance,
		this->default_stiffness,
		this->default_tolerance_in_m
	);
	return RBCDistanceLimitHandler(body_a, body_b, this->rb->constraints->distance_limits, idx);
}
stark::models::RBCDirectionHandler stark::models::RigidBodies::add_constraint_direction(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob)
{
	const int idx = this->rb->constraints->directions->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_direction(d_glob),
		body_b.global_to_local_direction(d_glob),
		this->default_stiffness,
		this->default_tolerance_in_deg
	);
	return RBCDirectionHandler(body_a, body_b, this->rb->constraints->directions, idx);
}
stark::models::RBCAngleLimitHandler stark::models::RigidBodies::add_constraint_angle_limit(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double admissible_angle_deg)
{
	const int idx = this->rb->constraints->angle_limits->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_direction(d_glob),
		body_b.global_to_local_direction(d_glob),
		RigidBodyConstraints::AngleLimits::opening_distance_of_angle(admissible_angle_deg),
		this->default_stiffness,
		this->default_tolerance_in_deg
	);
	return RBCAngleLimitHandler(body_a, body_b, this->rb->constraints->angle_limits, idx);
}
stark::models::RBCDampedSpringHandler stark::models::RigidBodies::add_constraint_spring(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double stiffness, double damping)
{
	const int idx = this->rb->constraints->damped_springs->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_point(a_glob),
		body_b.global_to_local_point(b_glob),
		(a_glob - b_glob).norm(),
		stiffness,
		damping
	);
	return RBCDampedSpringHandler(body_a, body_b, this->rb->constraints->damped_springs, idx);
}
stark::models::RBCLinearVelocityHandler stark::models::RigidBodies::add_constraint_linear_velocity(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_v, double max_force, double delay)
{
	const int idx = this->rb->constraints->linear_velocity->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_direction(d_glob),
		target_v,
		max_force,
		delay
	);
	return RBCLinearVelocityHandler(body_a, body_b, this->rb->constraints->linear_velocity, idx);
}
stark::models::RBCAngularVelocityHandler stark::models::RigidBodies::add_constraint_angular_velocity(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_w, double max_torque, double delay)
{
	const int idx = this->rb->constraints->angular_velocity->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_direction(d_glob),
		target_w,
		max_torque,
		delay
	);
	return RBCAngularVelocityHandler(body_a, body_b, this->rb->constraints->angular_velocity, idx);
}
stark::models::RBCFixHandler stark::models::RigidBodies::add_constraint_fix(const RigidBodyHandler& body)
{
	auto anchor_point = this->add_constraint_global_point(body, body.get_translation());
	auto z_lock = this->add_constraint_global_direction(body, Eigen::Vector3d::UnitZ());
	auto x_lock = this->add_constraint_global_direction(body, Eigen::Vector3d::UnitX());
	return RBCFixHandler(body, anchor_point, z_lock, x_lock);
}
stark::models::RBCAttachmentHandler stark::models::RigidBodies::add_constraint_attachment(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b)
{
	auto point = this->add_constraint_point(body_a, body_b, 0.5*(body_a.get_translation() + body_b.get_translation()));
	auto z_lock = this->add_constraint_direction(body_a, body_b, Eigen::Vector3d::UnitZ());
	auto x_lock = this->add_constraint_direction(body_a, body_b, Eigen::Vector3d::UnitX());
	return RBCAttachmentHandler(body_a, body_b, point, z_lock, x_lock);
}
stark::models::RBCPointWithAngleLimitHandler stark::models::RigidBodies::add_constraint_point_with_angle_limit(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, const double admissible_angle_deg)
{
	auto point = this->add_constraint_point(body_a, body_b, 0.5 * (body_a.get_translation() + body_b.get_translation()));
	auto angle_limit = this->add_constraint_angle_limit(body_a, body_b, d_glob, admissible_angle_deg);
	return RBCPointWithAngleLimitHandler(body_a, body_b, point, angle_limit);
}
stark::models::RBCHingeJointHandler stark::models::RigidBodies::add_constraint_hinge(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob)
{
	auto point = this->add_constraint_point(body_a, body_b, p_glob);
	auto direction = this->add_constraint_direction(body_a, body_b, d_glob);
	return RBCHingeJointHandler(body_a, body_b, point, direction);
}
stark::models::RBCHingeJointWithAngleLimitHandler stark::models::RigidBodies::add_constraint_hinge_with_angle_limit(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double admissible_angle_deg)
{
	const Eigen::Vector3d u = (d_glob.dot(Eigen::Vector3d::UnitX()) < 0.5) ? d_glob.cross(Eigen::Vector3d::UnitX()) : d_glob.cross(Eigen::Vector3d::UnitY());

	auto hinge_joint = this->add_constraint_hinge(body_a, body_b, p_glob, d_glob);
	auto angle_limits = this->add_constraint_angle_limit(body_a, body_b, u, admissible_angle_deg);
	return RBCHingeJointWithAngleLimitHandler(body_a, body_b, hinge_joint, angle_limits);
}
stark::models::RBCSliderHandler stark::models::RigidBodies::add_constraint_slider(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob)
{
	auto point_on_axes = this->add_constraint_point_on_axis(body_a, body_b, p_glob, d_glob);
	auto dir_lock = this->add_constraint_direction(body_a, body_b, d_glob);
	return RBCSliderHandler(body_a, body_b, point_on_axes, dir_lock);
}
stark::models::RBCPrismaticSliderHandler stark::models::RigidBodies::add_constraint_prismatic_slider(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob)
{
	const Eigen::Vector3d u = (d_glob.dot(Eigen::Vector3d::UnitX()) < 0.5) ? d_glob.cross(Eigen::Vector3d::UnitX()) : d_glob.cross(Eigen::Vector3d::UnitY());

	auto slider = this->add_constraint_slider(body_a, body_b, p_glob, d_glob);
	auto dir_lock = this->add_constraint_direction(body_a, body_b, u);
	return RBCPrismaticSliderHandler(body_a, body_b, slider, dir_lock);
}
stark::models::RBCSpringWithLimitsHandler stark::models::RigidBodies::add_spring_with_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double stiffness, double min_length, double max_length, double damping)
{
	auto spring = this->add_constraint_spring(body_a, body_b, a_glob, b_glob, stiffness, damping);
	auto distance_limits = this->add_constraint_distance_limits(body_a, body_b, a_glob, b_glob, min_length, max_length);
	return RBCSpringWithLimitsHandler(body_a, body_b, spring, distance_limits);
}
stark::models::RBCPrismaticPressHandler stark::models::RigidBodies::add_prismatic_press(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double target_v, double max_force, double delay)
{
	auto prismatic_slider = this->add_constraint_prismatic_slider(body_a, body_b, p_glob, d_glob);
	auto linear_velocity = this->add_constraint_linear_velocity(body_a, body_b, d_glob, target_v, max_force, delay);
	return RBCPrismaticPressHandler(body_a, body_b, prismatic_slider, linear_velocity);
}
stark::models::RBCMotorHandler stark::models::RigidBodies::add_motor(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double target_w, double max_torque, double delay)
{
	auto hinge_joint = this->add_constraint_hinge(body_a, body_b, p_glob, d_glob);
	auto angular_velocity = this->add_constraint_angular_velocity(body_a, body_b, d_glob, target_w, max_torque, delay);
	return RBCMotorHandler(body_a, body_b, hinge_joint, angular_velocity);
}
