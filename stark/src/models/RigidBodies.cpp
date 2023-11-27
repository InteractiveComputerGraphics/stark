#include "RigidBodies.h"

#include "../utils/mesh_utils.h"

stark::models::RigidBodies::RigidBodies(Stark& stark, spRigidBodyDynamics dyn)
{
	this->rb = std::make_shared<RigidBodiesInternal>(stark, dyn);
}

stark::models::RigidBodyHandler stark::models::RigidBodies::add(const double mass, const Eigen::Matrix3d& inertia_local, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
{
	const int idx = this->rb->dyn->add(displacement, rotate_deg, rotation_axis);
	this->rb->inertia->add(mass, inertia_local, 0.0, 0.0);
	this->rb->collision_meshes.push_back(utils::Mesh<3>());
	this->rb->render_meshes.push_back(utils::Mesh<3>());
	return RigidBodyHandler(this->rb, idx);
}
stark::models::RigidBodyHandler stark::models::RigidBodies::add_sphere(const double mass, const double radius, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis, const int subdivisions)
{
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_sphere(mass, radius);
	auto body = this->add(mass, inertia_local, displacement, rotate_deg, rotation_axis);
	
	const utils::Mesh m = utils::make_sphere(radius, subdivisions);
	body.set_collision_mesh(m.vertices, m.conn);
	body.set_render_mesh(m.vertices, m.conn);
	
	return body;
}
stark::models::RigidBodyHandler stark::models::RigidBodies::add_box(const double mass, const Eigen::Vector3d& size, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
{
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_box(mass, size);
	auto body = this->add(mass, inertia_local, displacement, rotate_deg, rotation_axis);

	const utils::Mesh m = utils::make_box(size);
	body.set_collision_mesh(m.vertices, m.conn);
	body.set_render_mesh(m.vertices, m.conn);

	return body;
}
stark::models::RigidBodyHandler stark::models::RigidBodies::add_cylinder(const double mass, const double radius, const double full_height, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis, const int slices, const int stacks)
{
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_cylinder(mass, radius, full_height);
	auto body = this->add(mass, inertia_local, displacement, rotate_deg, rotation_axis);

	const utils::Mesh m = utils::make_cylinder(radius, full_height, slices, stacks);
	body.set_collision_mesh(m.vertices, m.conn);
	body.set_render_mesh(m.vertices, m.conn);

	return body;
}
stark::models::RigidBodyHandler stark::models::RigidBodies::add_torus(const double mass, const double outer_radius, const double inner_radius, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis, const int slices, const int stacks)
{
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_torus(mass, outer_radius, inner_radius);
	auto body = this->add(mass, inertia_local, displacement, rotate_deg, rotation_axis);

	const utils::Mesh m = utils::make_torus(outer_radius, inner_radius, slices, stacks);
	body.set_collision_mesh(m.vertices, m.conn);
	body.set_render_mesh(m.vertices, m.conn);

	return body;
}

stark::models::AnchorPointHandler stark::models::RigidBodies::add_constraint_anchor_point(const RigidBodyHandler& body, const Eigen::Vector3d& p_glob, double stiffness_per_kg)
{
	const int idx = this->rb->constraints->anchor_points->add(
		body.index(), 
		body.global_to_local_point(p_glob),
		p_glob, 
		stiffness_per_kg * body.get_mass()
	);
	return AnchorPointHandler(body, this->rb->constraints->anchor_points, idx);
}
stark::models::BallJointHandler stark::models::RigidBodies::add_constraint_ball_joint(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, double stiffness_per_kg)
{
	const int idx = this->rb->constraints->ball_joints->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_point(p_glob),
		body_b.global_to_local_point(p_glob),
		stiffness_per_kg * std::max(body_a.get_mass(), body_b.get_mass())
	);
	return BallJointHandler(body_a, body_b, this->rb->constraints->ball_joints, idx);
}
stark::models::RelativeDirectionLockHandler stark::models::RigidBodies::add_constraint_relative_direction_lock(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double stiffness_per_kg)
{
	const int idx = this->rb->constraints->relative_direction_locks->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_direction(d_glob),
		body_b.global_to_local_direction(d_glob),
		stiffness_per_kg * std::max(body_a.get_mass(), body_b.get_mass())
	);
	return RelativeDirectionLockHandler(body_a, body_b, this->rb->constraints->relative_direction_locks, idx);
}
stark::models::PointOnAxisConstraintHandler stark::models::RigidBodies::add_constraint_point_on_axis(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, const Eigen::Vector3d& p_glob, double stiffness_per_kg)
{
	const int idx = this->rb->constraints->point_on_axis->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_point(p_glob),
		body_a.global_to_local_direction(d_glob),
		body_b.global_to_local_point(p_glob),
		stiffness_per_kg * std::max(body_a.get_mass(), body_b.get_mass())
	);
	return PointOnAxisConstraintHandler(body_a, body_b, this->rb->constraints->point_on_axis, idx);
}
stark::models::DampedSpringHandler stark::models::RigidBodies::add_spring(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double stiffness, double damping)
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
	return DampedSpringHandler(body_a, body_b, this->rb->constraints->damped_springs, idx);
}
stark::models::DistanceLimitHandler stark::models::RigidBodies::add_constraint_distance_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double min_length, double max_length, double stiffness_per_kg)
{
	const int idx = this->rb->constraints->distance_limits->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_point(a_glob),
		body_b.global_to_local_point(b_glob),
		min_length,
		max_length,
		stiffness_per_kg * std::max(body_a.get_mass(), body_b.get_mass())
	);
	return DistanceLimitHandler(body_a, body_b, this->rb->constraints->distance_limits, idx);
}
stark::models::AngleLimitHandler stark::models::RigidBodies::add_constraint_angle_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double admissible_angle_deg, double stiffness_per_kg)
{
	const int idx = this->rb->constraints->angle_limits->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_direction(d_glob),
		body_b.global_to_local_direction(d_glob),
		std::cos(utils::deg2rad(admissible_angle_deg)),
		stiffness_per_kg * std::max(body_a.get_mass(), body_b.get_mass())
	);
	return AngleLimitHandler(body_a, body_b, this->rb->constraints->angle_limits, idx);
}
stark::models::RelativeLinearVelocityMotorHandler stark::models::RigidBodies::add_motor_linear(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_v, double max_force, double delay)
{
	const int idx = this->rb->constraints->relative_linear_velocity_motors->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_direction(d_glob),
		target_v,
		max_force,
		delay
	);
	return RelativeLinearVelocityMotorHandler(body_a, body_b, this->rb->constraints->relative_linear_velocity_motors, idx);
}
stark::models::RelativeAngularVelocityMotorHandler stark::models::RigidBodies::add_motor_angular(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_w, double max_torque, double delay)
{
	const int idx = this->rb->constraints->relative_angular_velocity_motors->add(
		body_a.index(),
		body_b.index(),
		body_a.global_to_local_direction(d_glob),
		target_w,
		max_torque,
		delay
	);
	return RelativeAngularVelocityMotorHandler(body_a, body_b, this->rb->constraints->relative_angular_velocity_motors, idx);
}
