#pragma once
#include <vector>
#include <array>
#include <memory>

#include <Eigen/Dense>

#include "../solver/Stark.h"
#include "RigidBodiesInternal.h"
#include "RigidBodyHandler.h"
#include "rigidbody_constraints_ui.h"


namespace stark::models
{
	/*
	*	UI.
	*/
	class RigidBodies 
	{
	public:
		/* Fields */
		spRigidBodiesInternal rb;

		/* Methods */
		RigidBodies(Stark& stark, spRigidBodyDynamics dyn);

		// Add rigid bodies
		RigidBodyHandler add(const double mass, const Eigen::Matrix3d& inertia_local);
		RigidBodyHandler add_sphere(const double mass, const double radius, const int subdivisions = 2);
		RigidBodyHandler add_box(const double mass, const Eigen::Vector3d& size);
		RigidBodyHandler add_cylinder(const double mass, const double radius, const double full_height, const int slices = 16, const int stacks = 1);
		RigidBodyHandler add_torus(const double mass, const double outer_radius, const double inner_radius, const int slices = 16, const int stacks = 32);

		// Add constraints
		AnchorPointHandler add_constraint_anchor_point(const RigidBodyHandler& body, const Eigen::Vector3d& p_glob, double stiffness_per_kg = 1e6);
		BallJointHandler add_constraint_ball_joint(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, double stiffness_per_kg = 1e6);
		RelativeDirectionLockHandler add_constraint_relative_direction_lock(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double stiffness_per_kg = 1e6);
		PointOnAxisConstraintHandler add_constraint_point_on_axis(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, const Eigen::Vector3d& p_glob, double stiffness_per_kg = 1e6);
		DampedSpringHandler add_spring(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double stiffness, double damping = 0.0);
		DistanceLimitHandler add_constraint_distance_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double min_length, double max_length, double stiffness_per_kg = 1e6);
		AngleLimitHandler add_constraint_angle_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double admissible_angle_deg, double stiffness_per_kg = 1e6);
		RelativeLinearVelocityMotorHandler add_motor_linear(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_v, double max_force, double delay = 0.01);
		RelativeAngularVelocityMotorHandler add_motor_angular(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_w, double max_torque, double delay = 0.01);
	};
	using spRigidBodies = std::shared_ptr<RigidBodies>;
}
