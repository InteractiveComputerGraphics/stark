#pragma once
#include <vector>
#include <array>
#include <memory>

#include <Eigen/Dense>

#include "../../core/Stark.h"
#include "RigidBodiesInternal.h"
#include "RigidBodyHandler.h"
#include "rigidbody_constraints_ui.h"


namespace stark::models
{
	class RigidBodies 
	{
	private:
		constexpr static double DEFAULT_HARD_STIFFNESS = 1e6;
		constexpr static double DEFAULT_HARD_TOLERANCE_IN_M = 0.001;
		constexpr static double DEFAULT_HARD_TOLERANCE_IN_DEG = 1.0;

	public:
		/* Fields */
		spRigidBodiesInternal rb;

		/* Methods */
		RigidBodies(stark::core::Stark& stark, spRigidBodyDynamics dyn);
		void write_render_meshes(bool boolean = true);
		void write_collision_meshes(bool boolean = true);

		// Add rigid bodies
		RigidBodyHandler add(const double mass, const Eigen::Matrix3d& inertia_local);
		RigidBodyHandler add_sphere(const double mass, const double radius, const int subdivisions = 2);
		RigidBodyHandler add_box(const double mass, const Eigen::Vector3d& size);
		RigidBodyHandler add_cylinder(const double mass, const double radius, const double full_height, const int slices = 16, const int stacks = 1);
		RigidBodyHandler add_torus(const double mass, const double outer_radius, const double inner_radius, const int slices = 16, const int stacks = 32);

		// Add constraints
		//// Base
		AnchorPointHandler add_constraint_anchor_point(const RigidBodyHandler& body, const Eigen::Vector3d& p_glob, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double stiffness = DEFAULT_HARD_STIFFNESS);
		AbsoluteDirectionLockHandler add_constraint_absolute_direction_lock(const RigidBodyHandler& body, const Eigen::Vector3d& d_glob, double tolerance_in_deg = DEFAULT_HARD_TOLERANCE_IN_DEG, double stiffness = DEFAULT_HARD_STIFFNESS);
		BallJointHandler add_constraint_ball_joint(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double stiffness = DEFAULT_HARD_STIFFNESS);
		RelativeDirectionLockHandler add_constraint_relative_direction_lock(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double tolerance_in_deg = DEFAULT_HARD_TOLERANCE_IN_DEG, double stiffness = DEFAULT_HARD_STIFFNESS);
		PointOnAxisConstraintHandler add_constraint_point_on_axis(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double stiffness = DEFAULT_HARD_STIFFNESS);
		DampedSpringHandler add_spring(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double stiffness, double damping = 0.0);
		DistanceLimitHandler add_constraint_distance_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double min_length, double max_length, double stiffness = DEFAULT_HARD_STIFFNESS);
		AngleLimitHandler add_constraint_angle_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double admissible_angle_deg, double stiffness = DEFAULT_HARD_STIFFNESS);
		RelativeLinearVelocityMotorHandler add_relative_linear_velocity_motor(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_v, double max_force, double delay = 0.01);
		RelativeAngularVelocityMotorHandler add_relative_angular_velocity_motor(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& d_glob, double target_w, double max_torque, double delay = 0.01);
		
		//// Derived
		FixedConstraintHandler add_constraint_fixed(const RigidBodyHandler& body, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double tolerance_in_deg = DEFAULT_HARD_TOLERANCE_IN_DEG, double stiffness = DEFAULT_HARD_STIFFNESS);
		HingeJointHandler add_constraint_hinge(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double tolerance_in_deg = DEFAULT_HARD_TOLERANCE_IN_DEG, double stiffness = DEFAULT_HARD_STIFFNESS);
		HingeJointWithLimitsHandler add_constraint_hinge_with_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double admissible_angle_deg, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double tolerance_in_deg = DEFAULT_HARD_TOLERANCE_IN_DEG, double stiffness = DEFAULT_HARD_STIFFNESS);
		SliderHandler add_constraint_slider(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double tolerance_in_deg = DEFAULT_HARD_TOLERANCE_IN_DEG, double stiffness = DEFAULT_HARD_STIFFNESS);
		PrismaticSliderHandler add_constraint_prismatic_slider(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double tolerance_in_deg = DEFAULT_HARD_TOLERANCE_IN_DEG, double stiffness = DEFAULT_HARD_STIFFNESS);
		//PrismaticSliderWithLimitsHandler add_constraint_prismatic_slider_with_limits(...);
		SpringWithLimitsHandler add_spring_with_limits(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& a_glob, const Eigen::Vector3d& b_glob, double stiffness, double min_length, double max_length, double damping = 0.0, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double limit_stiffness = DEFAULT_HARD_STIFFNESS);
		PrismaticPressHandler add_prismatic_press(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double target_v, double max_force, double delay = 0.01, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double tolerance_in_deg = DEFAULT_HARD_TOLERANCE_IN_DEG, double slider_stiffness = DEFAULT_HARD_STIFFNESS);
		MotorHandler add_motor(const RigidBodyHandler& body_a, const RigidBodyHandler& body_b, const Eigen::Vector3d& p_glob, const Eigen::Vector3d& d_glob, double target_w, double max_torque, double delay = 0.01, double tolerance_in_m = DEFAULT_HARD_TOLERANCE_IN_M, double tolerance_in_deg = DEFAULT_HARD_TOLERANCE_IN_DEG, double hinge_stiffness = DEFAULT_HARD_STIFFNESS);
		//SuspensionHandler add_suspension(...);
	};
	using spRigidBodies = std::shared_ptr<RigidBodies>;
}
