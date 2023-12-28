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
		/* Fields */
		spRigidBodiesInternal rb;
		double default_stiffness = 1e6;
		double default_tolerance_in_m = 0.001;
		double default_tolerance_in_deg = 1.0;

	public:
		/* Methods */
		RigidBodies(stark::core::Stark& stark, spRigidBodyDynamics dyn);
		void write_render_meshes(bool boolean = true);
		void write_collision_meshes(bool boolean = true);

		void set_default_constraint_stiffness(double stiffness);
		void set_default_constraint_distance_tolerance(double tolerance_in_m);
		void set_default_constraint_angle_tolerance(double tolerance_in_deg);
		double get_default_constraint_stiffness() const;
		double get_default_constraint_distance_tolerance() const;
		double get_default_constraint_angle_tolerance() const;

		// Add rigid bodies
		RigidBodyHandler add(const double mass, const Eigen::Matrix3d& inertia_local);
		RigidBodyHandler add_sphere(const double mass, const double radius, const int subdivisions = 2);
		RigidBodyHandler add_box(const double mass, const Eigen::Vector3d& size);
		RigidBodyHandler add_box(const double mass, const double size);
		RigidBodyHandler add_cylinder(const double mass, const double radius, const double full_height, const int slices = 16, const int stacks = 1);
		RigidBodyHandler add_torus(const double mass, const double outer_radius, const double inner_radius, const int slices = 16, const int stacks = 32);

		// Add constraints
		//// Base
		RBCGlobalPointHandler add_constraint_global_point(
			const RigidBodyHandler& body, 
			const Eigen::Vector3d& p_glob
		);
		RBCGlobalDirectionHandler add_constraint_global_direction(
			const RigidBodyHandler& body, 
			const Eigen::Vector3d& d_glob
		);
		RBCPointHandler add_constraint_point(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b, 
			const Eigen::Vector3d& p_glob
		);
		RBCPointOnAxisHandler add_constraint_point_on_axis(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b, 
			const Eigen::Vector3d& p_glob, 
			const Eigen::Vector3d& d_glob
		);
		RBCDistanceHandler add_constraint_distance(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b, 
			const Eigen::Vector3d& a_glob,
			const Eigen::Vector3d& b_glob
		);
		RBCDistanceLimitHandler add_constraint_distance_limits(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b, 
			const Eigen::Vector3d& a_glob, 
			const Eigen::Vector3d& b_glob, 
			double min_distance,
			double max_distance
		);
		RBCDirectionHandler add_constraint_direction(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b, 
			const Eigen::Vector3d& d_glob
		);
		RBCAngleLimitHandler add_constraint_angle_limit(
			const RigidBodyHandler& body_a,
			const RigidBodyHandler& body_b, 
			const Eigen::Vector3d& d_glob,
			double admissible_angle_deg
		);

		RBCDampedSpringHandler add_constraint_spring(
			const RigidBodyHandler& body_a,
			const RigidBodyHandler& body_b,
			const Eigen::Vector3d& a_glob, 
			const Eigen::Vector3d& b_glob, 
			double stiffness, 
			double damping = 0.0
		);
		RBCLinearVelocityHandler add_constraint_linear_velocity(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b,
			const Eigen::Vector3d& d_glob, 
			double target_v, 
			double max_force,
			double delay = 0.01
		);
		RBCAngularVelocityHandler add_constraint_angular_velocity(
			const RigidBodyHandler& body_a,
			const RigidBodyHandler& body_b,
			const Eigen::Vector3d& d_glob, 
			double target_w, 
			double max_torque, 
			double delay = 0.01
		);
		
		//// Derived
		RBCFixHandler add_constraint_fix(
			const RigidBodyHandler& body
		);
		RBCAttachmentHandler add_constraint_attachment(
			const RigidBodyHandler& body_a,
			const RigidBodyHandler& body_b
		);
		RBCPointWithAngleLimitHandler add_constraint_point_with_angle_limit(
			const RigidBodyHandler& body_a,
			const RigidBodyHandler& body_b,
			const Eigen::Vector3d& p_glob,
			const Eigen::Vector3d& d_glob,
			const double admissible_angle_deg
		);
		RBCHingeJointHandler add_constraint_hinge(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b,
			const Eigen::Vector3d& p_glob, 
			const Eigen::Vector3d& d_glob
		);
		RBCHingeJointWithAngleLimitHandler add_constraint_hinge_with_angle_limit(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b, 
			const Eigen::Vector3d& p_glob, 
			const Eigen::Vector3d& d_glob,
			double admissible_angle_deg
		);
		RBCSpringWithLimitsHandler add_spring_with_limits(
			const RigidBodyHandler& body_a,
			const RigidBodyHandler& body_b, 
			const Eigen::Vector3d& a_glob, 
			const Eigen::Vector3d& b_glob, 
			double stiffness,
			double min_length,
			double max_length,
			double damping = 0.0
		);
		RBCSliderHandler add_constraint_slider(
			const RigidBodyHandler& body_a,
			const RigidBodyHandler& body_b,
			const Eigen::Vector3d& p_glob,
			const Eigen::Vector3d& d_glob
		);
		RBCPrismaticSliderHandler add_constraint_prismatic_slider(
			const RigidBodyHandler& body_a,
			const RigidBodyHandler& body_b,
			const Eigen::Vector3d& p_glob, 
			const Eigen::Vector3d& d_glob
		);
		RBCPrismaticPressHandler add_prismatic_press(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b,
			const Eigen::Vector3d& p_glob, 
			const Eigen::Vector3d& d_glob, 
			double target_v,
			double max_force,
			double delay = 0.01
		);
		RBCMotorHandler add_motor(
			const RigidBodyHandler& body_a, 
			const RigidBodyHandler& body_b,
			const Eigen::Vector3d& p_glob,
			const Eigen::Vector3d& d_glob, 
			double target_w, 
			double max_torque,
			double delay = 0.01
		);
	};
}
