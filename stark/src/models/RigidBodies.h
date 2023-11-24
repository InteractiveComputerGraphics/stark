#pragma once
#include <vector>
#include <array>
#include <unordered_map>
#include <memory>

#include <Eigen/Dense>
#include <symx>

#include "../solver/Stark.h"
#include "RigidBodiesData.h"
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
		spRigidBodiesData data;

		/* Methods */
		RigidBodies(Stark& stark, spRigidBodyDynamics dyn);
		RigidBodyHandler add(const double mass, const Eigen::Matrix3d& inertia_loc, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		RigidBodyHandler add_sphere(const double mass, const double radius, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int subdivisions = 2);
		RigidBodyHandler add_box(const double mass, const Eigen::Vector3d& size, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		RigidBodyHandler add_cylinder(const double mass, const double radius, const double full_height, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int slices = 16, const int stacks = 1);
		RigidBodyHandler add_torus(const double mass, const double outer_radius, const double inner_radius, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int slices = 16, const int stacks = 32);


		AnchorPointHandler add_constraint_anchor_point(const RigidBodyHandler& body, const Eigen::Vector3d& p_glob, double stiffness_per_kg = 1e6);
		void add_constraint_ball_joint(const int body_0, const int body_1, const Eigen::Vector3d& p_glob);
		void add_constraint_hinge_joint(const int& body_0, const int& body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global);
		void add_constraint_slider(const int body_0, const int body_1, const Eigen::Vector3d& p0_global, const Eigen::Vector3d& p1_global, const double spring_stiffness = 0.0, const double spring_damping = 0.0);
		void add_constraint_relative_direction_lock(const int body_0, const int body_1, const Eigen::Vector3d& d_global);
		void add_constraint_relative_direction_lock(const int body_0, const int body_1);
		void add_constraint_freeze(const int body_id);
		void add_constraint_motor(const int body_0, const int body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global, const double max_torque, const double target_w, const double delay = 0.01);
		void add_constraint_parallel_gripper(const int base, const int right_finger, const int left_finger, const Eigen::Vector3d& d_global, const double max_force, const double closing_velocity, const double delay = 0.01);

	private:
		// Stark callbacks
		void _write_frame(Stark& sim);
	};
	using spRigidBodies = std::shared_ptr<RigidBodies>;
}
