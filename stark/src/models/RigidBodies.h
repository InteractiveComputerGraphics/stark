#pragma once
#include <vector>
#include <array>

#include <Eigen/Dense>
#include <symx>
#include <TriangleMeshCollisionDetection>

#include "../solver/Stark.h"
#include "../utils/MultiMesh.h"
#include "../utils/mesh_generators.h"
#include "../utils/inertia_tensors.h"
#include "RigidBodyContacts.h"
#include "RigidBodyFriction.h"


namespace stark::models
{
	class RigidBodies 
	{
	private:
		struct AnchorPoints
		{
			std::vector<Eigen::Vector3d> loc;
			std::vector<Eigen::Vector3d> target;
			std::vector<std::array<int, 2>> conn; // { "idx", "a" }
		};
		struct BallJoints
		{
			std::vector<Eigen::Vector3d> loc_a;
			std::vector<Eigen::Vector3d> loc_b;
			std::vector<std::array<int, 3>> conn; // { "idx", "a", "b" }
		};
		struct RelativeDirectionLock
		{
			std::vector<Eigen::Vector3d> loc_da;
			std::vector<Eigen::Vector3d> loc_db;
			std::vector<std::array<int, 3>> conn; // { "idx", "a", "b" }
		};
		struct Sliders
		{
			std::vector<Eigen::Vector3d> loc_a;
			std::vector<Eigen::Vector3d> loc_b;
			std::vector<Eigen::Vector3d> loc_da;
			std::vector<double> rest_length;
			std::vector<double> spring_stiffness;
			std::vector<double> spring_damping;
			std::vector<std::array<int, 3>> conn; // { "idx", "a", "b" }
		};
		struct Motors
		{
			std::vector<Eigen::Vector3d> loc_da;
			std::vector<double> max_torque;
			std::vector<double> target_w;
			std::vector<double> correction_stiffness;
			std::vector<std::array<int, 3>> conn; // { "idx", "a", "b" }
		};
		struct Constraints
		{
			AnchorPoints anchor_points;
			BallJoints ball_joints;
			RelativeDirectionLock relative_direction_lock;
			Sliders sliders;
			Motors motors;
		};

	public:
		/* Fields */
		std::vector<Eigen::Vector3d> t0;  // Translation at time n
		std::vector<Eigen::Vector3d> t1;  // Translation at time n+1
		std::vector<Eigen::Quaterniond> q0;  // Quaternion at time n
		std::vector<std::array<double, 4>> q0_;  // Quaternion at time n (for SymX)
		std::vector<Eigen::Quaterniond> q1;  // Quaternion at time n+1
		std::vector<Eigen::Matrix3d> R0;  // Rotation matrix at time n
		std::vector<Eigen::Matrix3d> R1;  // Rotation matrix at time n+1
		std::vector<Eigen::Vector3d> v0;  // Velocity at time n
		std::vector<Eigen::Vector3d> v1;  // Velocity at time n+1
		std::vector<Eigen::Vector3d> w0;  // Angular velocity at time n
		std::vector<Eigen::Vector3d> w1;  // Angular velocity at time n+1
		std::vector<Eigen::Vector3d> a;  // Acceleration
		std::vector<Eigen::Vector3d> aa;  // Angular acceleration
		std::vector<Eigen::Vector3d> force;  // Forces
		std::vector<Eigen::Vector3d> torque;  // Torques
		std::vector<Eigen::Matrix3d> J_loc;  // Inertia tensor local coordinates
		std::vector<double> mass;
		symx::DoF dof_v;
		symx::DoF dof_w;
		bool write_VTK = true;

		// Inertia
		std::vector<std::array<double, 9>> J0_glob;  // Inertia tensor local coordinates
		std::vector<std::array<double, 9>> J0_inv_glob;  // Inverse inertia tensor local coordinates
		std::vector<std::array<int32_t, 1>> conn_inertia;
		double damping = 0.0;

		// Constraints
		Constraints constraints;
		double constraint_stiffness = 1e6;
		std::vector<Eigen::Vector3d> motor_torque;

		// Contacts
		std::vector<std::array<int32_t, 2>> edges;
		tmcd::IntersectionDetection id;
		tmcd::ProximityDetection pd;
		RigidBodyContacts contacts;
		std::vector<Eigen::Vector3d> collision_x1;

		// Friction
		RigidBodyFriction friction;
		std::vector<double> mu;

		// Output
		utils::MultiMesh<3> mesh;  // Always stays at rest positions


		/* Methods */
		void init(Stark& sim);
		int add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double mass, const Eigen::Matrix3d& inertia_loc);
		int add_and_transform(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double mass, const Eigen::Matrix3d& inertia_loc, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });

		int add_sphere(const double mass, const double radius, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int subdivisions = 2);
		int add_box(const double mass, const Eigen::Vector3d& size, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		int add_cylinder(const double mass, const double radius, const double full_height, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int slices = 16, const int stacks = 1);
		int add_torus(const double mass, const double outer_radius, const double inner_radius, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int slices = 16, const int stacks = 32);

		int add_sphere(const double mass, const double radius, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		int add_box(const double mass, const Eigen::Vector3d& size, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		int add_cylinder(const double mass, const double radius, const double full_height, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		int add_torus(const double mass, const double outer_radius, const double inner_radius, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });

		void add_constraint_anchor_point(const int body_id, const Eigen::Vector3d& p_glob);
		void add_constraint_ball_joint(const int body_0, const int body_1, const Eigen::Vector3d& p_glob);
		void add_constraint_hinge_joint(const int body_0, const int body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global);
		void add_constraint_slider(const int body_0, const int body_1, const Eigen::Vector3d& p0_global, const Eigen::Vector3d& p1_global, const double spring_stiffness = 0.0, const double spring_damping = 0.0);
		void add_constraint_relative_direction_lock(const int body_0, const int body_1, const Eigen::Vector3d& d_global);
		void add_constraint_freeze(const int body_id);
		void add_constraint_motor(const int body_0, const int body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global, const double max_torque, const double target_w, const double correction_stiffness);

		void set_damping(const double damping);
		void set_displacement(const int body_id, const Eigen::Vector3d& displacement);
		void add_displacement(const int body_id, const Eigen::Vector3d& displacement);
		void set_rotation(const int body_id, const Eigen::Quaterniond& q);
		void set_rotation(const int body_id, const double& angle_deg, const Eigen::Vector3d& axis);
		void add_rotation(const int body_id, const Eigen::Quaterniond& q);
		void add_rotation(const int body_id, const double& angle_deg, const Eigen::Vector3d& axis);
		void set_velocity(const int body_id, const Eigen::Vector3d& vel_glob_coords);
		void add_velocity(const int body_id, const Eigen::Vector3d& vel_glob_coords);
		void set_angular_velocity(const int body_id, const Eigen::Vector3d& angular_vel_glob_coords);
		void add_angular_velocity(const int body_id, const Eigen::Vector3d& angular_vel_glob_coords);
		void add_force_at(const int body_id, const Eigen::Vector3d& force_glob_coords, const Eigen::Vector3d& application_point_glob_coords);
		void set_force_at_centroid(const int body_id, const Eigen::Vector3d& force_glob_coords);
		void add_force_at_centroid(const int body_id, const Eigen::Vector3d& force_glob_coords);
		void set_torque(const int body_id, const Eigen::Vector3d& torque_glob_coords);
		void add_torque(const int body_id, const Eigen::Vector3d& torque_glob_coords);
		void set_acceleration(const int body_id, const Eigen::Vector3d& acc_glob_coords);
		void add_acceleration(const int body_id, const Eigen::Vector3d& acc_glob_coords);
		void set_angular_acceleration(const int body_id, const Eigen::Vector3d& ang_acc_glob_coords);
		void add_angular_acceleration(const int body_id, const Eigen::Vector3d& ang_acc_glob_coords);
		void set_friction(const int body_id, const double coulombs_mu);
		Eigen::Vector3d get_point_in_global_coordinates(const int body_id, const Eigen::Vector3d& p);
		Eigen::Vector3d get_point_in_global_coordinates(const int body_id, const int mesh_vertex_i);
		int get_n_bodies() const;
		bool is_empty() const;
		bool is_body_declared(const int body_id) const;

	private:
		// Helpers
		void _update_collision_x1(Stark& sim, const double dt);
		const tmcd::ProximityResults& _run_proximity_detection(const std::vector<Eigen::Vector3d>& x, Stark& sim);
		void _update_contacts(Stark& sim);
		void _update_friction_contacts(Stark& sim);
		void _update_motors(Stark& sim);

		// Stark callbacks
		void _before_time_step(Stark& sim);
		void _after_time_step(Stark& sim);
		bool _is_valid_configuration(Stark& sim);
		void _write_frame(Stark& sim);

		// Energy groups
		void _energies_mechanical(Stark& sim);
		void _energies_contact(Stark& sim);
		void _energies_friction(Stark& sim);
	};
}
