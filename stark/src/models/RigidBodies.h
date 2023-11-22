#pragma once
#include <vector>
#include <array>
#include <unordered_map>
#include <memory>

#include <Eigen/Dense>
#include <symx>

#include "../solver/Stark.h"
#include "../utils/Logger.h"
#include "RigidBodyDynamics.h"
#include "deformables/MeshOutputGroups.h"
#include "deformables/Id.h"
#include "../utils/MultiMesh/MultiMesh_.h"
//#include "../utils/MultiMeshEdges.h"
#include "../utils/mesh_generators.h"
#include "../utils/inertia_tensors.h"
//#include "../utils/unordered_array_set_and_map.h"


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
			std::vector<double> delay;
			std::vector<std::array<int, 3>> conn; // { "idx", "a", "b" }
		};
		struct ParallelGripper
		{
			std::vector<Eigen::Vector3d> loc_da;
			std::vector<double> max_force;
			std::vector<double> target_v;
			std::vector<double> delay;
			std::vector<std::array<int, 3>> conn; // { "idx", "a", "b" }
		};
		struct Constraints
		{
			AnchorPoints anchor_points;
			BallJoints ball_joints;
			RelativeDirectionLock relative_direction_lock;
			Sliders sliders;
			Motors motors;
			ParallelGripper parallel_gripper;
		};

	public:
		/* Fields */
		spRigidBodyDynamics rb_dyn;

		// Inertia
		std::vector<std::array<double, 9>> J0_glob;  // Inertia tensor local coordinates
		std::vector<std::array<double, 9>> J0_inv_glob;  // Inverse inertia tensor local coordinates
		symx::LabelledConnectivity<1> conn_intertia{ { "rb" } };
		double inertia_damping = 0.0;

		// Constraints
		Constraints constraints;
		double constraint_stiffness = 1e6;
		std::vector<Eigen::Vector3d> motor_torque;
		utils::Logger constraint_logger;

		// Output
		MeshOutputGroups output_groups;  // local_indices
		utils::MultiMesh_<3> collision_mesh;  // Always stays at rest positions
		utils::MultiMesh_<3> render_mesh;  // Always stays at rest positions


		/* Methods */
		RigidBodies(Stark& stark, spRigidBodyDynamics rb_dyn);
		Id add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double mass, const Eigen::Matrix3d& inertia_loc);
		Id add_and_transform(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double mass, const Eigen::Matrix3d& inertia_loc, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });

		Id add_sphere(const double mass, const double radius, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int subdivisions = 2);
		Id add_box(const double mass, const Eigen::Vector3d& size, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		Id add_cylinder(const double mass, const double radius, const double full_height, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int slices = 16, const int stacks = 1);
		Id add_torus(const double mass, const double outer_radius, const double inner_radius, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int slices = 16, const int stacks = 32);

		Id add_sphere(const double mass, const double radius, const std::vector<Eigen::Vector3d>& collision_vertices, const std::vector<std::array<int, 3>>& collision_triangles, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		Id add_box(const double mass, const Eigen::Vector3d& size, const std::vector<Eigen::Vector3d>& collision_vertices, const std::vector<std::array<int, 3>>& collision_triangles, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		Id add_cylinder(const double mass, const double radius, const double full_height, const std::vector<Eigen::Vector3d>& collision_vertices, const std::vector<std::array<int, 3>>& collision_triangles, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		Id add_torus(const double mass, const double outer_radius, const double inner_radius, const std::vector<Eigen::Vector3d>& collision_vertices, const std::vector<std::array<int, 3>>& collision_triangles, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });

		void add_constraint_anchor_point(const int body_id, const Eigen::Vector3d& p_glob);
		void add_constraint_ball_joint(const int body_0, const int body_1, const Eigen::Vector3d& p_glob);
		void add_constraint_hinge_joint(const int body_0, const int body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global);
		void add_constraint_slider(const int body_0, const int body_1, const Eigen::Vector3d& p0_global, const Eigen::Vector3d& p1_global, const double spring_stiffness = 0.0, const double spring_damping = 0.0);
		void add_constraint_relative_direction_lock(const int body_0, const int body_1, const Eigen::Vector3d& d_global);
		void add_constraint_relative_direction_lock(const int body_0, const int body_1);
		void add_constraint_freeze(const int body_id);
		void add_constraint_motor(const int body_0, const int body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global, const double max_torque, const double target_w, const double delay = 0.01);
		void add_constraint_parallel_gripper(const int base, const int right_finger, const int left_finger, const Eigen::Vector3d& d_global, const double max_force, const double closing_velocity, const double delay = 0.01);

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
		void set_friction(const int body_0, const int body_1, const double coulombs_mu);
		double get_friction(const int body_0, const int body_1);

		Eigen::Vector3d get_point_in_global_coordinates(const int body_id, const Eigen::Vector3d& p);
		Eigen::Vector3d get_point_in_global_coordinates(const int body_id, const int mesh_vertex_i);
		int get_n_bodies() const;
		bool is_empty() const;
		bool is_body_declared(const int body_id) const;

	private:
		// Stark callbacks
		void _before_time_step(Stark& sim);
		void _after_time_step(Stark& sim);
		void _write_frame(Stark& sim);
	};
	using spRigidBodies = std::shared_ptr<RigidBodies>;
}
