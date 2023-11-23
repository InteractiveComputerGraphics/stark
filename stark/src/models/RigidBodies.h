#pragma once
#include <vector>
#include <array>
#include <unordered_map>
#include <memory>

#include <Eigen/Dense>
#include <symx>

#include "../solver/Stark.h"
#include "../utils/MultiMesh/MultiMesh_.h"
#include "deformables/MeshOutputGroups.h"
#include "../utils/mesh_generators.h"
#include "../utils/inertia_tensors.h"

#include "deformables/Id.h"
#include "RigidBodyDynamics.h"
#include "EnergyRigidBodyInertia.h"
#include "EnergyRigidBodyConstraints.h"
#include "rigidbody_constraints_ui.h"


namespace stark::models
{
	class RigidBodies 
	{
	public:
		/* Fields */
		spRigidBodyDynamics dyn;
		spEnergyRigidBodyInertia energy_inertia;
		spEnergyRigidBodyConstraints energy_constraints;

		// Meshes
		std::vector<utils::Mesh<3>> collision_meshes;  // Always stays at rest positions
		std::vector<utils::Mesh<3>> render_meshes;  // Always stays at rest positions

		// Output
		MeshOutputGroups output_groups;  // local_indices
		bool write_render_mesh = true;
		bool write_collision_mesh = false;
		bool write_transformation_sequences = false;


		/* Methods */
		RigidBodies(Stark& stark, spRigidBodyDynamics dyn);
		Id add(const double mass, const Eigen::Matrix3d& inertia_loc, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });

		Id add_sphere(const double mass, const double radius, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int subdivisions = 2);
		Id add_box(const double mass, const Eigen::Vector3d& size, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		Id add_cylinder(const double mass, const double radius, const double full_height, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int slices = 16, const int stacks = 1);
		Id add_torus(const double mass, const double outer_radius, const double inner_radius, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 }, const int slices = 16, const int stacks = 32);

		void set_collision_mesh(const Id& id, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles);
		void set_render_mesh(const Id& id, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles);

		AnchorPointHandler add_constraint_anchor_point(const Id& body_id, const Eigen::Vector3d& p_glob);

		void add_constraint_ball_joint(const int body_0, const int body_1, const Eigen::Vector3d& p_glob);
		void add_constraint_hinge_joint(const int& body_0, const int& body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global);
		void add_constraint_slider(const int body_0, const int body_1, const Eigen::Vector3d& p0_global, const Eigen::Vector3d& p1_global, const double spring_stiffness = 0.0, const double spring_damping = 0.0);
		void add_constraint_relative_direction_lock(const int body_0, const int body_1, const Eigen::Vector3d& d_global);
		void add_constraint_relative_direction_lock(const int body_0, const int body_1);
		void add_constraint_freeze(const int body_id);
		void add_constraint_motor(const int body_0, const int body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global, const double max_torque, const double target_w, const double delay = 0.01);
		void add_constraint_parallel_gripper(const int base, const int right_finger, const int left_finger, const Eigen::Vector3d& d_global, const double max_force, const double closing_velocity, const double delay = 0.01);

		int get_n_bodies() const;
		bool is_empty() const;
		bool is_body_declared(const int body_id) const;

	private:
		// Stark callbacks
		void _write_frame(Stark& sim);
	};
	using spRigidBodies = std::shared_ptr<RigidBodies>;
}
