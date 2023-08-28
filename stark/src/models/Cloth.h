#pragma once
#include <vector>
#include <array>

#include <Eigen/Dense>
#include <symx>
#include <TriangleMeshCollisionDetection>

#include "../solver/Stark.h"
#include "../utils/MeshWithDynamics.h"
#include "../utils/MultiMeshEdges.h"
#include "../utils/unordered_array_set_and_map.h"
#include "TriangleMeshContacts.h"
#include "TriangleMeshFriction.h"


namespace stark::models
{
	class Cloth 
	{
	public:
		enum class MaterialPreset { Cotton, Towel };

		/* Fields */
		utils::MeshWithDynamics<3> model;
		symx::DoF dof;
		bool changed_discretization = true;
		bool write_VTK = true;

		// Inertia
		std::vector<std::array<int32_t, 1>> conn_nodes;
		std::vector<double> lumped_mass;  // [kg] per vertex
		std::vector<double> density;  // [kg/m2]  per mesh
		double inertial_damping = 0.0;

		// Strain (Neo-Hookean)
		std::vector<double> young_modulus;  // per mesh
		std::vector<double> poisson_ratio;  // per mesh
		std::vector<std::array<int32_t, 5>> conn_mesh_numbered_triangles;  // [glob_tri_id, mesh_id, i, j, k]
		std::vector<std::array<double, 4>> DXinv;  // per triangle
		std::vector<double> triangle_area_rest;  // per triangle
		double strain_damping = 0.0;

		//// Strain limiting
		bool is_strain_limiting_active = true;
		std::vector<double> strain_limiting_start;  // per mesh
		std::vector<double> strain_limiting_stiffness;  // per mesh
		std::vector<std::array<int32_t, 4>> conn_mesh_numbered_edges;  // [glob_edge_id, mesh_id, i, j]
		std::vector<double> edge_rest_length;  // per edge

		// Bending (Bergou)
		std::vector<double> bending_stiffness;  // per mesh
		std::vector<std::array<int32_t, 6>> conn_numbered_mesh_internal_edges;  // [glob_ie_id, mesh_id, i, j, k, l]
		std::vector<std::array<double, 16>> bergou_Q_matrix;    // per internal angle
		double bending_damping = 0.0;

		// Prescribed positions
		std::unordered_map<int, Eigen::Vector3d> prescribed_nodes_map;
		std::vector<Eigen::Vector3d> prescribed_positions;
		std::vector<std::array<int32_t, 2>> conn_enumerated_prescribed_positions;
		bool changed_prescribed_vertices = true;

		// Attachments
		utils::unordered_array_set<int, 2> attached_nodes_set;
		std::vector<std::array<int32_t, 2>> conn_attached_nodes;
		bool changed_attachments = true;

		// Contacts
		utils::MultiMeshEdges edges;
		tmcd::IntersectionDetection id;
		tmcd::ProximityDetection pd;
		TriangleMeshContacts contacts;
		std::vector<Eigen::Vector3d> collision_x1;

		// Friction
		utils::unordered_array_map<int, 2, double> mu;
		TriangleMeshFriction friction;

		/* Methods */
		void init(Stark& sim);

		// Cloth interface
		//// Setters
		int add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const MaterialPreset material = MaterialPreset::Towel);
		void set_damping(const double inertial_damping = 2.0, const double strain_damping = 0.1, const double bending_damping = 0.1);
		void set_material_preset(const int cloth_id, const MaterialPreset material);
		void set_density(const int cloth_id, const double density = 0.5);
		void set_strain_parameters(const int cloth_id, const double young_modulus = 1e3, const double poisson_ratio = 0.3, const double strain_limit = 1.10, const double strain_limit_stiffness = 1.0);
		void set_bending_stiffness(const int cloth_id, const double bending_stiffness = 1e-5);
		void set_friction(const int cloth_0, const int cloth_1, const double coulombs_mu);
		double get_friction(const int cloth_0, const int cloth_1);
		void enable_writing_vtk(const bool write = true);

		void set_vertex_target_position_as_initial(const int cloth_id, const int vertex_id);
		void set_vertex_target_position(const int cloth_id, const int vertex_id, const Eigen::Vector3d& position);
		void set_attached_vertices(const int cloth_0_id, const int vertex_0_idx, const int cloth_1_id, const int vertex_1_idx);
		void freeze(const int cloth_id);
		void clear_vertex_target_position();
		void clear_attached_vertices();

		void set_acceleration(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& acceleration);
		void add_acceleration(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& acceleration);
		void set_velocity(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& velocity);
		void add_velocity(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& velocity);
		void set_position(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& position);
		void add_displacement(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& displacement);
		void clear_acceleration();

		//// Getters
		bool is_cloth_declared(const int cloth_id) const;
		int	get_n_cloths() const;
		bool is_empty() const;
		Eigen::Vector3d get_vertex(const int cloth_id, const int vertex_idx) const;
		Eigen::Vector3d get_velocity(const int cloth_id, const int vertex_idx) const;
		const utils::TriangleMultiMesh& get_mesh() const;

	private:
		// Helpers
		void _exit_if_cloth_not_declared(const int cloth_id);
		void _init_simulation_structures(const int n_threads);
		void _update_collision_x1(Stark& sim);
		const tmcd::ProximityResults& _run_proximity_detection(const std::vector<Eigen::Vector3d>& x, Stark& sim);
		void _update_contacts(Stark& sim);
		void _update_friction_contacts(Stark& sim);
		
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