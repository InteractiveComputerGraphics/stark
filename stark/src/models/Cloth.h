#pragma once
#include <unordered_map>

#include <symx>
#include <TriangleMeshCollisionDetection>
#include <vtkio>

#include "../solver/Stark.h"
#include "../utils/MeshWithDynamics.h"
#include "../utils/unordered_array_set_and_map.h"


namespace stark::models
{
	class Cloth 
	{
	public:
		enum class MaterialPreset { Cotton, Wool, Silk, RubberShell };
		struct ClothSelfContacts
		{
			std::vector<std::array<int32_t, 2>> point_point;
			std::vector<std::array<int32_t, 3>> point_edge;
			std::vector<std::array<int32_t, 4>> point_triangle;
			//std::vector<std::array<int32_t, 4>> edge_edge;
		};


		/* Fields */
		utils::MeshWithDynamics<3> model;
		bool changed_discretization = true;
		bool write_VTK = true;

		// Inertia
		std::vector<std::array<int32_t, 1>> conn_nodes;
		std::vector<double> lumped_mass;  // [kg] per vertex
		std::vector<double> density;  // [kg/m2]  per mesh
		double damping = 0.0;

		// Strain (Neo-Hookean)
		std::vector<double> young_modulus;  // per mesh
		std::vector<double> poisson_ratio;  // per mesh
		std::vector<std::array<int32_t, 5>> conn_mesh_numbered_triangles;  // [glob_tri_id, mesh_id, i, j, k]
		std::vector<std::array<double, 4>> DXinv;  // per triangle
		std::vector<double> triangle_area_rest;  // per triangle

		//// Strain limiting
		bool is_strain_limiting_active = true;
		std::vector<double> strain_limiting_start;  // per mesh
		std::vector<double> strain_limiting_stiffness;  // per mesh

		// Bending (Bergou)
		std::vector<double> bending_stiffness;  // per mesh
		std::vector<std::array<int32_t, 6>> conn_numbered_mesh_internal_edges;  // [glob_ie_id, mesh_id, i, j, k, l]
		std::vector<std::array<double, 16>> bergou_Q_matrix;    // per internal angle

		// Prescribed positions
		std::unordered_map<int, Eigen::Vector3d> prescribed_nodes_map;
		std::vector<Eigen::Vector3d> prescribed_positions;
		std::vector<std::array<int32_t, 2>> conn_enumerated_prescribed_positions;
		bool changed_prescribed_vertices = true;

		// Attachments
		utils::unordered_array_set<int, 2> attached_nodes_set;
		std::vector<std::array<int32_t, 2>> conn_attached_nodes;
		bool changed_attachments = true;

		// Collisions
		std::vector<std::array<int32_t, 2>> edges;
		std::vector<Eigen::Vector3d> collision_x;
		tmcd::IntersectionDetection id;
		tmcd::ProximityDetection pd;
		ClothSelfContacts contacts;
		std::vector<std::array<int32_t, 6>> ee_point_point;
		std::vector<std::array<int32_t, 5>> ee_point_edge;
		std::vector<std::array<int32_t, 4>> ee_edge_edge;

		/* Methods */
		// Physical System Interface
		void init(Stark& sim);

		// Cloth interface
		//// Setters
		int add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const MaterialPreset material = MaterialPreset::Cotton);
		void set_damping(const double damping);
		void set_material_preset(const int cloth_id, const MaterialPreset material);
		void set_density(const int cloth_id, const double density = 0.5);
		void set_strain_parameters(const int cloth_id, const double young_modulus = 1e3, const double poisson_ratio = 0.3, const double strain_limit = 1.10, const double strain_limit_stiffness = 1e6);
		void set_bending_stiffness(const int cloth_id, const double bending_stiffness = 1e-5);
		//void set_contact_parameters(const bool activate, const double dhat = 0.005, const double stiffness = 1e6);
		void enable_writing_vtk(const bool write = true);

		void set_vertex_target_position_as_initial(const int cloth_id, const int vertex_id);
		void set_vertex_target_position(const int cloth_id, const int vertex_id, const Eigen::Vector3d& position);
		void set_attached_vertices(const int cloth_0_id, const int vertex_0_idx, const int cloth_1_id, const int vertex_1_idx);
		void clear_vertex_target_position();
		void clear_attached_vertices();

		void set_acceleration(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& acceleration);
		void add_acceleration(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& acceleration);
		void set_velocity(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& velocity);
		void add_velocity(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& velocity);
		void set_position(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& position);
		void add_displacement(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& displacement);

		//// Getters
		bool is_cloth_declared(const int cloth_id) const;
		int	get_n_cloths() const;
		Eigen::Vector3d get_vertex(const int cloth_id, const int vertex_idx) const;
		Eigen::Vector3d get_velocity(const int cloth_id, const int vertex_idx) const;
		const utils::TriangleMultiMesh& get_mesh() const;

	private:
		// Stark callbacks
		void _before_time_step(Stark& sim);
		void _after_time_step(Stark& sim);
		bool _is_valid_configuration(Stark& sim);
		void _write_frame(Stark& sim);

		// Helpers
		void _exit_if_cloth_not_declared(const int cloth_id);
		void _init_simulation_structures(const int n_threads);
		void _update_collision_x(Stark& sim);
		void _update_contacts(Stark& sim);
	};
}