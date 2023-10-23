#pragma once
#include <vector>
#include <array>

#include <Eigen/Dense>
#include <symx>

#include "../solver/Stark.h"
#include "../utils/MeshWithDynamics.h"
#include "../utils/MultiMeshEdges.h"
#include "../utils/unordered_array_set_and_map.h"


namespace stark::models
{
	class DeformableSolids 
	{
	public:

		/* Fields */
		utils::MeshWithDynamics<4> model;
		symx::DoF dof;
		bool changed_discretization = true;

		// Inertia
		std::vector<std::array<int32_t, 1>> conn_nodes;
		std::vector<double> lumped_mass;  // [kg] per vertex
		std::vector<double> density;  // [kg/m3]  per mesh
		double inertial_damping = 0.0;

		// Strain (Neo-Hookean)
		std::vector<double> young_modulus;  // per mesh
		std::vector<double> poisson_ratio;  // per mesh
		std::vector<std::array<int32_t, 6>> conn_mesh_numbered_tets;  // [glob_tri_id, mesh_id, a, b, c, d]

		// Prescribed positions
		std::unordered_map<int, Eigen::Vector3d> prescribed_nodes_map;
		std::vector<Eigen::Vector3d> prescribed_positions;
		std::vector<std::array<int32_t, 2>> conn_enumerated_prescribed_positions;
		bool changed_prescribed_vertices = true;

		/* Methods */
		void init(Stark& sim);

		// Cloth interface
		//// Setters
		int add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 4>>& tets);
		void set_damping(const double inertial_damping = 2.0);
		void set_density(const int body_id, const double density = 0.5);
		void set_strain_parameters(const int body_id, const double young_modulus = 1e3, const double poisson_ratio = 0.3);

		void set_vertex_target_position_as_initial(const int body_id, const int vertex_id);
		void set_vertex_target_position(const int body_id, const int vertex_id, const Eigen::Vector3d& position);
		void freeze(const int body_id);
		void clear_vertex_target_position();

		void set_acceleration(const int body_id, const int vertex_idx, const Eigen::Vector3d& acceleration);
		void add_acceleration(const int body_id, const int vertex_idx, const Eigen::Vector3d& acceleration);
		void set_velocity(const int body_id, const int vertex_idx, const Eigen::Vector3d& velocity);
		void add_velocity(const int body_id, const int vertex_idx, const Eigen::Vector3d& velocity);
		void set_position(const int body_id, const int vertex_idx, const Eigen::Vector3d& position);
		void add_displacement(const int body_id, const int vertex_idx, const Eigen::Vector3d& displacement);
		void clear_acceleration();

		//// Getters
		int	get_n_bodies() const;
		bool is_empty() const;
		Eigen::Vector3d get_vertex(const int body_id, const int vertex_idx) const;
		Eigen::Vector3d get_velocity(const int body_id, const int vertex_idx) const;

	private:
		// Helpers
		void _init_simulation_structures(const int n_threads);
		
		// Stark callbacks
		void _before_time_step(Stark& sim);
		void _after_time_step(Stark& sim);
		void _write_frame(Stark& sim);

		// Energy groups
		void _energies_mechanical(Stark& sim);
	};
}