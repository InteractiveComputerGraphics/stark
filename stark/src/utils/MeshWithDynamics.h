#pragma once
#include <vector>
#include <array>

#include <Eigen/Dense>

#include "MultiMesh.h"


namespace stark::utils
{
	template<int N_NODES_PER_ELEM>
	class MeshWithDynamics
	{
	public:
		/* Fields */
		// Dynamics
		std::vector<Eigen::Vector3d> X;  // Node positions at rest
		std::vector<Eigen::Vector3d> x0;  // Node positions at time n
		std::vector<Eigen::Vector3d> x1;  // Node positions at time n+1
		std::vector<Eigen::Vector3d> v0;  // Node velocities at time n
		std::vector<Eigen::Vector3d> v1;  // Node velocities at time n+1
		std::vector<Eigen::Vector3d> a;	  // Node accelerations

		// Concatenated meshes
		MultiMesh<N_NODES_PER_ELEM> mesh;

		/* Methods */
		MeshWithDynamics() = default;
		~MeshWithDynamics() = default;
		int add_mesh(
			const std::vector<Eigen::Vector3d>& vertices,
			const std::vector<std::array<int, N_NODES_PER_ELEM>>& connectivity);
		void set_acceleration(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& acceleration);
		void add_acceleration(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& acceleration);
		void set_velocity(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& velocity);
		void add_velocity(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& velocity);
		void set_position(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& position);
		void add_displacement(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& displacement);
		void clear();
	};

	// ================================= DEFINITIONS =================================
	template<int N_NODES_PER_ELEM>
	inline int MeshWithDynamics<N_NODES_PER_ELEM>::add_mesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, N_NODES_PER_ELEM>>& connectivity)
	{
		// Dynamics
		this->mesh.add_mesh(vertices, connectivity);
		const int n = (int)this->mesh.vertices.size();

		this->X = this->mesh.vertices;
		this->x0 = this->mesh.vertices;
		this->x1 = this->mesh.vertices;

		this->v0.resize(n, Eigen::Vector3d::Zero());
		this->v1.resize(n, Eigen::Vector3d::Zero());
		this->a.resize(n, Eigen::Vector3d::Zero());

		return this->mesh.get_n_meshes() - 1;
	}
	template<int N_NODES_PER_ELEM>
	inline void MeshWithDynamics<N_NODES_PER_ELEM>::set_acceleration(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& acceleration)
	{
		this->a[this->mesh.get_global_vertex_idx(body_id, loc_vertex_id)] = acceleration;
	}
	template<int N_NODES_PER_ELEM>
	inline void MeshWithDynamics<N_NODES_PER_ELEM>::add_acceleration(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& acceleration)
	{
		this->a[this->mesh.get_global_vertex_idx(body_id, loc_vertex_id)] += acceleration;
	}
	template<int N_NODES_PER_ELEM>
	inline void MeshWithDynamics<N_NODES_PER_ELEM>::set_velocity(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& velocity)
	{
		const int glob_vertex_id = this->mesh.get_global_vertex_idx(body_id, loc_vertex_id);
		this->v0[glob_vertex_id] = velocity;
		this->v1[glob_vertex_id] = velocity;
	}
	template<int N_NODES_PER_ELEM>
	inline void MeshWithDynamics<N_NODES_PER_ELEM>::add_velocity(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& velocity)
	{
		const int glob_vertex_id = this->mesh.get_global_vertex_idx(body_id, loc_vertex_id);
		this->v0[glob_vertex_id] += velocity;
		this->v1[glob_vertex_id] += velocity;
	}
	template<int N_NODES_PER_ELEM>
	inline void MeshWithDynamics<N_NODES_PER_ELEM>::set_position(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& position)
	{
		const int glob_vertex_id = this->mesh.get_global_vertex_idx(body_id, loc_vertex_id);
		this->x0[glob_vertex_id] = position;
		this->x1[glob_vertex_id] = position;
	}
	template<int N_NODES_PER_ELEM>
	inline void MeshWithDynamics<N_NODES_PER_ELEM>::add_displacement(const int body_id, const int loc_vertex_id, const Eigen::Vector3d& displacement)
	{
		const int glob_vertex_id = this->mesh.get_global_vertex_idx(body_id, loc_vertex_id);
		this->x0[glob_vertex_id] += displacement;
		this->x1[glob_vertex_id] += displacement;
	}
	template<int N_NODES_PER_ELEM>
	inline void MeshWithDynamics<N_NODES_PER_ELEM>::clear()
	{
		this->mesh.clear();
		this->X.clear();
		this->x0.clear();
		this->x1.clear();
		this->v0.clear();
		this->v1.clear();
		this->a.clear();
	}
}