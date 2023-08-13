#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <cassert>

#include <Eigen/Dense>


namespace stark::utils
{
	/* =========================  DEFINITIONS  ========================= */
	template<std::size_t N>
	class MultiMesh
	{
	public:
		/* Fields */
		std::vector<Eigen::Matrix<double, 3, 1>> vertices;
		std::vector<int> vertices_offsets;
		std::vector<std::array<int, N>> connectivity;
		std::vector<int> connectivity_offsets;

		/* Methods */
		MultiMesh();
		~MultiMesh() {};

		int add_mesh(const std::vector<Eigen::Matrix<double, 3, 1>>& vertices, const std::vector<std::array<int, N>>& connectivity);
		void get_connectivity_local(std::vector<std::array<int, N>>& connectivity_out, const int mesh_i) const;
		Eigen::VectorXd get_concat_vertices() const;
		int get_n_meshes() const;
		int get_n_elements(const int mesh_i) const;
		int get_n_vertices(const int mesh_i) const;
		int get_n_elements() const;
		int get_n_vertices() const;
		std::array<int, 2> get_elements_range(const int mesh_i) const;
		std::array<int, 2> get_vertices_range(const int mesh_i) const;
		int get_mesh_containing_vertex(const int vertex_i) const;
		int get_mesh_containing_element(const int element_i) const;
		int get_local_vertex_idx(const int mesh_i, const int vertex_i) const;
		int get_local_element_idx(const int mesh_i, const int element_i) const;
		int get_global_vertex_idx(const int mesh_i, const int local_vertex_i) const;
		int get_global_element_idx(const int mesh_i, const int local_element_i) const;
		void clear();

		template<typename FUNCTION> // void(int elem_glob_idx, std::array<int, N>& elem_glob, std::array<Eigen::Vector3d*, N>& vertices)
		void for_each_element_parallel_const(const int mesh_i, FUNCTION callback, const int n_threads = 1) const;
	};





	/* =========================  IMPLEMENTATION  ========================= */
	template<std::size_t N>
	inline MultiMesh<N>::MultiMesh()
	{
		this->clear();
	}
	template<std::size_t N>
	inline int MultiMesh<N>::add_mesh(const std::vector<Eigen::Matrix<double, 3, 1>>& vertices, const std::vector<std::array<int, N>>& connectivity)
	{
		// Append connectivity
		const int offset = (int)this->vertices.size();
		for (const std::array<int, N>& element : connectivity) {
			std::array<int, N> element_offset = element;
			for (int& idx : element_offset) {
				idx += offset;
			}
			this->connectivity.push_back(element_offset);
		}
		this->connectivity_offsets.push_back((int)this->connectivity.size());

		// Append vertices
		this->vertices.insert(this->vertices.end(), vertices.begin(), vertices.end());
		this->vertices_offsets.push_back((int)this->vertices.size());

		return this->get_n_meshes() - 1;
	}
	template<std::size_t N>
	inline void MultiMesh<N>::get_connectivity_local(std::vector<std::array<int, N>>& connectivity_out, const int mesh_i) const
	{
		const int elem_start = this->connectivity_offsets[mesh_i];
		const int elem_end = this->connectivity_offsets[mesh_i + 1];
		const int n_elems = elem_end - elem_start;
		const int offset = this->vertices_offsets[mesh_i];
		for (int elem_i = elem_start; elem_i < elem_end; elem_i++) {
			std::array<int, N> elem = this->connectivity[elem_i];
			for (int i = 0; i < N; i++) {
				elem[i] -= offset;
			}
			connectivity_out.push_back(elem);
		}
	}
	template<std::size_t N>
	inline Eigen::VectorXd MultiMesh<N>::get_concat_vertices() const
	{
		Eigen::VectorXd x(3*this->get_n_vertices());
		for (int i = 0; i < this->get_n_vertices(); i++) {
			x.segment(3*i, 3) = this->vertices[i];
		}
		return x;
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_n_meshes() const
	{
		return (int)this->vertices_offsets.size() - 1;
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_n_elements(const int mesh_i) const
	{
		return this->connectivity_offsets[mesh_i + 1] - this->connectivity_offsets[mesh_i];
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_n_elements() const
	{
		return (int)this->connectivity.size();
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_n_vertices() const
	{
		return (int)this->vertices.size();
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_n_vertices(const int mesh_i) const
	{
		return this->vertices_offsets[mesh_i + 1] - this->vertices_offsets[mesh_i];
	}
	template<std::size_t N>
	inline std::array<int, 2> MultiMesh<N>::get_elements_range(const int mesh_i) const
	{
		return { this->connectivity_offsets[mesh_i], this->connectivity_offsets[mesh_i + 1] };
	}
	template<std::size_t N>
	inline std::array<int, 2> MultiMesh<N>::get_vertices_range(const int mesh_i) const
	{
		return { this->vertices_offsets[mesh_i], this->vertices_offsets[mesh_i + 1] };
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_mesh_containing_vertex(const int vertex_i) const
	{
		const auto it = std::binary_search(this->vertices_offsets.begin(), this->vertices_offsets.end(), vertex_i);
		if (it == this->vertices_offsets.end()) {
			std::cout << "stb error: MultiMesh.get_mesh_containing_vertex() out of bounds." << std::endl;
			exit(-1);
		}
		else {
			return (int)std::distance(this->vertices_offsets.begin(), it);
		}
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_mesh_containing_element(const int element_i) const
	{
		for (int i = 1; i < (int)this->connectivity_offsets.size(); i++) {
			if (element_i < this->connectivity_offsets[i]) {
				return i - 1;
			}
		}
		std::cout << "stb error: MultiMesh.get_mesh_containing_element() out of bounds." << std::endl;
		exit(-1);
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_local_vertex_idx(const int mesh_i, const int vertex_i) const
	{
		assert(mesh_i < this->get_n_meshes());
		const int idx = vertex_i - this->vertices_offsets[mesh_i];
		assert(idx >= 0 && idx < this->get_n_vertices(mesh_i));
		return idx;
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_local_element_idx(const int mesh_i, const int element_i) const
	{
		assert(mesh_i < this->get_n_meshes());
		const int idx = element_i - this->connectivity_offsets[mesh_i];
		assert(idx >= 0 && idx < this->get_n_elements(mesh_i));
		return idx;
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_global_vertex_idx(const int mesh_i, const int local_vertex_i) const
	{
		return this->vertices_offsets[mesh_i] + local_vertex_i;
	}
	template<std::size_t N>
	inline int MultiMesh<N>::get_global_element_idx(const int mesh_i, const int local_element_i) const
	{
		return  this->connectivity_offsets[mesh_i] + local_element_i;
	}
	template<std::size_t N>
	inline void MultiMesh<N>::clear()
	{
		this->vertices.clear();
		this->connectivity.clear();
		this->vertices_offsets.clear();
		this->connectivity_offsets.clear();
		this->vertices_offsets.push_back(0);
		this->connectivity_offsets.push_back(0);
	}
	template<std::size_t N>
	template<typename FUNCTION>
	inline void MultiMesh<N>::for_each_element_parallel_const(const int mesh_i, FUNCTION callback, const int n_threads) const
	{
		const std::array<int, 2> range = this->get_elements_range(mesh_i);
		#pragma omp parallel for schedule(static) num_threads(n_threads)
		for (int elem_i = range[0]; elem_i < range[1]; elem_i++) {
			const std::array<int, N> elem = this->connectivity[elem_i];
			std::array<const Eigen::Vector3d*, N> vertices;
			for (int loc_vertex_i = 0; loc_vertex_i < (int)N; loc_vertex_i++) {
				vertices[loc_vertex_i] = &this->vertices[elem[loc_vertex_i]];
			}
			callback(elem_i, elem, vertices);
		}
	}
	using TriangleMultiMesh = MultiMesh<3>;
	using TetraMultiMesh = MultiMesh<4>;
}
