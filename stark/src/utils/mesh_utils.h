#pragma once
#include <vector>
#include <array>
#include <string>
#include <type_traits>


#include <Eigen/Dense>
#include <vtkio>


namespace stark::utils
{
	// General utils
	static constexpr double PI = 3.14159265358979323846;
	double deg2rad(const double deg);
	double rad2deg(const double rad);

	// Load
	void load_obj(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_triangles, const std::string path);

	// Primitives
	Eigen::Vector3d triangle_normal(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
	double triangle_area(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
	double unsigned_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
	double signed_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);

	// Meshes
	//// Topology
	template<std::size_t N>
	void find_edges_from_simplices(std::vector<std::array<int, 2>>& out_edges, const std::vector<std::array<int, N>>& simplices, const int n_nodes);
	template<std::size_t N>
	std::vector<std::array<int, 2>> find_edges_from_simplices(const std::vector<std::array<int, N>>& simplices, const int n_nodes);
	void find_node_node_map_simplex(std::vector<std::vector<int>>& output, const int32_t* connectivity, const int32_t n_simplices, const int32_t n_nodes_per_simplex, const int32_t n_nodes);
	void find_internal_angles(std::vector<std::array<int, 4>>& internal_edges, const std::vector<std::array<int, 3>>& triangles, const int n_nodes);

	//// Other
	void compute_node_normals(std::vector<Eigen::Vector3d>& output, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles);
	void generate_triangular_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_connectivity, const Eigen::Vector2d& bottom, const Eigen::Vector2d& top, const std::array<int, 2>& n_quads_per_dim, const bool randomize = false, const double z = 0.0);
	std::array<int, 2> generate_triangular_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_connectivity, const double x_length, const double y_length, const int n_short_side, const bool randomize = false, const double z = 0.0);
	void generate_tet_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 4>>& out_tets, const Eigen::Vector3d& bottom, const Eigen::Vector3d& top, const std::array<int, 3>& n_quads_per_dim);
	void write_VTK(const std::string path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const bool generate_normals = true);
	std::vector<int> vertices_in_AABB(const std::vector<Eigen::Vector3d>& vertices, const Eigen::Vector3d& bottom, const Eigen::Vector3d& top);

	//// Transformations
	void move(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& translation);
	void rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis);
	void rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot);
	void scale(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& scale);
	void scale(std::vector<Eigen::Vector3d>& points, const double scale);
	void mirror(std::vector<Eigen::Vector3d>& points, const int dim, const double pivot = 0.0);


	// DEFINITIONS ==========================================================================================
	template<std::size_t N>
	inline void find_edges_from_simplices(std::vector<std::array<int, 2>>& out_edges, const std::vector<std::array<int, N>>& simplices, const int n_nodes)
	{
		out_edges.reserve(N * simplices.size());
		for (const std::array<int, N>& simplex : simplices) {
			for (std::size_t i = 0; i < N; i++) {
				for (std::size_t j = i + 1; j < N; j++) {
					out_edges.push_back({ std::min(simplex[i], simplex[j]), std::max(simplex[i], simplex[j]) });
				}
			}
		}
		std::sort(out_edges.begin(), out_edges.end(), [&](const std::array<int, 2>& a, const std::array<int, 2>& b) { return a[0] * n_nodes + a[1] < b[0] * n_nodes + b[1]; });
		auto end_unique = std::unique(out_edges.begin(), out_edges.end());
		out_edges.erase(end_unique, out_edges.end());
	}
	template<std::size_t N>
	std::vector<std::array<int, 2>> find_edges_from_simplices(const std::vector<std::array<int, N>>& simplices, const int n_nodes)
	{
		std::vector<std::array<int, 2>> output;
		find_edges_from_simplices(output, simplices, n_nodes);
		return output;
	}
}