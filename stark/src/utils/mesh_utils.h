#pragma once
#include <vector>
#include <array>
#include <string>

#include <Eigen/Dense>
#include <vtkio>


namespace stark::utils
{
	// Primitives
	Eigen::Vector3d triangle_normal(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
	double triangle_area(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
	double unsigned_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
	double signed_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);

	// Meshes
	//// Topology
	void find_edges(std::vector<std::array<int, 2>>& out_edges, const std::vector<std::array<int, 3>>& triangles, const int n_nodes);
	void find_node_node_map_simplex(std::vector<std::vector<int>>& output, const int32_t* connectivity, const int32_t n_simplices, const int32_t n_nodes_per_simplex, const int32_t n_nodes);
	void find_internal_angles(std::vector<std::array<int, 4>>& internal_edges, const std::vector<std::array<int, 3>>& triangles, const int n_nodes);

	//// Other
	void compute_node_normals(std::vector<Eigen::Vector3d>& output, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles);
	void generate_triangular_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_connectivity, const Eigen::Vector2d& bottom, const Eigen::Vector2d& top, const std::array<int, 2>& n_quads_per_dim, const bool randomize = false, const double z = 0.0);
	void write_VTK(const std::string path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const bool generate_normals = true);

	//// Transformations
	void move(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& translation);
	void rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis);
	void rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot);
	void scale(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& scale);
	void mirror(std::vector<Eigen::Vector3d>& points, const int dim, const double pivot = 0.0);
}