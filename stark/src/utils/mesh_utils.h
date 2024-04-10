#pragma once
#include <vector>
#include <array>
#include <string>
#include <type_traits>
#include <tuple>

#include <Eigen/Dense>
#include <vtkio>

#define _USE_MATH_DEFINES
#include <math.h>

#include "Mesh.h"

namespace stark
{
	// Misc
	double deg2rad(const double deg);
	double rad2deg(const double rad);

	// Set operations
	template<typename T>
	std::vector<T> gather(const std::vector<T>& data, const std::vector<int>& indices);
	template<std::size_t N>
	std::vector<std::array<int, N>> apply_map(const std::vector<std::array<int, N>>& connectivity, const std::vector<int>& map);

	// Read/Write
	std::vector<Mesh<3>> load_obj(const std::string& path);
	template<std::size_t N>
	void load_vtk(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, N>>& out_conn, const std::string& path);
	template<std::size_t N>
	Mesh<N> load_vtk(const std::string& path);
	void write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets);
	void write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const bool generate_normals = false);
	void write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 2>>& edges);
	void write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 1>>& points);

	// Primitives
	Eigen::Vector3d triangle_normal(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
	double triangle_area(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
	double unsigned_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
	double signed_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);

	// Topology
	template<std::size_t N>
	void reduce_connectivity(std::vector<std::array<int, N>>& out_new_conn, std::vector<int>& out_new_to_old_map, const std::vector<std::array<int, N>>& conn, const int n_nodes);
	template<std::size_t N>
	std::tuple<std::vector<std::array<int, N>>, std::vector<int>> reduce_connectivity(const std::vector<std::array<int, N>>& conn, const int n_nodes);
	template<std::size_t N>
	void find_edges_from_simplices(std::vector<std::array<int, 2>>& out_edges, const std::vector<std::array<int, N>>& simplices, const int n_nodes);
	template<std::size_t N>
	std::vector<std::array<int, 2>> find_edges_from_simplices(const std::vector<std::array<int, N>>& simplices, const int n_nodes);
	void find_node_node_map_simplex(std::vector<std::vector<int>>& output, const int32_t* connectivity, const int32_t n_simplices, const int32_t n_nodes_per_simplex, const int32_t n_nodes);
	void find_internal_angles(std::vector<std::array<int, 4>>& internal_edges, const std::vector<std::array<int, 3>>& triangles, const int n_nodes);
	void find_perimeter_edges(std::vector<std::array<int, 2>>& out_perimeter_edges, std::vector<int>& out_edge_to_triangle_node_map, const std::vector<std::array<int, 3>>& triangles, const int n_nodes);
	std::tuple<std::vector<std::array<int, 2>>, std::vector<int>> find_perimeter_edges(const std::vector<std::array<int, 3>>& triangles, const int n_nodes);
	void find_sharp_edges(std::vector<std::array<int, 2>>& out_edges, std::vector<int>& out_old_to_new_map, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, double angle_deg_threshold);
	std::tuple<std::vector<std::array<int, 2>>, std::vector<int>> find_sharp_edges(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, double angle_deg_threshold);
	void find_surface(std::vector<std::array<int, 3>>& out_triangles, std::vector<int>& out_triangle_to_tet_node_map, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets);
	std::tuple<std::vector<std::array<int, 3>>, std::vector<int>> find_surface(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets);
	void clean_triangle_mesh(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_triangles, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double merge_by_distance = 0.0);
	std::tuple<std::vector<Eigen::Vector3d>, std::vector<std::array<int, 3>>> clean_triangle_mesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double merge_by_distance = 0.0);

	// Geometry
	double total_volume(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets);
	void compute_node_normals(std::vector<Eigen::Vector3d>& output, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles);

	// Transformations
	void move(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& translation);
	void rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis);
	void rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot);
	void scale(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& scale);
	void scale(std::vector<Eigen::Vector3d>& points, const double scale);
	void mirror(std::vector<Eigen::Vector3d>& points, const int dim, const double pivot = 0.0);
	Eigen::Vector3d rotate_deg(const Eigen::Vector3d& point, const Eigen::Matrix3d& R, const Eigen::Vector3d& pivot);




	// DEFINITIONS ==========================================================================================
	template<typename T>
	std::vector<T> gather(const std::vector<T>& data, const std::vector<int>& indices)
	{
		std::vector<T> output;
		output.reserve(indices.size());
		for (int i : indices) {
			output.push_back(data[i]);
		}
		return output;
	}
	template<std::size_t N>
	std::vector<std::array<int, N>> apply_map(const std::vector<std::array<int, N>>& connectivity, const std::vector<int>& map)
	{
		std::vector<std::array<int, N>> output(connectivity.size());
		for (size_t i = 0; i < connectivity.size(); i++) {
			for (size_t j = 0; j < N; j++) {
				output[i][j] = map[connectivity[i][j]];
			}
		}
		return output;
	}
	template<std::size_t N>
	void load_vtk(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, N>>& out_conn, const std::string& path)
	{
		vtkio::VTKFile vtk_file;
		vtk_file.read(path);
		vtk_file.get_points_to_twice_indexable(out_vertices);
		vtk_file.get_cells_to_twice_indexable(out_conn);
	}
	template<std::size_t N>
	Mesh<N> load_vtk(const std::string& path)
	{
		stark::Mesh<N> m;
		load_vtk(m.vertices, m.conn, path);
		return m;
	}
	template<std::size_t N>
	void reduce_connectivity(std::vector<std::array<int, N>>& out_new_conn, std::vector<int>& out_new_to_old_map, const std::vector<std::array<int, N>>& conn, const int n_nodes)
	{
		std::vector<int> old_to_new(n_nodes, -1);
		int new_node_count = 0;
		for (const std::array<int, N>& elem : conn) {
			std::array<int, N> new_elem;
			for (std::size_t i = 0; i < N; i++) {
				if (old_to_new[elem[i]] == -1) {
					old_to_new[elem[i]] = new_node_count;
					new_node_count++;
				}
				new_elem[i] = old_to_new[elem[i]];
			}
			out_new_conn.push_back(new_elem);
		}

		out_new_to_old_map.resize(new_node_count);
		for (int i = 0; i < n_nodes; i++) {
			if (old_to_new[i] != -1) {
				out_new_to_old_map[old_to_new[i]] = i;
			}
		}
	}
	template<std::size_t N>
	std::tuple<std::vector<std::array<int, N>>, std::vector<int>> reduce_connectivity(const std::vector<std::array<int, N>>& conn, const int n_nodes)
	{
		std::vector<std::array<int, N>> out_conn;
		std::vector<int> out_map;
		reduce_connectivity(out_conn, out_map, conn, n_nodes);
		return { out_conn, out_map };
	}
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