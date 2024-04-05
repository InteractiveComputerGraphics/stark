#pragma once
#include <vector>
#include <array>
#include <cstdint>

#include <Eigen/Dense>

#include "Mesh.h"

namespace stark
{
	Mesh<3> make_sphere(const double radius, const int subdivisions = 2);
	Mesh<3> make_box(const Eigen::Vector3d& size);
	Mesh<3> make_box(const double size);
	Mesh<3> make_cylinder(const double radius, const double full_height, const int slices = 16, const int stacks = 1);
	Mesh<3> make_torus(const double outer_radius, const double inner_radius, const int slices = 32, const int stacks = 8);
	Mesh<3> make_knot(const double size, const double inner_radius, const int slices = 32, const int stacks = 8);

	void generate_triangle_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_connectivity, const Eigen::Vector2d& center, const Eigen::Vector2d& dimensions, const std::array<int, 2>& n_quads_per_dim, const double z = 0.0);
	Mesh<3> generate_triangle_grid(const Eigen::Vector2d& center, const Eigen::Vector2d& dimensions, const std::array<int, 2>& n_quads_per_dim, const double z = 0.0);
	void generate_tet_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 4>>& out_tets, const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const std::array<int, 3>& n_quads_per_dim);
	Mesh<4> generate_tet_grid(const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const std::array<int, 3>& n_quads_per_dim);
	void generate_segment_line(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 2>>& out_connectivity, const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments);
	Mesh<2> generate_segment_line(const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments);
}