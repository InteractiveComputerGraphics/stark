#pragma once
#include <vector>
#include <array>
#include <cstdint>

#include <Eigen/Dense>


namespace stark::utils
{
	struct Mesh
	{
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int32_t, 3>> triangles;
	};

	Mesh make_sphere(const double radius, const int subdivisions = 2);
	Mesh make_box(const Eigen::Vector3d& size);
	Mesh make_box(const double size);
	Mesh make_cylinder(const double radius, const double full_height, const int slices = 16, const int stacks = 1);
	Mesh make_torus(const double outer_radius, const double inner_radius, const int slices = 32, const int stacks = 8);
	Mesh make_knot(const double scale, const double inner_radius, const int slices = 32, const int stacks = 8);
}