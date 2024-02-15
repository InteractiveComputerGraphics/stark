#pragma once
#include <vector>
#include <array>
#include <cstdint>

#include <Eigen/Dense>

#include "Mesh.h"

namespace stark::utils
{
	Mesh<3> make_sphere(const double radius, const int subdivisions = 2);
	Mesh<3> make_box(const Eigen::Vector3d& size);
	Mesh<3> make_box(const double size);
	Mesh<3> make_cylinder(const double radius, const double full_height, const int slices = 16, const int stacks = 1);
	Mesh<3> make_torus(const double outer_radius, const double inner_radius, const int slices = 32, const int stacks = 8);
	Mesh<3> make_knot(const double scale, const double inner_radius, const int slices = 32, const int stacks = 8);
}