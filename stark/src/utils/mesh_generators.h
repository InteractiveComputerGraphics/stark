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

	Mesh make_sphere(const int subdivisions = 2);
}