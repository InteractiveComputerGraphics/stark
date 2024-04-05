#pragma once
#include <vector>
#include <array>

#include <Eigen/Dense>

namespace stark
{
	template<std::size_t N>
	struct Mesh
	{
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, N>> conn;
	};
}