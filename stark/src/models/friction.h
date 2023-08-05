#pragma once
#include <array>

#include <Eigen/Dense>
#include <symx>
#include <wincrypt.h>

namespace stark::models
{
	std::array<double, 3> barycentric_point_triangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
	std::array<double, 2> barycentric_point_edge(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b);
	std::array<double, 2> barycentric_edge_edge(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q);

	std::array<double, 6> point_point_projection_matrix(const Eigen::Vector3d& p, const Eigen::Vector3d& a);
	std::array<double, 6> point_edge_projection_matrix(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b);
	std::array<double, 6> triangle_projection_matrix(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
}
