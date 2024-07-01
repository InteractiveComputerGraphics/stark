#pragma once
#include <array>

#include <Eigen/Dense>
#include <symx>


namespace stark
{
	std::array<double, 3> barycentric_point_triangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
	std::array<double, 2> barycentric_point_edge(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b);
	std::array<double, 2> barycentric_edge_edge(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q);

	std::array<double, 6> projection_matrix_point_point(const Eigen::Vector3d& p, const Eigen::Vector3d& a);
	std::array<double, 6> projection_matrix_point_edge(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b);
	std::array<double, 6> projection_matrix_triangle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
	std::array<double, 6> projection_matrix_edge_edge(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q);

	double edge_edge_mollifier(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q,
		const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& P, const Eigen::Vector3d& Q);
}
