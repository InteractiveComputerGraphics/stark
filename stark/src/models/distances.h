#pragma once
#include <symx>
#include <Eigen/Dense>

namespace stark
{
	double sq_distance_point_point(const Eigen::Vector3d& p, const Eigen::Vector3d& q);
	double sq_distance_point_line(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b);
	double sq_distance_point_plane(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
	double sq_distance_point_plane(const Eigen::Vector3d& a, const Eigen::Vector3d& plane_point, const Eigen::Vector3d& plane_normal);
	double sq_distance_line_line(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q);
	double distance_point_point(const Eigen::Vector3d& p, const Eigen::Vector3d& q);
	double distance_point_line(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b);
	double distance_point_plane(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
	double distance_point_plane(const Eigen::Vector3d& a, const Eigen::Vector3d& plane_point, const Eigen::Vector3d& plane_normal);
	double distance_line_line(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q);

	symx::Scalar sq_distance_point_point(const symx::Vector& p, const symx::Vector& q);
	symx::Scalar sq_distance_point_line(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b);
	symx::Scalar sq_distance_point_plane(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b, const symx::Vector& c);
	symx::Scalar sq_distance_point_plane(const symx::Vector& a, const symx::Vector& plane_point, const symx::Vector& plane_normal);
	symx::Scalar sq_distance_line_line(const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q);
	symx::Scalar distance_point_point(const symx::Vector& p, const symx::Vector& q);
	symx::Scalar distance_point_line(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b);
	symx::Scalar distance_point_plane(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b, const symx::Vector& c);
	symx::Scalar distance_point_plane(const symx::Vector& a, const symx::Vector& plane_point, const symx::Vector& plane_normal);
	symx::Scalar distance_line_line(const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q);

	double signed_distance_point_plane(const Eigen::Vector3d& a, const Eigen::Vector3d& plane_point, const Eigen::Vector3d& plane_normal);
}
