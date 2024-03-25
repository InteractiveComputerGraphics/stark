#pragma once
#include <vector>
#include <math.h>

#include <Eigen/Dense>
#include <symx>

namespace stark
{
	symx::Vector time_integration(const symx::Vector& x0, const symx::Vector& v1, const symx::Scalar& dt);
	Eigen::Vector3d time_integration(const Eigen::Vector3d& x0, const Eigen::Vector3d& v1, const double dt);
	std::vector<symx::Vector> time_integration(const std::vector<symx::Vector>& x0, const std::vector<symx::Vector>& v1, const symx::Scalar& dt);
}