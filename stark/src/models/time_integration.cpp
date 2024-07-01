#include "time_integration.h"

symx::Vector stark::time_integration(const symx::Vector& x0, const symx::Vector& v1, const symx::Scalar& dt)
{
	return x0 + dt * v1;
}

Eigen::Vector3d stark::time_integration(const Eigen::Vector3d& x0, const Eigen::Vector3d& v1, const double dt)
{
	return x0 + dt * v1;
}

std::vector<symx::Vector> stark::time_integration(const std::vector<symx::Vector>& x0, const std::vector<symx::Vector>& v1, const symx::Scalar& dt)
{
	std::vector<symx::Vector> x1;
	for (int i = 0; i < (int)x0.size(); i++) {
		x1.push_back(time_integration(x0[i], v1[i], dt));
	}
	return x1;
}
