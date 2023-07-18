#include "time_integration.h"

symx::Vector stark::models::euler_integration(const symx::Vector& x0, const symx::Vector& v1, const symx::Scalar& dt)
{
	return x0 + dt * v1;
}

std::vector<symx::Vector> stark::models::euler_integration(const std::vector<symx::Vector>& x0, const std::vector<symx::Vector>& v1, const symx::Scalar& dt)
{
	std::vector<symx::Vector> x1;
	for (int i = 0; i < (int)x0.size(); i++) {
		x1.push_back(euler_integration(x0[i], v1[i], dt));
	}
	return x1;
}
