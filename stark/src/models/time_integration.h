#pragma once
#include <vector>
#include <math.h>

#include <symx>

namespace stark::models
{
	symx::Vector euler_integration(const symx::Vector& x0, const symx::Vector& v1, const symx::Scalar& dt);
	std::vector<symx::Vector> euler_integration(const std::vector<symx::Vector>& x0, const std::vector<symx::Vector>& v1, const symx::Scalar& dt);
}