#pragma once
#include <vector>

#include <symx>
#include <Eigen/Dense>

namespace stark
{
	symx::Matrix triangle_jacobian(const std::vector<symx::Vector>& x);
	symx::Matrix tet_jacobian(const std::vector<symx::Vector>& x);

	std::array<symx::Scalar, 2> eigenvalues_sym_2x2(const symx::Matrix& A);
	std::array<symx::Scalar, 3> eigenvalues_sym_3x3(const symx::Matrix& A);
}