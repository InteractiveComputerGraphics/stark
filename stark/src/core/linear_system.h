#pragma once
#include <Eigen/Dense>

#include <symx>
#include <BlockedSparseMatrix/BlockedSparseMatrix.h>

namespace stark::core
{
	using BCRS = bsm::BlockedSparseMatrix<symx::Assembly::BLOCK_SIZE, symx::Assembly::BLOCK_SIZE, double>;
	int solve_linear_system_with_CG(Eigen::VectorXd& solution, BCRS& lhs, const Eigen::VectorXd& rhs, const int max_iterations, const double tol, const int n_threads);
	void solve_linear_system_with_directLU(Eigen::VectorXd& solution, BCRS& lhs, const Eigen::VectorXd& rhs);
}