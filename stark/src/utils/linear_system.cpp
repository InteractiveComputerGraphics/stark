#include "linear_system.h"

int stark::utils::solve_linear_system_with_CG(Eigen::VectorXd& solution_with_initial_guess, BCRS& lhs, const Eigen::VectorXd& rhs, const int max_iterations, const double tol, const int n_threads)
{
	lhs.set_preconditioner(bsm::Preconditioner::BlockDiagonal);
	lhs.prepare_preconditioning(n_threads);

	const int ndofs = (int)rhs.size();
	cg::Info info = cg::solve<double>(solution_with_initial_guess.data(), rhs.data(), ndofs, tol, max_iterations,
		[&](double* b, const double* x, const int size) { lhs.spmxv_from_ptr(b, x, n_threads);  },
		[&](double* z, const double* r, const int size) { lhs.apply_preconditioning(z, r, n_threads); }
	, n_threads);
	return info.n_iterations;
}

void stark::utils::solve_linear_system_with_directLU(Eigen::VectorXd& solution, BCRS& lhs, const Eigen::VectorXd& rhs)
{
	std::vector<Eigen::Triplet<double>> triplets;
	lhs.to_triplets(triplets);

	Eigen::SparseMatrix<double> s;
	s.resize(rhs.size(), rhs.size());
	s.setFromTriplets(triplets.begin(), triplets.end());
	s.makeCompressed();

	Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> lu;
	lu.analyzePattern(s);
	lu.factorize(s);
	solution = lu.solve(rhs);
}
