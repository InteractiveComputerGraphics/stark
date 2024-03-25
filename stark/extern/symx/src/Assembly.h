#pragma once

#include <Eigen/Dense>
#include <BlockedSparseMatrix/BlockedSparseMatrix.h>
#include <BlockedSparseMatrix/ParallelVector.h>
#include <BlockedSparseMatrix/ParallelNumber.h>

namespace symx
{
	class Assembly
	{
	public:
		constexpr static std::size_t BLOCK_SIZE = 3;

		/* Fields */
		bsm::BlockedSparseMatrix<BLOCK_SIZE, BLOCK_SIZE, double> hess;
		bsm::ParallelVector<Eigen::VectorXd> grad;
		bsm::ParallelNumber<double> E;
		int n_threads = -1;
		int ndofs = -1;
		double compiled_runtime = 0.0;
		std::vector<int> dof_set_offsets;

		/* Methods */
		void reset(const std::vector<int>& dof_set_offsets, const int n_threads = -1, const bool reset_hess = true, const bool reset_grad = true);
	};

	struct Assembled
	{
		bsm::BlockedSparseMatrix<Assembly::BLOCK_SIZE, Assembly::BLOCK_SIZE, double>* hess;
		Eigen::VectorXd* grad;
		double* E;
		double compiled_runtime = 0.0;
		Assembled(Assembly& assembly)
			: hess(&assembly.hess), grad(&assembly.grad.get_solution()), E(&assembly.E.get_solution())
		{
			this->compiled_runtime = assembly.compiled_runtime;
		}
	};
}
