#pragma once

#include <Eigen/Dense>
#include <BlockedSparseMatrix/ParallelVector.h>
#include <BlockedSparseMatrix/ParallelNumber.h>

#include "ElementHessians.h"

namespace symx
{
	// Collects thread-local contributions during element evaluation.
	// Workflow: start() -> evaluate elements -> stop() -> read results
	class Assembly
	{
	public:
		/* Fields */
		spElementHessians element_hessians = std::make_shared<ElementHessians>();
		bsm::ParallelVector<Eigen::VectorXd> grad;  // Thread-local gradient accumulator
		bsm::ParallelNumber<double> E;              // Thread-local energy accumulator
		int n_threads = -1;
		int ndofs = -1;
		std::vector<int> dof_set_offsets;           // Maps dof_set index -> global dof offset
		
		/* Methods */
		void start(const std::vector<int>& dof_set_offsets, int n_threads, bool hess, bool grad);
		void stop(bool hess, bool grad);
	};
}
