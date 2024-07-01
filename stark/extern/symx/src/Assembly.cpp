#include "Assembly.h"

void symx::Assembly::reset(const std::vector<int>& dof_set_offsets, const int n_threads, const bool reset_hess, const bool reset_grad)
{
	this->dof_set_offsets = dof_set_offsets;
	this->ndofs = dof_set_offsets.back();
	this->n_threads = n_threads;
	if (this->n_threads == -1) {
		this->n_threads = omp_get_max_threads();
	}
	if (reset_hess) {
		this->hess.start_insertion(this->ndofs, this->ndofs);
	}
	if (reset_grad) {
		this->grad.reset(this->n_threads, this->ndofs);
	}
	this->E.reset(this->n_threads);
	this->compiled_runtime = 0.0;
}
