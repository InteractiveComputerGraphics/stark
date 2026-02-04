#include "ElementHessians.h"
#include <cstring>
#include <algorithm>

void symx::ElementHessians::start(int n_threads)
{
	this->thread_hessians_values.resize(n_threads);
	this->thread_hessian_rows.resize(n_threads);
	this->thread_hessians.resize(n_threads);
	for (int i = 0; i < n_threads; i++) {
		this->thread_hessians_values[i].clear();
		this->thread_hessian_rows[i].clear();
		this->thread_hessians[i].clear();
	}
	this->is_projected.clear();
	this->hessian_updates.clear();
	this->n_projected_count.reset(n_threads);
}

symx::WriteHessian symx::ElementHessians::new_hessian(int thread_id, int n_blocks_per_dim)
{
	// Allocate memory for the new hessian
	int* rows = this->thread_hessian_rows[thread_id].get_cursor_with_space_to_write(n_blocks_per_dim);
	const int hess_size = BLOCK_SIZE * n_blocks_per_dim;
	double* values = this->thread_hessians_values[thread_id].get_cursor_with_space_to_write(hess_size * hess_size);
	
	// Hessian Info
	HessianView hessian;
	hessian.n_blocks_per_dim = n_blocks_per_dim;
	hessian.block_rows = rows;
	hessian.values = values;
	hessian.is_valid = true;
	this->thread_hessians[thread_id].push_back(hessian);

	// User writer
	WriteHessian write;
	write.block_rows = rows;
	write.values = values;

	return write;
}

void symx::ElementHessians::stop()
{
	concatenate_in_parallel(this->hessians, this->thread_hessians);
}

void symx::ElementHessians::project_to_PD_inplace__all(double projection_eps, bool use_mirroring)
{
	const int n_threads = static_cast<int>(this->thread_hessians.size());
	const int n_hessians = static_cast<int>(this->hessians.size());

	if (this->is_projected.size() == 0) {
		this->is_projected.resize(n_hessians, static_cast<uint8_t>(false));
	}

	#pragma omp parallel for schedule(dynamic) num_threads(n_threads)
	for (int hess_i = 0; hess_i < n_hessians; hess_i++) {
		if (this->is_projected[hess_i] == static_cast<uint8_t>(true)) {
			continue;
		}
		HessianView& hessian = this->hessians[hess_i];
		project_to_PD_inplace(hessian.values, BLOCK_SIZE * hessian.n_blocks_per_dim, projection_eps, use_mirroring);
		this->is_projected[hess_i] = static_cast<uint8_t>(true);
		this->n_projected_count.get(omp_get_thread_num())++;
	}
}

int symx::ElementHessians::project_to_PD_for_update__selectively(double projection_eps, bool use_mirroring, const std::vector<uint8_t> &active_block_dofs)
{
	return this->_project_to_PD_for_update(projection_eps, use_mirroring, &active_block_dofs);
}

int symx::ElementHessians::project_to_PD_for_update__all(double projection_eps, bool use_mirroring)
{
    return this->_project_to_PD_for_update(projection_eps, use_mirroring, nullptr);
}

int symx::ElementHessians::_project_to_PD_for_update(double projection_eps, bool use_mirroring, const std::vector<uint8_t>* active_block_dofs_ptr)
{
	// Incremental projection for PPN:
	// 1. Find unprojected elements touching active DOFs (or all if active_block_dofs_ptr == nullptr)
	// 2. Project them, storing (projected - original) as the "update"
	// 3. update_global() will add these differences to the already-assembled global Hessian

	const int n_hessians = static_cast<int>(this->hessians.size());
	const int n_threads = static_cast<int>(this->thread_hessians.size());
	this->thread_double_buffers.resize(n_threads);
	this->thread_hessians_to_project.resize(n_threads);
	this->thread_hessian_updates.resize(n_threads);
	for (int i = 0; i < n_threads; i++) {
		this->thread_hessian_updates[i].clear();
		this->thread_hessians_to_project[i].clear();
	}

	if (this->is_projected.size() == 0) {
		this->is_projected.resize(n_hessians, static_cast<uint8_t>(false));
	}

	// Handy
	const std::vector<uint8_t>& active_block_dofs = *active_block_dofs_ptr;
	int n_active_block_dofs = 0;
	if (active_block_dofs_ptr != nullptr) {
		n_active_block_dofs = static_cast<int>(active_block_dofs.size());
	}

	// Find hessians to project
	#pragma omp parallel for schedule(static) num_threads(n_threads)
	for (int hess_i = 0; hess_i < n_hessians; hess_i++) {
		if (this->is_projected[hess_i] == static_cast<uint8_t>(true)) {
			continue;
		}

		HessianView& hessian = this->hessians[hess_i];

		bool project = false;
		if (active_block_dofs_ptr == nullptr) {
			// Project all unprojected
			project = true;
		}
		else {
			// Do we have to project based on active dofs?
			for (int block_row = 0; block_row < hessian.n_blocks_per_dim; block_row++) {
				const int block_idx = hessian.block_rows[block_row];
				if (block_idx < n_active_block_dofs && active_block_dofs[block_idx]) {
					project = true;
					break;
				}
			}
		}

		if (project) {
			const int thread_id = omp_get_thread_num();
			this->thread_hessians_to_project[thread_id].push_back(hess_i);
		}
	}
	concatenate_in_parallel(this->hessians_to_project, this->thread_hessians_to_project);

	// Exit if nothing to project
	const int n_hessians_to_project = static_cast<int>(this->hessians_to_project.size());
	if (n_hessians_to_project == 0) {
		this->hessian_updates.clear();
		return 0;
	}

	// Project selected hessians to PD
	#pragma omp parallel for schedule(static) num_threads(n_threads)
	for (int i = 0; i < n_hessians_to_project; i++) {
		const int hess_i = this->hessians_to_project[i];
		HessianView& hessian = this->hessians[hess_i];
		
		// Mark as projected. It will never be visited again after this.
		this->is_projected[hess_i] = static_cast<uint8_t>(true);
		const int thread_id = omp_get_thread_num();
		this->n_projected_count.get(thread_id)++;

		// Dimensions
		const int size = BLOCK_SIZE * hessian.n_blocks_per_dim;
		const int n_entries = size * size;

		// Copy the original Hessian
		this->thread_double_buffers[thread_id].resize(n_entries);
		double* orig_hess = this->thread_double_buffers[thread_id].data();
		std::memcpy(orig_hess, hessian.values, n_entries * sizeof(double));

		// Project to PD
		const bool changed = project_to_PD_inplace(hessian.values, size, projection_eps, use_mirroring);

		// Only if it changed (had neg eigvals) we store the difference
		if (changed) {
			// Compute difference: projected - original and store in hessian.values
			// (The element is marked as projected, so it won't be used again)
			for (int j = 0; j < n_entries; j++) {
				hessian.values[j] -= orig_hess[j];
			}
			this->thread_hessian_updates[thread_id].push_back(hessian);
		}
	}
	concatenate_in_parallel(this->hessian_updates, this->thread_hessian_updates);

	return n_hessians_to_project;
}

int symx::ElementHessians::n_projected() const
{
	return this->n_projected_count.get_sum();
}

double symx::ElementHessians::projection_ratio() const
{
	const int total = static_cast<int>(hessians.size());
	return total > 0 ? static_cast<double>(n_projected()) / static_cast<double>(total) : 0.0;
}

template<typename T>
void symx::ElementHessians::concatenate_in_parallel(std::vector<T>& output, std::vector<std::vector<T>>& input)
{
	const int n_threads = static_cast<int>(input.size());
	
	// Calculate offsets (reuse member vector)
	this->concat_offsets.resize(n_threads + 1);
	this->concat_offsets[0] = 0;
	for (int i = 0; i < n_threads; i++) {
		this->concat_offsets[i + 1] = this->concat_offsets[i] + static_cast<int>(input[i].size());
	}
	
	// Resize output
	output.clear();
	output.resize(this->concat_offsets.back());
	
	// Copy in parallel
	#pragma omp parallel num_threads(n_threads)
	{
		const int thread_id = omp_get_thread_num();
		const int begin = this->concat_offsets[thread_id];
		const int n = static_cast<int>(input[thread_id].size());

		for (int i = 0; i < n; i++) {
			output[begin + i] = input[thread_id][i];
		}
	}
}

symx::ElementHessians::spBSM symx::ElementHessians::assemble_global(int n_threads, int ndofs)
{
	// Build global BCSR from element Hessians.
	this->hess->start_insertion(ndofs, ndofs);
	const int n_elements = (int)this->size();
	
	#pragma omp parallel for num_threads(n_threads) schedule(static)
	for (int element_i = 0; element_i < n_elements; element_i++) {
		HessianView loc = this->hessians[element_i];
		const int n_blocks_per_dim = loc.n_blocks_per_dim;

		const int size = n_blocks_per_dim * BLOCK_SIZE;
		for (int loc_block_row = 0; loc_block_row < n_blocks_per_dim; loc_block_row++) {
			const int glob_row_begin = loc.block_rows[loc_block_row] * BLOCK_SIZE;
			const int loc_row_begin = loc_block_row * BLOCK_SIZE;
			for (int loc_block_col = 0; loc_block_col < n_blocks_per_dim; loc_block_col++) {
				const int glob_col_begin = loc.block_rows[loc_block_col] * BLOCK_SIZE;
				const int loc_col_begin = loc_block_col * BLOCK_SIZE;

				std::array<HESS_STORAGE_FLOAT, BLOCK_SIZE*BLOCK_SIZE> data;
				for (int i = 0; i < (int)BLOCK_SIZE; i++) {
					for (int j = 0; j < (int)BLOCK_SIZE; j++) {
						data[BLOCK_SIZE * i + j] = (HESS_STORAGE_FLOAT)loc.values[(loc_row_begin + i)*size + (loc_col_begin + j)];
					}
				}
				this->hess->add_block_from_ptr<HESS_STORAGE_FLOAT, bsm::Ordering::RowMajor>(glob_row_begin, glob_col_begin, data.data());
			}
		}
	}
	this->hess->end_insertion(n_threads);

	return this->hess;
}

symx::ElementHessians::spBSM symx::ElementHessians::update_global(int n_threads)
{
	// Add projection differences to already-assembled global Hessian (for PPN incremental updates)
	const int n_elements = (int)this->hessian_updates.size();

	#pragma omp parallel for num_threads(n_threads) schedule(static)
	for (int element_i = 0; element_i < n_elements; element_i++) {
		HessianView loc = this->hessian_updates[element_i];
		const int n_blocks_per_dim = loc.n_blocks_per_dim;

		const int size = n_blocks_per_dim * BLOCK_SIZE;
		for (int loc_block_row = 0; loc_block_row < n_blocks_per_dim; loc_block_row++) {
			const int glob_row_begin = loc.block_rows[loc_block_row] * BLOCK_SIZE;
			const int loc_row_begin = loc_block_row * BLOCK_SIZE;
			for (int loc_block_col = 0; loc_block_col < n_blocks_per_dim; loc_block_col++) {
				const int glob_col_begin = loc.block_rows[loc_block_col] * BLOCK_SIZE;
				const int loc_col_begin = loc_block_col * BLOCK_SIZE;

				std::array<HESS_STORAGE_FLOAT, BLOCK_SIZE*BLOCK_SIZE> data;
				for (int i = 0; i < (int)BLOCK_SIZE; i++) {
					for (int j = 0; j < (int)BLOCK_SIZE; j++) {
						data[BLOCK_SIZE * i + j] = (HESS_STORAGE_FLOAT)loc.values[(loc_row_begin + i)*size + (loc_col_begin + j)];
					}
				}
				bool success = this->hess->update_block_from_ptr<HESS_STORAGE_FLOAT, bsm::Ordering::RowMajor>(glob_row_begin, glob_col_begin, data.data());
				if (!success) {
					std::cout << "\nError symx::ElementHessians::update_global(): Block not found in the global Hessian. This should not happen." << std::endl;
					std::cout << "Block row: " << loc.block_rows[loc_block_row] << std::endl;
					std::cout << "Block col: " << loc.block_rows[loc_block_col] << std::endl;
					exit(-1);
				}
			}
		}
	}

	return this->hess;
}
