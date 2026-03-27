#pragma once
#include <vector>
#include <cstdint>
#include <omp.h>
#include <BlockedSparseMatrix/BlockedSparseMatrix.h>
#include <BlockedSparseMatrix/ParallelNumber.h>

#include "chunked_vector.h"
#include "project_to_PD.h"


namespace symx
{
	// Write handle returned by new_hessian()
	struct WriteHessian
	{
		int* block_rows = nullptr;   // Global block indices for this element
		double* values = nullptr;    // Dense element Hessian (row-major)
	};
	
	
	// Stores element Hessians and handles projection-to-PD + global assembly.
	// 
	// Key insight for PPN: After assemble_global(), we can project more elements
	// incrementally. project_to_PD_for_update_*() computes (projected - original)
	// and stores the difference for update_global() to add to the global Hessian.
	class ElementHessians
	{
		/* Definitions */
	private:

		// Read-only view of a stored element Hessian
		struct HessianView
		{
			int n_blocks_per_dim = -1;   // e.g., 4 for Tet4 (4 nodes)
			int* block_rows = nullptr;
			double* values = nullptr;
			bool is_valid = false;
		};

	public:
		static constexpr int BLOCK_SIZE = 3;     // 3D DOFs
		static constexpr int CHUNK_SIZE = 10000; // For chunked_vector allocation
		#ifdef SYMX_HESS_STORAGE_FLOAT
			using HESS_STORAGE_FLOAT = SYMX_HESS_STORAGE_FLOAT;
		#else
			using HESS_STORAGE_FLOAT = double;    // Global Hessian storage (saves memory and bandwidth)
		#endif
		using spBSM = bsm::spBlockedSparseMatrix<BLOCK_SIZE, BLOCK_SIZE, HESS_STORAGE_FLOAT>;


		/* Fields */
		spBSM hess = std::make_shared<bsm::BlockedSparseMatrix<BLOCK_SIZE, BLOCK_SIZE, HESS_STORAGE_FLOAT>>();

		// Hessian storage (thread-local during collection, merged in stop())
		std::vector<HessianView> hessians;
		std::vector<std::vector<HessianView>> thread_hessians;
		std::vector<chunked_vector<double, CHUNK_SIZE>> thread_hessians_values;
		std::vector<chunked_vector<int, CHUNK_SIZE>> thread_hessian_rows;
		
		// Projection tracking (persists across incremental projections)
		std::vector<uint8_t> is_projected;
		bsm::ParallelNumber<int> n_projected_count;
		
		// For incremental updates: differences (projected - original) to add to global
		std::vector<std::vector<HessianView>> thread_hessian_updates;
		std::vector<HessianView> hessian_updates;
		bool update_all = false;
		
		// Workspace buffers (avoid repeated allocations)
		std::vector<std::vector<double>> thread_double_buffers;
		std::vector<std::vector<int>> thread_hessians_to_project;
		std::vector<int> hessians_to_project;
		std::vector<int> concat_offsets;

		/* Methods */
		ElementHessians() = default;
		~ElementHessians() = default;

		// Collection phase
		void start(int n_threads);
		WriteHessian new_hessian(int thread_id, int n_blocks_per_dim);
		void stop();

		// First assembly: project in-place, then assemble
		void project_to_PD_inplace__all(double projection_eps, bool use_mirroring);
		spBSM assemble_global(int n_threads, int ndofs);
		
		// Incremental projection: project, compute diff, update global
		int project_to_PD_for_update__selectively(double projection_eps, bool use_mirroring, const std::vector<uint8_t>& active_block_dofs);
		int project_to_PD_for_update__all(double projection_eps, bool use_mirroring);
		spBSM update_global(int n_threads);
		
		// Stats
		int size() const { return static_cast<int>(hessians.size()); }
		int n_projected() const;
		double projection_ratio() const;
		
	private:
		int _project_to_PD_for_update(double projection_eps, bool use_mirroring, const std::vector<uint8_t>* active_block_dofs);

		template<typename T>
		void concatenate_in_parallel(std::vector<T>& output, std::vector<std::vector<T>>& input);
	};

	using spElementHessians = std::shared_ptr<ElementHessians>;
}
