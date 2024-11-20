 #pragma once
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <mutex>
#include <memory>
#include <cassert>
#include <type_traits>
#include <omp.h>
#ifdef BSM_ENABLE_AVX
#include <immintrin.h>
#endif

#include "types.h"

namespace bsm
{
	/* ===============================  DECLARATIONS  =============================== */
	enum class Preconditioner { Diagonal, BlockDiagonal };
	enum class Ordering { RowMajor, ColMajor };
	enum class ThreadSafety { Safe, Unsafe };

	template<typename FLOAT, Ordering ORDERING, size_t BLOCK_ROWS, size_t BLOCK_COLS>
	class MatrixIndexer
	{
	private:
		const FLOAT* m_data;

	public:
		MatrixIndexer(const FLOAT* data) : m_data(data) {};
		inline const FLOAT& operator()(const int row, const int col) const {
			return at(this->m_data, row, col);
		};

		static const FLOAT& at(const FLOAT* data, const int row, const int col)
		{
			if constexpr (ORDERING == Ordering::RowMajor) {
				return data[BLOCK_COLS * row + col];
			}
			else {
				return data[BLOCK_ROWS * col + row];
			}
		}
	};

	/**
	* @brief Blocked sparse matrix.
	* 
	* @tparam BLOCK_ROWS Number of rows in each block.
	* @tparam BLOCK_COLS Number of columns in each block.
	* @tparam STORAGE_FLOAT Float type used to store the non-zeros. It can be different that the one used for actual operations. Default: double.
	*/
	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT = double>
	class BlockedSparseMatrix
	{
	public:
		/* Methods */
		BlockedSparseMatrix() = default;
		~BlockedSparseMatrix() = default;

		/**
		* @brief Initializes the data structures required to insert blocks into the BlockedSparseMatrix. It is mandatory
		* to call this function before any .add...() method. Not needed before .set...() methods. Once all the blocks
		* have been added, call .end_insertion().
		*
		* @param n_rows Number of rows of the matrix
		* @param n_cols Number of cols of the matrix
		*/
		void start_insertion(const int n_rows, const int n_cols);

		/**
		* @brief Builds the BlockedSparseMatrix from the added blocks.
		*
		* @param n_threads Number of threads to use for the matrix build. If -1, use all cores available. Default: -1.
		*/
		void end_insertion(const int n_threads = -1);

		/**
		* @brief Constructs a BlockedSparseMatrix from a sequence of Blocks.
		*
		* @param begin Begin Block iterator
		* @param end End Block iterator
		* @param n_rows Number of rows of the matrix
		* @param n_cols Number of cols of the matrix
		* @param n_threads Number of threads to use for the matrix build. If -1, use all cores available. Default: -1.
		*/
		void set_from_blocks(const Block<STORAGE_FLOAT, BLOCK_ROWS, BLOCK_COLS>* begin, const Block<STORAGE_FLOAT, BLOCK_ROWS, BLOCK_COLS>* end, const int n_rows, const int n_cols, const int n_threads = -1);
		
		/**
		* @brief Constructs a BlockedSparseMatrix from a sequence of triplets. These triplets can be `bsm::Triplet`
		* but it also works with `Eigen::Triplet` since they share the required interface.
		* 
		* @details The procedure is parallelized and it can take advantage of existing memory allocations.
		* Although the optimal matrix building procedure is to use the `.add_...()` methods, `.set_from_triplets()`
		* exists to provide the user with a drop-in replacement for Eigen.
		* 
		* @tparam TripletIterator Iterator or pointer to bsm::Triplet or Eigen::Triplet
		* 
		* @param begin Begin triplet iterator
		* @param end End triplet iterator
		* @param n_rows Number of rows of the matrix
		* @param n_cols Number of cols of the matrix
		* @param n_threads Number of threads to use for the matrix build. If -1, use all cores available. Default: -1.
		*/
		template<typename TripletIterator>
		void set_from_triplets(const TripletIterator begin, const TripletIterator end, const int n_rows, const int n_cols, const int n_threads = -1);
		
		/**
		* @brief Constructs a BlockedSparseMatrix from an Eigen::SparseMatrix
		*
		* @details The procedure is parallelized and it can take advantage of existing memory allocations.
		* Although the optimal matrix building procedure is to use the `.add_...()` methods, `.set_from_eigen_sparse()`
		* exists to provide the user with a drop-in replacement for Eigen.
		*
		* @tparam EigenSparseMatrix Eigen::SparseMatrix. It can be double or float and both Eigen::RowMajor and Eigen::ColMajor.
		*
		* @param n_threads Number of threads to use for the matrix build. If -1, use all cores available. Default: -1.
		*/
		template<typename EigenSparseMatrix>
		void set_from_eigen_sparse(EigenSparseMatrix& m, const int n_threads = -1);

		/**
		 * @brief (Thread safe) Adds a matrix of size BLOCK_ROWS*BLOCK_COLS to the BlockedSparseMatrix.
		 * 
		 * @details Use this method to add non-zero blocks to the BlockedSparseMatrix. This method will
		 * figure out what to do whether the block is already allocated in the existing Blocked CRS data structure
		 * or whether it has to be stored aside to be inserted upon calling `.end_insertion()`.
		 * You can use any data structure for the matrix m as long as it implements a "(row, column)" access method.
		 * By default, block additions are parallel blocked to avoid data races with other threads, however
		 * if you know data races are not possible (eg each thread inserts to a disjoint set of rows) blocking
		 * can be disabled with the second template argument which should increase performance substantially.
		 *
		 * @tparam MATRIX_PARENTHESIS_INDEXABLE Type that uses parenthesis to index a matrix. ie matrix(i, j).
		 * @tparam THREAD_SAFETY Whether to use mutexes to block write access to other threads. Default ThreadSafety::Safe.
		 * 
		 * @param base_row row of the element m(0, 0)
		 * @param base_col col of the element m(0, 0)
		 * @param m	Matrix to insert
		 */
		template<typename MATRIX_PARENTHESIS_INDEXABLE, ThreadSafety THREAD_SAFETY = ThreadSafety::Safe>
		void add_block(const int base_row, const int base_col, const MATRIX_PARENTHESIS_INDEXABLE& m);

		/**
		 * @brief (Thread safe) Adds a matrix of size BLOCK_ROWS*BLOCK_COLS to the BlockedSparseMatrix.
		 *
		 * @details (... see `.add_block()` documentation)
		 * 
		 * @warning It is assumed that BLOCK_ROWS*BLOCK_COLS numbers are available from the pointer to the
		 * first number of the matrix.
		 *
		 * @tparam FLOAT Float type of the numbers to insert.
		 * @tparam ORDERING Ordering (RowMajor or ColMajor) of the matrix according to the layout of its numbers.
		 * @tparam THREAD_SAFETY Whether to use mutexes to block write access to other threads. Default ThreadSafety::Safe.
		 *
		 * @param base_row row of the element m(0, 0)
		 * @param base_col col of the element m(0, 0)
		 * @param m	Matrix to insert stored flat in memory with single index access.
		 */
		template<typename FLOAT, Ordering ORDERING, ThreadSafety THREAD_SAFETY = ThreadSafety::Safe>
		void add_block_from_ptr(const int base_row, const int base_col, const FLOAT* m);

		/**
		* @brief Sparse Matrix - Dense Vector product.
		*
		* @tparam ResizeableVector 1D indexable contiguous-in-memory data structure that implements .resize(), eg. std::vector<FLOAT> or Eigen::VectorXd.
		*
		* @param solution Output vector. Can be uninitialized.
		* @param v Vector to multiply the sparse matrix with.
		* @param n_threads Number of threads to use for the matrix build. If -1, use all cores available. Default: -1.
		*/
		template<typename ResizeableVector>
		void spmxv(ResizeableVector& solution, const ResizeableVector& v, const int n_threads = -1);

		/**
		* @brief Sparse Matrix - Dense Vector product.
		*
		* @param solution Pointer to the first element of the output vector. The whole vector must be already allocated.
		* @param v Pointer to the first element of the vector to multiply the sparse matrix with.
		* @param n_threads Number of threads to use for the matrix build. If -1, use all cores available. Default: -1.
		*/
		void spmxv_from_ptr(double* solution, const double* v, const int n_threads = -1);

		/**
		* @brief Sets the preconditioned for the Conjugate Gradient solve.
		*
		* @param Preconditioner Preconditioner to use. By default, BlockDiagonal is used.
		*/
		void set_preconditioner(const Preconditioner preconditioner);

		/**
		* @brief Precalculates the selected preconditioner for the current matrix.
		*
		* @param n_threads Number of threads to use for the preconditioning preparation. If -1, use all cores available. Default: -1.
		*/
		void prepare_preconditioning(const int n_threads = -1);

		/**
		* @brief Applies the selected preconditioner to a given vector.
		*
		* @details Can only be used after calling prepare_preconditioning. Similar to spmxv method.
		* 
		* @tparam ResizeableVector 1D indexable contiguous-in-memory data structure that implements .resize(), eg. std::vector<FLOAT> or Eigen::VectorXd.
		*
		* @param solution Output vector. Can be uninitialized.
		* @param v Vector to multiply the preconditioned matrix with.
		* @param n_threads Number of threads to use for the multiplication. If -1, use all cores available. Default: -1.
		*/
		template<typename OPERATION_FLOAT>
		void apply_preconditioning(OPERATION_FLOAT* solution, const OPERATION_FLOAT* v, const int n_threads = -1);

		/**
		 * @brief 
		 * @tparam STORAGE_FLOAT 
		*/
		template<typename Triplet>
		void to_triplets(std::vector<Triplet>& triplets);

		void clear();

	private:
		/* Methods */
		// Insertion
		template<ThreadSafety THREAD_SAFETY, typename MATRIX_PARENTHESIS_INDEXABLE>
		void _insert_new_block(const int base_row, const int base_col, const MATRIX_PARENTHESIS_INDEXABLE& m);
		template<ThreadSafety THREAD_SAFETY, typename MATRIX_PARENTHESIS_INDEXABLE>
		void _sum_to_existing_block(const int base_row, const int block_n, const MATRIX_PARENTHESIS_INDEXABLE& m);
		template<typename MATRIX_PARENTHESIS_INDEXABLE>
		void _sum_block(STORAGE_FLOAT* dest, const MATRIX_PARENTHESIS_INDEXABLE& m);

		// Arithmetic
		template<typename OPERATION_FLOAT>
		void _spmxv(OPERATION_FLOAT* solution, const OPERATION_FLOAT* v, const int n_threads = -1);

		/* Definitions */
		static_assert(std::is_same_v<STORAGE_FLOAT, double> || std::is_same_v<STORAGE_FLOAT, float>);

		using ColBlock = ColBlockTemplated<STORAGE_FLOAT, BLOCK_ROWS, BLOCK_COLS>;

		struct CRS
		{
			std::vector<int> rows; // Offsets to the beginning of the block rows
			std::vector<int> cols; // Columns of the first element of each block
			bsm::avx::avector<STORAGE_FLOAT, 64> vals;  // bsm::avx::avector is a std::vector that is aligned to memory registers
		};

		/* Fields */
		// Matrix
		int n_rows = -1;
		int n_cols = -1;
		int n_block_rows = -1;
		int n_block_cols = -1;

		// CRS
		/*
			Note: We keed two matrices in memory because in case of new entries,
			we allocate the new matrix and then copy the entries that fitted in the
			previous one into the new one.
		*/
		std::vector<char> curr_crs_vals_is_nnz;
		std::unique_ptr<CRS> crs_current;
		std::unique_ptr<CRS> crs_next;

		//// Non existing entries
		std::vector<std::vector<ColBlock>> dynamic_row_buckets;

		// Preconditioning
		Preconditioner preconditioner = Preconditioner::BlockDiagonal;
		bsm::avx::avector<STORAGE_FLOAT, 32> diag_inv;

		// Logic
		std::vector<std::mutex> row_bucket_mutexes;
		std::vector<std::mutex> row_crs_mutexes;
		int n_rows_current_crs = -1;
	};




	/* ===============================  DEFINITIONS  =============================== */
	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::clear()
	{
		this->n_rows = -1;
		this->n_cols = -1;
		this->n_block_rows = -1;
		this->n_block_cols = -1;

		this->curr_crs_vals_is_nnz.clear();
		this->crs_current->rows.clear();
		this->crs_current->cols.clear();
		this->crs_current->vals.clear();
		this->crs_next->rows.clear();
		this->crs_next->cols.clear();
		this->crs_next->vals.clear();

		this->dynamic_row_buckets.clear();
		this->diag_inv.clear();

		this->row_bucket_mutexes.clear();
		this->row_crs_mutexes.clear();
		this->n_rows_current_crs = -1;
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::start_insertion(const int n_rows, const int n_cols)
	{
		// Dimensions
		this->n_rows = n_rows;
		this->n_cols = n_cols;
		this->n_block_rows = (n_rows % BLOCK_ROWS == 0) ? n_rows / BLOCK_ROWS : n_rows / BLOCK_ROWS + 1;
		this->n_block_cols = (n_cols % BLOCK_COLS == 0) ? n_cols / BLOCK_COLS : n_cols / BLOCK_COLS + 1;;

		// First insertion
		if (this->crs_current == nullptr) {
			this->crs_current = std::make_unique<CRS>();
			this->crs_next = std::make_unique<CRS>();
		}
		this->n_rows_current_crs = std::max(0, (int)this->crs_current->rows.size() - 1) * BLOCK_ROWS;

		// Allocations
		//// Existing CRS matrix (current)
		std::fill(this->crs_current->vals.begin(), this->crs_current->vals.end(), static_cast<STORAGE_FLOAT>(0.0));
		this->curr_crs_vals_is_nnz.resize(this->crs_current->cols.size());
		std::fill(this->curr_crs_vals_is_nnz.begin(), this->curr_crs_vals_is_nnz.end(), static_cast<char>(0));
		if (this->row_crs_mutexes.size() != this->n_rows_current_crs / BLOCK_ROWS) {
			this->row_crs_mutexes = std::vector<std::mutex>(this->n_rows_current_crs);
		}

		//// New bucket entries
		this->dynamic_row_buckets.resize(this->n_block_rows);
		if (this->row_bucket_mutexes.size() != this->n_block_rows) {
			this->row_bucket_mutexes = std::vector<std::mutex>(n_block_rows);
		}
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::end_insertion(const int n_threads)
	{
		constexpr int BLOCK_SIZE = BLOCK_ROWS * BLOCK_COLS;
		const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() : n_threads;

		/*
			We handle 3 cases:
				- Matrix size and sparsity pattern didn't change.
				- There is no old matrix. (Will happen in the first usage)
				- General case -> Merge old sparsity pattern (minus new zeros) with the new blocks in the buckets.
		*/

		// [Case 1] Matrix size and sparsity pattern didn't change.
		const int n_current_block_rows = std::max(0, (int)this->crs_current->rows.size() - 1);
		if (n_current_block_rows == this->n_block_rows) {

			// Check if there are new entries
			bool are_new_entries = false;
			for (int i = 0; i < this->n_block_rows; i++) {
				if (this->dynamic_row_buckets[i].size() > 0) {
					are_new_entries = true;
					break;
				}
			}

			if (!are_new_entries) {
				bool is_any_old_entry_now_zero = false;
				for (int i = 0; i < (int)this->curr_crs_vals_is_nnz.size(); i++) {
					if (this->curr_crs_vals_is_nnz[i] == static_cast<char>(0)) {
						is_any_old_entry_now_zero = true;
						break;
					}
				}

				if (!is_any_old_entry_now_zero) {
					return;  // Same matrix sparsity pattern. `this->crs_current` is already the updated matrix
				}
			}
		}

		// Count nnz blocks per row (common processing for cases 2 and 3)
		this->crs_next->rows.resize(this->n_block_rows + 1);
		#pragma omp parallel for num_threads(n_threads_) schedule(static)
		for (int i = 0; i < this->n_block_rows; i++) {
			this->crs_next->rows[i + 1] = (int)this->dynamic_row_buckets[i].size();
		}

		// [Case 2] There is no old matrix. Will happen in the first usage.
		if (this->crs_current->rows.size() == 0) {

			// Prefix sum
			this->crs_next->rows[0] = 0;
			for (int i = 0; i < this->n_block_rows; i++) {
				this->crs_next->rows[i + 1] = this->crs_next->rows[i] + this->crs_next->rows[i + 1];
			}

			// Allocate
			const int n_blocks = this->crs_next->rows.back();
			constexpr int BLOCK_SIZE = BLOCK_ROWS * BLOCK_COLS;
			this->crs_next->cols.resize(n_blocks);
#ifdef BSM_ENABLE_AVX
			this->crs_next->vals.reserve(BLOCK_SIZE * n_blocks + sizeof(__m256) / sizeof(STORAGE_FLOAT));  // Reserve an extra __m256 space for padding
#else
			this->crs_next->vals.reserve(BLOCK_SIZE * n_blocks + 1);
#endif
			this->crs_next->vals.resize(BLOCK_SIZE * n_blocks);

			// Copy data in buckets into the CRS
			#pragma omp parallel for num_threads(n_threads_) schedule(static)
			for (int block_row_i = 0; block_row_i < this->n_block_rows; block_row_i++) {
				std::vector<ColBlock>& bucket = this->dynamic_row_buckets[block_row_i];
				const int begin = this->crs_next->rows[block_row_i];

				for (int j = 0; j < bucket.size(); j++) {
					this->crs_next->cols[begin + j] = bucket[j].base_col;
					memcpy(this->crs_next->vals.data() + BLOCK_SIZE * (begin + j), bucket[j].vals.data(), sizeof(STORAGE_FLOAT) * BLOCK_SIZE);
				}
			}

			// Clear Buckets
			#pragma omp parallel for num_threads(n_threads_) schedule(static)
			for (int i = 0; i < this->n_block_rows; i++) {
				this->dynamic_row_buckets[i].clear();
			}

			// Swap pointers
			this->crs_current.swap(this->crs_next);
			return; // Exit of case 2
		}

		// [Case 3] General merge
		// Find new total sparsity pattern
		//// Add current nnz blocks per row (`dynamic_row_buckets` already counted)
		const int min_common_n_block_rows = std::min(this->n_block_rows, n_current_block_rows);
		#pragma omp parallel for num_threads(n_threads_) schedule(static)
		for (int i = 0; i < min_common_n_block_rows; i++) {
			int existing_count = 0;
			for (int j = this->crs_current->rows[i]; j < this->crs_current->rows[i + 1]; j++) {
				existing_count += (int)this->curr_crs_vals_is_nnz[j];
			}
			this->crs_next->rows[i + 1] += existing_count;
		}

		//// Prefix sum
		this->crs_next->rows[0] = 0;
		for (int i = 0; i < this->n_block_rows; i++) {
			this->crs_next->rows[i + 1] = this->crs_next->rows[i] + this->crs_next->rows[i + 1];
		}

		// Copy data from the current CRS and the new buckets to the next CRS
		/*
			- Both are sorted
			- The same non zero block cannot be in both existing and new
			- Current and next matrices can have different n_rows 
		*/
		//// Allocate new matrix
		const int n_blocks = this->crs_next->rows.back();
		this->crs_next->cols.resize(n_blocks);
#ifdef BSM_ENABLE_AVX
		this->crs_next->vals.reserve(BLOCK_SIZE * n_blocks + sizeof(__m256)/sizeof(STORAGE_FLOAT));  // Reserve an extra __m256 space for padding
#else
		this->crs_next->vals.reserve(BLOCK_SIZE * n_blocks + 1);
#endif
		this->crs_next->vals.resize(BLOCK_SIZE*n_blocks);

		//// Loop through the rows of the new CRS and copy the entries from the old CRS and the new row_buckets, in order
		#pragma omp parallel for num_threads(n_threads_) schedule(static)
		for (int block_row_i = 0; block_row_i < this->n_block_rows; block_row_i++) {
			std::vector<ColBlock>& bucket = this->dynamic_row_buckets[block_row_i];

			// Current CRS
			//// Find the block range we have in the current CRS row
			int current_cursor = 0;
			int current_end_cursor = 0;
			if (block_row_i < n_current_block_rows) { // The row might not exist
				current_cursor = this->crs_current->rows[block_row_i];
				current_end_cursor = this->crs_current->rows[block_row_i + 1];
			}

			//// Set a cursor for the current CRS in the first non-zero block
			while ((current_cursor < current_end_cursor) && (this->curr_crs_vals_is_nnz[current_cursor] == static_cast<char>(0))) { current_cursor++; }
			
			// New CRS buckets
			//// Find how many blocks we have in the row bucket
			const int bucket_end_cursor = (int)bucket.size();

			//// Ser a cursor for the new buckets (we don't have zero blocks there)
			int new_cursor = 0;

			// Loop over the the new CRS blocks to be filled picking the smallest block from current CRS and the bucket
			for (int offset = this->crs_next->rows[block_row_i]; offset < this->crs_next->rows[block_row_i + 1]; offset++) {
				
				// [Early exit] No more new blocks
				if (new_cursor == bucket_end_cursor) {
					for (int block_i = offset; block_i < this->crs_next->rows[block_row_i + 1]; block_i++) {
						// Set column
						this->crs_next->cols[block_i] = this->crs_current->cols[current_cursor];

						// Copy block data
						memcpy(this->crs_next->vals.data() + BLOCK_SIZE * block_i, this->crs_current->vals.data() + BLOCK_SIZE * current_cursor, sizeof(STORAGE_FLOAT) * BLOCK_SIZE);
						
						// Find next non-zero block
						current_cursor++;
						while ((current_cursor < current_end_cursor) && (this->curr_crs_vals_is_nnz[current_cursor] == static_cast<char>(0))) { current_cursor++; }
					}
					break;
				}

				// [Early exit] No more current blocks
				else if (current_cursor == current_end_cursor) {
					for(int block_i = offset; block_i < this->crs_next->rows[block_row_i + 1]; block_i++) {
						// Set column
						this->crs_next->cols[block_i] = bucket[new_cursor].base_col;

						// Copy block data
						memcpy(this->crs_next->vals.data() + BLOCK_SIZE * block_i, bucket[new_cursor].vals.data(), sizeof(STORAGE_FLOAT) * BLOCK_SIZE);

						// Next block
						new_cursor++;
					}
					break;
				}

				// General case: Remaining blocks both in current and new
				else {
					// The block in the current CRS under the cursor has a smaller column index
					if (this->crs_current->cols[current_cursor] < bucket[new_cursor].base_col) {
						this->crs_next->cols[offset] = this->crs_current->cols[current_cursor];
						memcpy(this->crs_next->vals.data() + BLOCK_SIZE * offset, this->crs_current->vals.data() + BLOCK_SIZE * current_cursor, sizeof(STORAGE_FLOAT) * BLOCK_SIZE);
						current_cursor++;
						while ((current_cursor < current_end_cursor) && (this->curr_crs_vals_is_nnz[current_cursor] == static_cast<char>(0))) { current_cursor++; }
					}

					// The block in bucket under the cursor has a smaller column index
					else {
						this->crs_next->cols[offset] = bucket[new_cursor].base_col;
						memcpy(this->crs_next->vals.data() + BLOCK_SIZE * offset, bucket[new_cursor].vals.data(), sizeof(STORAGE_FLOAT) * BLOCK_SIZE);
						new_cursor++;
					}
				}
			}
		}

		// Clear Buckets
		#pragma omp parallel for num_threads(n_threads_) schedule(static)
		for (int i = 0; i < this->n_block_rows; i++) {
			this->dynamic_row_buckets[i].clear();
		}

		// Swap pointers
		this->crs_current.swap(this->crs_next);

		return; // Exit of case 3
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<typename TripletIterator>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::set_from_triplets(const TripletIterator begin, const TripletIterator end, const int n_rows, const int n_cols, const int n_threads)
	{
		/*
		* Overview:
		*	- Each thread has a local buffer of blocks capped to 10.
		*	- Each thread loops over a section of the triplets.
		*	- As long as the next triplet is in one of the blocks in the buffer, it is added there.
		*	- The blocks in the buffer are added and the buffers cleared once it reaches 10 blocks or
		*		the next triplet to be included has a new block_row and block_col.
		* 
		* This strategy needs very little extra memory and works great in the case that the triplets in the
		* list are added as thery are typically computed in FEM codes: a NxM blocked submatrix is converted into
		* triplets lexicoghraphically, therefore, triplets will follow this order.
		*/

		this->start_insertion(n_rows, n_rows);
		constexpr int BLOCK_SIZE = BLOCK_ROWS * BLOCK_COLS;
		const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() / 2 : n_threads;
		const int n_triplets = (int)std::distance(begin, end);
		const TripletIterator triplets = begin;
		
		#pragma omp parallel num_threads(n_threads_)
		{
			using Block_ = Block<STORAGE_FLOAT, BLOCK_ROWS, BLOCK_COLS>;
			std::vector<Block_> buffer_blocks;
			buffer_blocks.reserve(10);

			const int thread_id = omp_get_thread_num();
			const int chunk_size = n_triplets / n_threads_;
			const int thread_begin_triplet = chunk_size * thread_id;
			const int thread_end_triplet = (thread_id == n_threads_ - 1) ? n_triplets : chunk_size * (thread_id + 1);

			Block_ dummy_block;
			dummy_block.base_row = -1;
			dummy_block.base_col = -1;
			Block_* curr_block = &dummy_block;  // Pointer to the current block

			for (int triplet_i = thread_begin_triplet; triplet_i < thread_end_triplet; triplet_i++) {
				const auto& triplet = triplets[triplet_i];
				const int block_row = triplet.row() / BLOCK_ROWS;
				const int block_col = triplet.col() / BLOCK_COLS;
				const int base_row = block_row * BLOCK_ROWS;
				const int base_col = block_col * BLOCK_COLS;
				const int loc_row = triplet.row() % BLOCK_ROWS;
				const int loc_col = triplet.col() % BLOCK_COLS;
				const bool different_block_row = base_row != curr_block->base_row;
				const bool different_block_col = base_col != curr_block->base_col;

				// Triplet does not belong to the curr_block: Find or create the next one
				if (different_block_row || different_block_col) {

					// Triplet is in a new block_row and block_col wrt the buffer blocks: flush the buffer
					if (buffer_blocks.size() == 10 || different_block_row && different_block_col) {
						for (const Block_& block : buffer_blocks) {
							this->add_block_from_ptr<STORAGE_FLOAT, Ordering::ColMajor>(block.base_row, block.base_col, block.vals.data());
						}
						buffer_blocks.clear();
					}

					// Find the relevant block in the buffer
					curr_block = &dummy_block;
					for (Block_& block : buffer_blocks) {
						if (base_col == block.base_col && base_row == block.base_row) {
							curr_block = &block;
							break;
						}
					}

					// Relevant block not in the buffer: Create a new block and add it to the buffer
					if (curr_block == &dummy_block) {
						Block_ block;
						block.base_col = base_col;
						block.base_row = base_row;
						std::fill(block.vals.begin(), block.vals.end(), static_cast<STORAGE_FLOAT>(0.0));
						buffer_blocks.push_back(block);
						curr_block = &buffer_blocks.back();
					}
				}
				
				// Add the triplet contribution to the current block
				curr_block->vals[loc_col * BLOCK_ROWS + loc_row] += static_cast<STORAGE_FLOAT>(triplet.value());
			}

			for (const Block_& block : buffer_blocks) {
				this->add_block_from_ptr<STORAGE_FLOAT, Ordering::ColMajor>(block.base_row, block.base_col, block.vals.data());
			}
		}
		this->end_insertion(n_threads);
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<typename EigenSparseMatrix>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::set_from_eigen_sparse(EigenSparseMatrix& m, const int n_threads)
	{
		/*
		* Note: The explanation and variables names follow the RowMajor storage convention.
		* If the Eigen sparse matrix is stored in ColMajor layout, the block_row and block_col 
		* indices are swapped at the end.
		* 
		* Overview: 
		*	- In parallel, we loop over the rows in groups of BLOCK_ROWS
		*	- We find which is the first non zero block corresponding to all the relevant rows
		*	- We collect the non zeros corresponding to the said block from all the relevant rows and add them into a block buffer
		*	- We add the block into the BlockedSparseMatrix
		* 
		* Note: It is assumed that Eigen keeps the inner indices sorted (From the Eigen docs: "Currently the elements 
		*		of a given inner vector are guaranteed to be always sorted by increasing inner indices.")
		*/
		
		const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() / 2 : n_threads;
		m.makeCompressed();
		const int* outer = m.outerIndexPtr();
		const int* inner = m.innerIndexPtr();
		const auto* vals = m.valuePtr();

		this->start_insertion((int)m.rows(), (int)m.cols());
		#pragma omp parallel for schedule(static) num_threads(n_threads_)
		for (int block_row = 0; block_row < this->n_block_rows; block_row++) {
			const int base_row = BLOCK_ROWS * block_row;

			// Find begin and end of relevant rows
			std::array<int, BLOCK_ROWS> inner_cursors;
			std::array<int, BLOCK_ROWS> inner_ends;
			for (int i = 0; i < BLOCK_ROWS; i++) {
				if (BLOCK_ROWS * block_row + i < this->n_rows) {
					inner_cursors[i] = outer[BLOCK_ROWS * block_row + i];
					inner_ends[i] = outer[BLOCK_ROWS * block_row + i + 1];
				} else{
					// This will work with the rest of the logic if nrows is not divisible by BLOCK_ROWS
					inner_cursors[i] = 0;
					inner_ends[i] = 0;
				}
			}

			// Find all blocks
			bool finished = false;
			while (!finished) {
				// Set block to zeros
				std::array<STORAGE_FLOAT, BLOCK_ROWS * BLOCK_COLS> block;
				std::fill(block.begin(), block.end(), static_cast<STORAGE_FLOAT>(0.0));
					
				// Find next block
				int begin_col = std::numeric_limits<int>::max();
				for (int i = 0; i < BLOCK_ROWS; i++) {
					if (inner_cursors[i] != inner_ends[i]) {
						begin_col = std::min(inner[inner_cursors[i]], begin_col);
					}
				}
				const int block_col = begin_col / BLOCK_COLS;
				begin_col = block_col * BLOCK_COLS;
				const int end_col = begin_col + BLOCK_COLS;

				// Add row non zeros to the block
				for (int row = 0; row < BLOCK_ROWS; row++) {
					while ((inner_cursors[row] != inner_ends[row]) && (inner[inner_cursors[row]] < end_col)) {
						const int col = inner[inner_cursors[row]] - begin_col;
						block[col * BLOCK_ROWS + row] += static_cast<STORAGE_FLOAT>(vals[inner_cursors[row]]);
						inner_cursors[row]++;
					}
				}

				// Add the block
				if (m.IsRowMajor) {
					this->add_block_from_ptr<STORAGE_FLOAT, Ordering::ColMajor, ThreadSafety::Unsafe>(base_row, begin_col, block.data());
				}
				else {
					this->add_block_from_ptr<STORAGE_FLOAT, Ordering::RowMajor, ThreadSafety::Unsafe>(begin_col, base_row, block.data());
				}

				// Check exit condition for the block_row (all rows finished)
				finished = true;
				for (int i = 0; i < BLOCK_ROWS; i++) {
					if (inner_cursors[i] != inner_ends[i]) {
						finished = false;
						break;
					}
				}
			}
		}

		this->end_insertion(n_threads_);
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<typename MATRIX_PARENTHESIS_INDEXABLE, ThreadSafety THREAD_SAFETY>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::add_block(const int base_row, const int base_col, const MATRIX_PARENTHESIS_INDEXABLE& m)
	{
		assert(base_row < this->n_rows);
		assert(base_col < this->n_cols);
		const int block_row = base_row / BLOCK_ROWS;

		// Beyond existing rows: non-existing entry (also triggered when first build)
		if (base_row >= this->n_rows_current_crs) {
			this->_insert_new_block<THREAD_SAFETY>(base_row, base_col, m);
		}

		// Might be already allocated. Have to search for it.
		else {
			// binary search the block row
			const std::vector<int>::iterator begin = this->crs_current->cols.begin() + this->crs_current->rows[block_row];
			const std::vector<int>::iterator end = this->crs_current->cols.begin() + this->crs_current->rows[block_row + 1];
			const std::vector<int>::iterator pos = std::lower_bound(begin, end, base_col);

			bool found = true;
			if (pos == end || (*pos) != base_col) {
				found = false;
			}

			// Insert
			if (!found) {
				this->_insert_new_block<THREAD_SAFETY>(base_row, base_col, m);
			}
			else {
				const int block_n = (int)std::distance(this->crs_current->cols.begin(), pos);
				this->_sum_to_existing_block<THREAD_SAFETY>(base_row, block_n, m);
			}
		}
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<typename FLOAT, Ordering ORDERING, ThreadSafety THREAD_SAFETY>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::add_block_from_ptr(const int base_row, const int base_col, const FLOAT* m)
	{
		this->add_block<MatrixIndexer<FLOAT, ORDERING, BLOCK_ROWS, BLOCK_COLS>, THREAD_SAFETY>(base_row, base_col, MatrixIndexer<FLOAT, ORDERING, BLOCK_ROWS, BLOCK_COLS>(m));
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<ThreadSafety THREAD_SAFETY, typename MATRIX_PARENTHESIS_INDEXABLE>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::_insert_new_block(const int base_row, const int base_col, const MATRIX_PARENTHESIS_INDEXABLE& m)
	{
		const int block_row = base_row / BLOCK_ROWS;
		if constexpr (THREAD_SAFETY == ThreadSafety::Safe) { this->row_bucket_mutexes[block_row].lock(); }
		std::vector<ColBlock>& bucket = this->dynamic_row_buckets[block_row];
		auto it = std::lower_bound(bucket.begin(), bucket.end(), base_col, [](const ColBlock& block, const int value) { return block.base_col < value; });
		const bool found = (it != bucket.end()) && (it->base_col == base_col);

		if (found) {
			this->_sum_block(it->vals.data(), m);
		}
		else {
			ColBlock new_block(base_col, m);
			bucket.insert(it, new_block);
		}
		if constexpr (THREAD_SAFETY == ThreadSafety::Safe) { this->row_bucket_mutexes[block_row].unlock(); }
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<ThreadSafety THREAD_SAFETY, typename MATRIX_PARENTHESIS_INDEXABLE>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::_sum_to_existing_block(const int base_row, const int block_n, const MATRIX_PARENTHESIS_INDEXABLE& m)
	{
		const int block_row = base_row / BLOCK_ROWS;
		STORAGE_FLOAT* vals = this->crs_current->vals.data() + BLOCK_ROWS * BLOCK_COLS * block_n;

		if constexpr (THREAD_SAFETY == ThreadSafety::Safe) { this->row_crs_mutexes[block_row].lock(); }
		this->curr_crs_vals_is_nnz[block_n] = static_cast<char>(1);
		this->_sum_block(vals, m);
		if constexpr (THREAD_SAFETY == ThreadSafety::Safe) { this->row_crs_mutexes[block_row].unlock(); }
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<typename MATRIX_PARENTHESIS_INDEXABLE>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::_sum_block(STORAGE_FLOAT* dest, const MATRIX_PARENTHESIS_INDEXABLE& m)
	{
		// Common cases specialization
		if constexpr (BLOCK_ROWS == 2 && BLOCK_COLS == 2) {
			dest[0] += static_cast<STORAGE_FLOAT>(m(0, 0));
			dest[1] += static_cast<STORAGE_FLOAT>(m(1, 0));

			dest[2] += static_cast<STORAGE_FLOAT>(m(0, 1));
			dest[3] += static_cast<STORAGE_FLOAT>(m(1, 1));
		}
		else if constexpr (BLOCK_ROWS == 3 && BLOCK_COLS == 3) {
			dest[0] += static_cast<STORAGE_FLOAT>(m(0, 0));
			dest[1] += static_cast<STORAGE_FLOAT>(m(1, 0));
			dest[2] += static_cast<STORAGE_FLOAT>(m(2, 0));

			dest[3] += static_cast<STORAGE_FLOAT>(m(0, 1));
			dest[4] += static_cast<STORAGE_FLOAT>(m(1, 1));
			dest[5] += static_cast<STORAGE_FLOAT>(m(2, 1));

			dest[6] += static_cast<STORAGE_FLOAT>(m(0, 2));
			dest[7] += static_cast<STORAGE_FLOAT>(m(1, 2));
			dest[8] += static_cast<STORAGE_FLOAT>(m(2, 2));
		}
		else if constexpr (BLOCK_ROWS == 4 && BLOCK_COLS == 4) {
			dest[0] += static_cast<STORAGE_FLOAT>(m(0, 0));
			dest[1] += static_cast<STORAGE_FLOAT>(m(1, 0));
			dest[2] += static_cast<STORAGE_FLOAT>(m(2, 0));
			dest[3] += static_cast<STORAGE_FLOAT>(m(3, 0));

			dest[4] += static_cast<STORAGE_FLOAT>(m(0, 1));
			dest[5] += static_cast<STORAGE_FLOAT>(m(1, 1));
			dest[6] += static_cast<STORAGE_FLOAT>(m(2, 1));
			dest[7] += static_cast<STORAGE_FLOAT>(m(3, 1));

			dest[8]  += static_cast<STORAGE_FLOAT>(m(0, 2));
			dest[9]  += static_cast<STORAGE_FLOAT>(m(1, 2));
			dest[10] += static_cast<STORAGE_FLOAT>(m(2, 2));
			dest[11] += static_cast<STORAGE_FLOAT>(m(3, 2));

			dest[12] += static_cast<STORAGE_FLOAT>(m(0, 3));
			dest[13] += static_cast<STORAGE_FLOAT>(m(1, 3));
			dest[14] += static_cast<STORAGE_FLOAT>(m(2, 3));
			dest[15] += static_cast<STORAGE_FLOAT>(m(3, 3));
		}

		// General case
		else {
			for (int i = 0; i < BLOCK_ROWS; i++) {
				for (int j = 0; j < BLOCK_COLS; j++) {
					dest[j * BLOCK_ROWS + i] += static_cast<STORAGE_FLOAT>(m(i, j));
				}
			}
		}
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::set_preconditioner(const Preconditioner preconditioner)
	{
		this->preconditioner = preconditioner;
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::set_from_blocks(const Block<STORAGE_FLOAT, BLOCK_ROWS, BLOCK_COLS>* begin, const Block<STORAGE_FLOAT, BLOCK_ROWS, BLOCK_COLS>* end, const int n_rows, const int n_cols, const int n_threads)
	{
		const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() / 2 : n_threads;
		const int n_blocks = (int)std::distance(begin, end);
		
		this->start_insertion(n_rows, n_rows);
		#pragma omp parallel for schedule(static) num_threads(n_threads_)
		for (int i = 0; i < n_blocks; i++) {
			this->add_block(begin[i].base_row, begin[i].base_col, begin[i].vals);
		}
		this->end_insertion(n_threads);
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<typename ResizeableVector>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::spmxv(ResizeableVector& solution, const ResizeableVector& v, const int n_threads)
	{
		if ((int)v.size() != this->n_rows) {
			std::cout << "BlockedSparseMatrix.spmxv() error: v.size() != n_rows in the matrix. (" << v.size() << " != " << BLOCK_ROWS*this->n_block_rows << ")" << std::endl;
			exit(-1);
		}
		solution.resize(this->n_rows);
		this->_spmxv(solution.data(), v.data(), n_threads);
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<typename OPERATION_FLOAT>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::_spmxv(OPERATION_FLOAT* solution, const OPERATION_FLOAT* v, const int n_threads)
	{
		/*
		* Standard CRS procedure just with blocks instead of single numbers and using AVX.
		* 
		* Overview:
		*	- Note: While the whole matrix is stored in a row major layout, the individual blocks are stored column major.
		*		This is done so we can apply AVX along the columns of the blocks to avoid doing horizontal sums on the
		*		numbers in the AVX lines.
		*	- Use binary search on the CRS block row offset vector to partition the n_blocks in n_thread chunks for parallel
		*		execution.
		*	- In parallel for each block row range, we compute the dot product of the sparse block rows with the rhs vector.
		*	- Each row of the matrix is assigned to an entry in the AVX line.
		*	- The result of each dot product is accumulated in a temporal AVX register.
		*	- In the end, we copy the result of each dot product to the solution vector.
		*	- If the number of rows of the blocks is larger than what fits in a AVX line,
		*		multiple AVX lines are used per block column.
		*/

		if constexpr (std::is_same_v<OPERATION_FLOAT, float>) {
			std::cout << "BlockedSparseMatrix error: Currently it cannot take advantage of OPERATION_FLOAT = float. Please use double instead." << std::endl;
			exit(-1);
		}
		
		const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() / 2 : n_threads;
		const bool sequential = n_threads_ > this->n_block_rows;
		constexpr int BLOCK_SIZE = BLOCK_ROWS * BLOCK_COLS;
#ifdef BSM_ENABLE_AVX
		constexpr int NUMBERS_PER_AVX_LINE = sizeof(__m256d) / sizeof(OPERATION_FLOAT);
#else
		constexpr int NUMBERS_PER_AVX_LINE = 1;
#endif
		constexpr int N_AVX_LINES_PER_BLOCK_COLUMN = (BLOCK_ROWS % NUMBERS_PER_AVX_LINE == 0) ? BLOCK_ROWS / NUMBERS_PER_AVX_LINE : BLOCK_ROWS / NUMBERS_PER_AVX_LINE + 1;

#ifdef BSM_ENABLE_AVX
        if constexpr (N_AVX_LINES_PER_BLOCK_COLUMN > 1) {
            std::cout << "BlockedSparseMatrix.spmxv() error: Currently can't handle larger than 4x4 blocks (bug in solution scattering)." << std::endl;
            exit(-1);
        }
#endif

		// We spawn threads here
		#pragma omp parallel num_threads(n_threads_)
		{
			// Thread workload distribution computation: Each thread gets equal number of non-zero blocks
			const int thread_id = omp_get_thread_num();

			const int n_blocks = this->crs_current->rows.back();
			const int chunksize = n_blocks / n_threads_;

			const auto it_begin = this->crs_current->rows.begin();
			const auto it_end = this->crs_current->rows.end();

			// We use binary search (std::lower_bound) to find the block row range limits
			int block_row_begin = 0;
			int block_row_end = this->n_block_rows;
			if (sequential) {
				if (thread_id > 0) {
					block_row_end = 0;
				}
			} 
			else {
				if (thread_id != 0) {
					block_row_begin = (int)std::distance(it_begin, std::lower_bound(it_begin, it_end, thread_id * chunksize));
				}
				if (thread_id != n_threads_ - 1) {
					block_row_end = (int)std::distance(it_begin, std::lower_bound(it_begin + block_row_begin, it_end, (thread_id + 1) * chunksize));
				}
			}

			// Each thread works on its own block row range
			for (int block_row_i = block_row_begin; block_row_i < block_row_end; block_row_i++) {

				// Use the CRS row offset vector to fetch the begin and end of the blocks
				const int begin = this->crs_current->rows[block_row_i];
				const int end = this->crs_current->rows[block_row_i + 1];

				// Set a pointer (moving cursor) to the begining of the CRS base column indices and values
				STORAGE_FLOAT* vals_scalar = this->crs_current->vals.data() + BLOCK_SIZE * begin;
				int* block_col_ptr = this->crs_current->cols.data() + begin;

				// Initialize the result of the rows dot products in the AVX temporal lines
#ifdef BSM_ENABLE_AVX
				std::array<__m256d, N_AVX_LINES_PER_BLOCK_COLUMN> solution_local;
				for (int avx_i = 0; avx_i < N_AVX_LINES_PER_BLOCK_COLUMN; avx_i++) {
					solution_local[avx_i] = _mm256_set1_pd(0.0);
				}
#else
				std::array<OPERATION_FLOAT, N_AVX_LINES_PER_BLOCK_COLUMN> solution_local;
				for (int avx_i = 0; avx_i < N_AVX_LINES_PER_BLOCK_COLUMN; avx_i++) {
					solution_local[avx_i] = 0.0;
				}
#endif

				// Actual dot product operation:
				//// For each non-zero block in the block CRS
				for (int i = begin; i < end; i++) {

					// Get the first column of the block
					int col = *block_col_ptr;

					// For each block column
					for (int j = 0; j < BLOCK_COLS; j++) {

						// For each AVX line in the block column
						for (int avx_i = 0; avx_i < N_AVX_LINES_PER_BLOCK_COLUMN; avx_i++) {
#ifdef BSM_ENABLE_AVX
							// Fetch the values (if STORAGE_FLOAT != OPERATION_FLOAT -> cast)
							__m256d vals;
							if constexpr (std::is_same_v<STORAGE_FLOAT, float>) {
								vals = _mm256_cvtps_pd(_mm128_loadu_ps(vals_scalar + NUMBERS_PER_AVX_LINE * avx_i));
							}
							else if constexpr (std::is_same_v<STORAGE_FLOAT, double>) {
								vals = _mm256_loadu_pd(vals_scalar + NUMBERS_PER_AVX_LINE * avx_i);
							}

							// Add the dot product contribution of the current rows solution
							solution_local[avx_i] = _mm256_fmadd_pd(vals, _mm256_set1_pd(v[col]), solution_local[avx_i]);
#else
							OPERATION_FLOAT val = *(vals_scalar + NUMBERS_PER_AVX_LINE * avx_i);
							solution_local[avx_i] += val * v[col];
#endif
						}

						// Move to the next column index
						col++;

						// Move the pointer to the next column values
						vals_scalar += BLOCK_ROWS;
					}
					block_col_ptr++;
				}

				// Once `solution_local` contains the result of all rows dot products, copy them to the solution vector
                double* res_scalar = nullptr;
#ifdef BSM_ENABLE_AVX
                std::array<double, 4> res_scalar_arr;
				_mm256_storeu_pd(res_scalar_arr.data(), solution_local[0]);
                res_scalar = res_scalar_arr.data();
#else
                res_scalar = reinterpret_cast<double*>(&solution_local[0]);
#endif
				const int base_row = BLOCK_ROWS * block_row_i;
				for (int i = 0; i < BLOCK_ROWS; i++) {
					if (base_row + i < this->n_rows) {
						solution[base_row + i] = res_scalar[i];
					}
				}
			}
		}
	}
	
	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::spmxv_from_ptr(double* solution, const double* v, const int n_threads)
	{
		this->_spmxv(solution, v, n_threads);
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::prepare_preconditioning(const int n_threads)
	{
		const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() / 2 : n_threads;
		constexpr int BLOCK_SIZE = BLOCK_ROWS * BLOCK_COLS;

		if (this->preconditioner == Preconditioner::Diagonal) {
#ifdef BSM_ENABLE_AVX
			this->diag_inv.resize(this->n_block_rows * BLOCK_ROWS + sizeof(__m256)/sizeof(STORAGE_FLOAT));
#else
			this->diag_inv.resize(this->n_block_rows * BLOCK_ROWS);
#endif

			#pragma omp parallel for schedule(static) num_threads(n_threads_)
			for (int block_row_i = 0; block_row_i < this->n_block_rows; block_row_i++) {
				const int base_row_i = BLOCK_ROWS*block_row_i;
				for (int j = this->crs_current->rows[block_row_i]; j < this->crs_current->rows[block_row_i + 1]; j++) {
					if (this->crs_current->cols[j] == base_row_i) {
						STORAGE_FLOAT* vals = this->crs_current->vals.data() + BLOCK_SIZE * j;
						for (int i = 0; i < BLOCK_ROWS; i++) {
							this->diag_inv[BLOCK_ROWS * block_row_i + i] = static_cast<STORAGE_FLOAT>(1.0 / static_cast<double>(vals[i*BLOCK_ROWS + i]));
						}
						break;
					}
				}
			}
		}
		else if (this->preconditioner == Preconditioner::BlockDiagonal) {
#ifdef BSM_ENABLE_AVX
			this->diag_inv.resize(this->n_block_rows * BLOCK_SIZE + sizeof(__m256)/sizeof(STORAGE_FLOAT));
#else
			this->diag_inv.resize(this->n_block_rows * BLOCK_SIZE);
#endif

			#pragma omp parallel for schedule(static) num_threads(n_threads_)
			for (int block_row_i = 0; block_row_i < this->n_block_rows; block_row_i++) {
				const int base_row_i = BLOCK_ROWS * block_row_i;
				for (int j = this->crs_current->rows[block_row_i]; j < this->crs_current->rows[block_row_i + 1]; j++) {
					if (this->crs_current->cols[j] == base_row_i) {
						STORAGE_FLOAT* m = this->crs_current->vals.data() + BLOCK_SIZE * j;
						STORAGE_FLOAT* m_inv = this->diag_inv.data() + BLOCK_SIZE * block_row_i;

						// Code for the inverse of **symmetric** matrices generated with sympy
						if constexpr (BLOCK_ROWS == 2) {
							const STORAGE_FLOAT denom = m[2 * 0 + 0] * m[2 * 1 + 1] - m[2 * 0 + 1] * m[2 * 1 + 0];
							const STORAGE_FLOAT x0 = static_cast<STORAGE_FLOAT>(1.0 / static_cast<double>(denom));

							m_inv[2 * 0 + 0] = m[2 * 1 + 1] * x0;
							m_inv[2 * 0 + 1] = -m[2 * 0 + 1] * x0;
							m_inv[2 * 1 + 0] = -m[2 * 1 + 0] * x0;
							m_inv[2 * 1 + 1] = m[2 * 0 + 0] * x0;
						}
						else if constexpr (BLOCK_ROWS == 3) {
							const STORAGE_FLOAT tmp0 = m[3 * 1 + 1] * m[3 * 2 + 2];
							const STORAGE_FLOAT tmp1 = ((m[3 * 1 + 2]) * (m[3 * 1 + 2]));
							const STORAGE_FLOAT tmp2 = m[3 * 0 + 2] * m[3 * 1 + 2];
							const STORAGE_FLOAT tmp3 = ((m[3 * 0 + 1]) * (m[3 * 0 + 1]));
							const STORAGE_FLOAT tmp4 = ((m[3 * 0 + 2]) * (m[3 * 0 + 2]));
							const STORAGE_FLOAT tmp5 = static_cast<STORAGE_FLOAT>(1.0 / static_cast<double>(m[3 * 0 + 0] * tmp0 - m[3 * 0 + 0] * tmp1 + 2 * m[3 * 0 + 1] * tmp2 - m[3 * 1 + 1] * tmp4 - m[3 * 2 + 2] * tmp3));
							m_inv[3 * 2 + 2] = tmp5 * (m[3 * 0 + 0] * m[3 * 1 + 1] - tmp3);
							m_inv[3 * 1 + 1] = tmp5 * (m[3 * 0 + 0] * m[3 * 2 + 2] - tmp4);
							m_inv[3 * 0 + 0] = tmp5 * (tmp0 - tmp1);
							m_inv[3 * 1 + 0] = -tmp5 * (m[3 * 0 + 1] * m[3 * 2 + 2] - tmp2);
							m_inv[3 * 0 + 1] = m_inv[3 * 1 + 0];
							m_inv[3 * 2 + 0] = tmp5 * (m[3 * 0 + 1] * m[3 * 1 + 2] - m[3 * 1 + 1] * m[3 * 0 + 2]);
							m_inv[3 * 0 + 2] = m_inv[3 * 2 + 0];
							m_inv[3 * 2 + 1] = -tmp5 * (m[3 * 0 + 0] * m[3 * 1 + 2] - m[3 * 0 + 1] * m[3 * 0 + 2]);
							m_inv[3 * 1 + 2] = m_inv[3 * 2 + 1];
						}
						else if constexpr (BLOCK_ROWS == 4) {
							const STORAGE_FLOAT tmp0 = m[4 * 2 + 2] * m[4 * 3 + 3];
							const STORAGE_FLOAT tmp1 = m[4 * 1 + 1] * tmp0;
							const STORAGE_FLOAT tmp2 = m[4 * 1 + 3] * m[4 * 2 + 3];
							const STORAGE_FLOAT tmp3 = 2 * m[4 * 1 + 2];
							const STORAGE_FLOAT tmp4 = ((m[4 * 2 + 3]) * (m[4 * 2 + 3]));
							const STORAGE_FLOAT tmp5 = m[4 * 1 + 1] * tmp4;
							const STORAGE_FLOAT tmp6 = ((m[4 * 1 + 2]) * (m[4 * 1 + 2]));
							const STORAGE_FLOAT tmp7 = m[4 * 3 + 3] * tmp6;
							const STORAGE_FLOAT tmp8 = ((m[4 * 1 + 3]) * (m[4 * 1 + 3]));
							const STORAGE_FLOAT tmp9 = m[4 * 2 + 2] * tmp8;
							const STORAGE_FLOAT tmp10 = m[4 * 0 + 0] * tmp2;
							const STORAGE_FLOAT tmp11 = m[4 * 1 + 2] * m[4 * 3 + 3];
							const STORAGE_FLOAT tmp12 = m[4 * 0 + 2] * tmp11;
							const STORAGE_FLOAT tmp13 = 2 * m[4 * 0 + 1];
							const STORAGE_FLOAT tmp14 = m[4 * 0 + 2] * tmp2;
							const STORAGE_FLOAT tmp15 = m[4 * 0 + 3] * m[4 * 2 + 3];
							const STORAGE_FLOAT tmp16 = m[4 * 1 + 2] * tmp15;
							const STORAGE_FLOAT tmp17 = m[4 * 0 + 3] * m[4 * 1 + 3];
							const STORAGE_FLOAT tmp18 = m[4 * 2 + 2] * tmp17;
							const STORAGE_FLOAT tmp19 = m[4 * 1 + 1] * tmp15;
							const STORAGE_FLOAT tmp20 = 2 * m[4 * 0 + 2];
							const STORAGE_FLOAT tmp21 = m[4 * 1 + 2] * tmp17;
							const STORAGE_FLOAT tmp22 = ((m[4 * 0 + 1]) * (m[4 * 0 + 1]));
							const STORAGE_FLOAT tmp23 = ((m[4 * 0 + 2]) * (m[4 * 0 + 2]));
							const STORAGE_FLOAT tmp24 = ((m[4 * 0 + 3]) * (m[4 * 0 + 3]));
							const STORAGE_FLOAT tmp25 = m[4 * 3 + 3] * tmp22;
							const STORAGE_FLOAT tmp26 = m[4 * 3 + 3] * tmp23;
							const STORAGE_FLOAT tmp27 = m[4 * 2 + 2] * tmp24;
							const STORAGE_FLOAT tmp28 = static_cast<STORAGE_FLOAT>(1.0 / static_cast<double>(m[4 * 0 + 0] * tmp1 - m[4 * 0 + 0] * tmp5 - m[4 * 0 + 0] * tmp7 - m[4 * 0 + 0] * tmp9 - m[4 * 1 + 1] * tmp26 - m[4 * 1 + 1] * tmp27 - m[4 * 2 + 2] * tmp25 + tmp10 * tmp3 + tmp12 * tmp13 - tmp13 * tmp14 - tmp13 * tmp16 + tmp13 * tmp18 + tmp19 * tmp20 - tmp20 * tmp21 + tmp22 * tmp4 + tmp23 * tmp8 + tmp24 * tmp6));
							m_inv[4 * 1 + 1] = tmp28 * (m[4 * 0 + 0] * tmp0 - m[4 * 0 + 0] * tmp4 + tmp15 * tmp20 - tmp26 - tmp27);
							m_inv[4 * 0 + 0] = tmp28 * (tmp1 + tmp2 * tmp3 - tmp5 - tmp7 - tmp9);
							m_inv[4 * 1 + 0] = tmp28 * (-m[4 * 0 + 1] * tmp0 + m[4 * 0 + 1] * tmp4 + tmp12 - tmp14 - tmp16 + tmp18);
							m_inv[4 * 0 + 1] = m_inv[4 * 1 + 0];
							const STORAGE_FLOAT tmp29 = m[4 * 1 + 1] * m[4 * 0 + 2];
							m_inv[4 * 2 + 0] = tmp28 * (m[4 * 0 + 1] * tmp11 - m[4 * 0 + 1] * tmp2 + m[4 * 0 + 2] * tmp8 - m[4 * 3 + 3] * tmp29 + tmp19 - tmp21);
							m_inv[4 * 0 + 2] = m_inv[4 * 2 + 0];
							const STORAGE_FLOAT tmp30 = m[4 * 0 + 1] * m[4 * 2 + 2];
							const STORAGE_FLOAT tmp31 = m[4 * 1 + 2] * m[4 * 2 + 3];
							const STORAGE_FLOAT tmp32 = m[4 * 1 + 1] * m[4 * 2 + 2];
							const STORAGE_FLOAT tmp33 = m[4 * 0 + 2] * m[4 * 1 + 2];
							m_inv[4 * 3 + 3] = tmp28 * (m[4 * 0 + 0] * tmp32 - m[4 * 0 + 0] * tmp6 - m[4 * 1 + 1] * tmp23 - m[4 * 2 + 2] * tmp22 + tmp13 * tmp33);
							m_inv[4 * 3 + 0] = tmp28 * (-m[4 * 0 + 1] * tmp31 - m[4 * 0 + 3] * tmp32 + m[4 * 0 + 3] * tmp6 + m[4 * 1 + 3] * tmp30 - m[4 * 1 + 3] * tmp33 + m[4 * 2 + 3] * tmp29);
							m_inv[4 * 0 + 3] = m_inv[4 * 3 + 0];
							const STORAGE_FLOAT tmp34 = m[4 * 0 + 1] * m[4 * 0 + 2];
							m_inv[4 * 2 + 1] = tmp28 * (-m[4 * 0 + 0] * tmp11 - m[4 * 0 + 1] * tmp15 - m[4 * 0 + 2] * tmp17 + m[4 * 1 + 2] * tmp24 + m[4 * 3 + 3] * tmp34 + tmp10);
							m_inv[4 * 1 + 2] = m_inv[4 * 2 + 1];
							const STORAGE_FLOAT tmp35 = m[4 * 0 + 0] * m[4 * 1 + 3];
							m_inv[4 * 3 + 1] = tmp28 * (m[4 * 0 + 0] * tmp31 - m[4 * 2 + 2] * tmp35 + m[4 * 0 + 3] * tmp30 - m[4 * 0 + 3] * tmp33 + m[4 * 1 + 3] * tmp23 - m[4 * 2 + 3] * tmp34);
							m_inv[4 * 1 + 3] = m_inv[4 * 3 + 1];
							const STORAGE_FLOAT tmp36 = m[4 * 0 + 0] * m[4 * 1 + 1];
							m_inv[4 * 2 + 2] = tmp28 * (-m[4 * 0 + 0] * tmp8 - m[4 * 1 + 1] * tmp24 + m[4 * 3 + 3] * tmp36 + tmp13 * tmp17 - tmp25);
							m_inv[4 * 3 + 2] = tmp28 * (-m[4 * 0 + 1] * m[4 * 1 + 2] * m[4 * 0 + 3] + m[4 * 1 + 2] * tmp35 + m[4 * 0 + 3] * tmp29 - m[4 * 1 + 3] * tmp34 + m[4 * 2 + 3] * tmp22 - m[4 * 2 + 3] * tmp36);
							m_inv[4 * 2 + 3] = m_inv[4 * 3 + 2];
						}
						else {
							std::cout << "BlockedSparseMatrix error: Only 2x2, 3x3 and 4x4 blocks are supported for BlockPreconditioning." << std::endl;
							exit(-1);
						}
					}
				}
			}
		}
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<typename OPERATION_FLOAT>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::apply_preconditioning(OPERATION_FLOAT* solution, const OPERATION_FLOAT* v, const int n_threads)
	{
		if constexpr (std::is_same_v<OPERATION_FLOAT, float>) {
			std::cout << "BlockedSparseMatrix error: Currently it cannot take advantage of OPERATION_FLOAT = float. Please use double instead." << std::endl;
			exit(-1);
		}

		constexpr int BLOCK_SIZE = BLOCK_ROWS * BLOCK_COLS;
		const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() / 2 : n_threads;

		if (this->preconditioner == Preconditioner::Diagonal) {
#ifdef BSM_ENABLE_AVX
			const int n_avx = this->n_rows / 4;

			#pragma omp parallel for schedule(static) num_threads(n_threads_)
			for (int i = 0; i < this->n_rows; i += 4) {
				_mm256_storeu_pd(&solution[i], _mm256_mul_pd(_mm256_loadu_pd(&this->diag_inv[i]), _mm256_loadu_pd(&v[i])) );
            }

			for (int i = 4 * (this->n_rows / 4); i < this->n_rows; i++) {
				solution[i] = this->diag_inv[i] * v[i];
			}
#else
			for (int i = 0; i < this->n_rows; i++) {
				solution[i] = this->diag_inv[i] * v[i];
			}
#endif
		}
		else if (this->preconditioner == Preconditioner::BlockDiagonal) {
			#pragma omp parallel for schedule(static) num_threads(n_threads_)
			for (int block_row_i = 0; block_row_i < this->n_block_rows; block_row_i++) {
				const int base_row = BLOCK_ROWS * block_row_i;

				STORAGE_FLOAT* vals_scalar = this->diag_inv.data() + BLOCK_SIZE * block_row_i;

#ifdef BSM_ENABLE_AVX
				__m256d sol = _mm256_set1_pd(0.0);
				for (int col = 0; col < BLOCK_COLS; col++) {
					__m256d vals;
					if constexpr (std::is_same_v<STORAGE_FLOAT, float>) {
						vals = _mm256_cvtps_pd(_mm128_loadu_ps(vals_scalar + BLOCK_ROWS * col));
					}
					else if constexpr (std::is_same_v<STORAGE_FLOAT, double>) {
						vals = _mm256_loadu_pd(vals_scalar + BLOCK_ROWS * col);
					}

					sol = _mm256_fmadd_pd(vals, _mm256_set1_pd(v[base_row + col]), sol);
				}

				static_assert(sizeof(__m256d) / sizeof(double) >= BLOCK_ROWS);

                std::array<double, 4> res_scalar_arr;
				_mm256_storeu_pd(res_scalar_arr.data(), sol);
                const double* res_scalar = res_scalar_arr.data();
#else
				std::array<double, BLOCK_ROWS> sol;
				for (int i = 0; i < BLOCK_ROWS; i++) {
					sol[i] = 0.0;
					for (int col = 0; col < BLOCK_COLS; col++) {
						double val = *(vals_scalar + BLOCK_ROWS * col + i);
						sol[i] += val * v[base_row + col];
					}
				}

				const double* res_scalar = sol.data();
#endif

				for (int i = 0; i < BLOCK_ROWS; i++) {
					if (base_row + i < this->n_rows) {
						solution[base_row + i] = res_scalar[i];
					}
				}
			}
		}
	}

	template<size_t BLOCK_ROWS, size_t BLOCK_COLS, typename STORAGE_FLOAT>
	template<typename Triplet>
	inline void BlockedSparseMatrix<BLOCK_ROWS, BLOCK_COLS, STORAGE_FLOAT>::to_triplets(std::vector<Triplet>& triplets)
	{
		triplets.clear();

		for (int block_row_i = 0; block_row_i < this->n_block_rows; block_row_i++) {
			const int base_row = BLOCK_ROWS * block_row_i;

			// Use the CRS row offset vector to fetch the begin and end of the blocks
			const int begin = this->crs_current->rows[block_row_i];
			const int end = this->crs_current->rows[block_row_i + 1];

			// Set a pointer (moving cursor) to the begining of the CRS base column indices and values
			STORAGE_FLOAT* vals_scalar = this->crs_current->vals.data() + BLOCK_ROWS*BLOCK_COLS * begin;
			int* block_col_ptr = this->crs_current->cols.data() + begin;

			for (int block_i = begin; block_i < end; block_i++) {
				int col = *block_col_ptr;
				for (int j = 0; j < BLOCK_COLS; j++) {
					for (int i = 0; i < BLOCK_ROWS; i++) {
						triplets.push_back({ base_row + i, col, *vals_scalar });
						vals_scalar++;
					}
					col++;
				}
				block_col_ptr++;
			}
		}
	}
}

