#pragma once
#include <array>

#include "AlignmentAllocator.h"

namespace bsm
{
	enum class Preconditioner { Diagonal, BlockDiagonal };
	enum class Ordering { RowMajor, ColMajor };
	enum class ThreadSafety { UseMutex, Unsafe };

	template<typename T>
	struct Triplet
	{
		int _row = -1;
		int _col = -1;
		T _val;

		template<typename FLOAT>
		Triplet(const int row, const int col, const FLOAT val) :
			_row(row), _col(col), _val(static_cast<T>(val)) {};

		// Eigen::Triplet interface
		const int& row() const { return this->_row; };
		const int& col() const { return this->_col; };
		const T& value() const { return this->_val; };
	};

	template<typename T, size_t NROWS, size_t NCOLS>
	struct Block
	{
		int base_row = -1;
		int base_col = -1;
		std::array<T, NROWS*NCOLS> vals; // Col Major

		template<typename MATRIX_PARENTHESIS_INDEXABLE>
		Block(const int base_row, const int base_col, const MATRIX_PARENTHESIS_INDEXABLE& m)
			: base_row(base_row), base_col(base_col)
		{
			for (int j = 0; j < NCOLS; j++) {
				for (int i = 0; i < NROWS; i++) {
					this->vals[NROWS * j + i] = static_cast<T>(m(i, j));
				}
			}
		}
		Block() = default;
	};

	template<typename T, size_t NROWS, size_t NCOLS>
	struct ColBlockTemplated
	{
		int base_col = -1;
		std::array<T, NROWS*NCOLS> vals; // Col Major (faster AVX block spmxv)

		template<typename MATRIX_PARENTHESIS_INDEXABLE>
		ColBlockTemplated(const int base_col, const MATRIX_PARENTHESIS_INDEXABLE& m)
			: base_col(base_col)
		{
			for (int j = 0; j < NCOLS; j++) {
				for (int i = 0; i < NROWS; i++) {
					this->vals[NROWS * j + i] = static_cast<T>(m(i, j));
				}
			}
		}
		ColBlockTemplated() = default;
	};

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
}
