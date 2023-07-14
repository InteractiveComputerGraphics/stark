#pragma once
#include <array>

#include "AlignmentAllocator.h"

namespace bsm
{
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
		std::array<T, NROWS*NCOLS> vals; // Col Major

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
}
