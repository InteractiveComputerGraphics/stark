#pragma once
#include <string>
#include <vector>
#include <array>
#include <cstring>

#include "Expressions.h"
#include "Scalar.h"
#include "Vector.h"
#include "Matrix.h"

namespace symx
{
	class SymbolicWorkSpace
	{
	public:
		/* Fields */
		Expressions expressions;

	public:
		/* Methods */
		// Standalone symbols
		Scalar make_scalar(const std::string label);
		Vector make_vector(const std::string label, const int32_t size);
		Matrix make_matrix(const std::string label, const std::array<int32_t, 2> shape);
		Matrix make_symmetric_matrix(const std::string label, const int32_t rows, const int32_t cols);

		std::vector<Scalar> make_scalars(const std::string label, const int32_t n);
		std::vector<Vector> make_vectors(const std::string label, const int32_t size, const int32_t n);

		Scalar make_branch(const Scalar& condition, const Scalar& positive_branch, const Scalar& negative_branch);
		Scalar get_zero();
		Scalar get_one();
		Vector get_zero_vector(const int32_t size);
		Matrix get_zero_matrix(const std::array<int32_t, 2> shape);
		Matrix get_zero_symmetric_matrix(const std::array<int32_t, 2> shape);
		Matrix get_identity_matrix(const int32_t size);
	};
}