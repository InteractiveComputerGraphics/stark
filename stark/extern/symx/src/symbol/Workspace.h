#pragma once
#include <string>
#include <vector>
#include <array>
#include <cstring>
#include <memory>

#include "Expressions.h"
#include "Scalar.h"
#include "Vector.h"
#include "Matrix.h"

namespace symx
{
	class Workspace
	{
	public:
		/* Methods */
		Workspace();

		Scalar make_scalar(const std::string label);
		Scalar get_scalar(const int32_t symbol_id);
		Vector make_vector(const std::string label, const int32_t size);
		Matrix make_matrix(const std::string label, const std::array<int32_t, 2> shape);

		std::vector<Scalar> make_scalars(const std::string label, const int32_t n);
		std::vector<Vector> make_vectors(const std::string label, const int32_t size, const int32_t n);
		std::vector<Matrix> make_matrices(const std::string label, const std::array<int32_t, 2> shape, const int32_t n);

		Scalar get_zero();
		Scalar get_one();
		Vector get_zero_vector(const int32_t size);
		Matrix get_zero_matrix(const std::array<int32_t, 2> shape);
		Matrix get_identity_matrix(const int32_t size);

		int32_t get_n_symbols() const;
		std::string get_checksum() const;
		const Expressions& get_expression_graph() const;

	private:
		/* Fields */
		Expressions expressions;
	};
}