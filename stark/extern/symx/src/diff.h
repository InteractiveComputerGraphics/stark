#pragma once
#include <iostream>
#include <vector>
#include <cassert>

#include "utils.h"
#include "Scalar.h"
#include "Vector.h"
#include "Matrix.h"

namespace symx
{
	Scalar diff(const Scalar& expr, const Scalar& symbol);
	Vector gradient(const Scalar& expr, const std::vector<Scalar>& symbols);
	Matrix hessian(const Scalar& expr, const std::vector<Scalar>& symbols, const bool symmetric = true);
	std::vector<Scalar> value_gradient(const Scalar& expr, const std::vector<Scalar>& symbols);
	Matrix hessian_as_symmetric_lower_triangular(const Scalar& expr, const std::vector<Scalar>& symbols);
	std::vector<Scalar> value_gradient_hessian(const Scalar& expr, const std::vector<Scalar>& symbols, const bool symmetric = true);
	std::vector<Scalar> value_gradient_hessian_blocked(const Scalar& expr, const std::vector<Scalar>& symbols, const int block_size, const bool symmetric = true);
}