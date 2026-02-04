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
	using DiffCache = std::unordered_map<int64_t, Scalar>;
	
	// Basic diff functions
	Scalar diff(const Scalar& expr, const Scalar& wrt);
	Scalar diff(const Scalar& expr, const Scalar& wrt, DiffCache& diff_cache);

	// Gradient functions (scalar -> vector)
	Vector gradient(const Scalar& expr, const std::vector<Scalar>& wrt);
	Vector gradient(const Scalar& expr, const std::vector<Scalar>& wrt, DiffCache& diff_cache);
	Vector gradient(const Scalar& expr, const Vector& wrt);
	Vector gradient(const Scalar& expr, const Vector& wrt, DiffCache& diff_cache);
	
	// Gradient functions (vector -> matrix, Jacobian)
	Matrix gradient(const Vector& vec, const std::vector<Scalar>& wrt, const bool symmetric);
	Matrix gradient(const Vector& vec, const std::vector<Scalar>& wrt, const bool symmetric, DiffCache& diff_cache);
	Matrix gradient(const Vector& vec, const Vector& wrt, const bool symmetric);
	Matrix gradient(const Vector& vec, const Vector& wrt, const bool symmetric, DiffCache& diff_cache);

	// Hessian functions
	Matrix hessian(const Scalar& expr, const std::vector<Scalar>& wrt);
	Matrix hessian(const Scalar& expr, const std::vector<Scalar>& wrt, DiffCache& diff_cache);
	Matrix hessian(const Scalar& expr, const Vector& wrt);
	Matrix hessian(const Scalar& expr, const Vector& wrt, DiffCache& diff_cache);

	// Combined value/gradient/hessian functions
	std::vector<Scalar> value_gradient(const Scalar& expr, const std::vector<Scalar>& wrt);
	std::vector<Scalar> value_gradient(const Scalar& expr, const std::vector<Scalar>& wrt, DiffCache& diff_cache);
	std::vector<Scalar> value_gradient_hessian(const Scalar& expr, const std::vector<Scalar>& wrt);
	std::vector<Scalar> value_gradient_hessian(const Scalar& expr, const std::vector<Scalar>& wrt, DiffCache& diff_cache);
}
