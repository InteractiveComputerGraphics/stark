#include "diff.h"
#include <cmath>

using namespace symx;

// =====================================================================================================
Scalar diff_impl(const Scalar& scalar, const Scalar& wrt, DiffCache& diff_cache)
{
	// Bottom
	if (scalar.expr.type == ExprType::Symbol) {
		if (scalar.expr.a == wrt.expr.a) {
			return scalar.get_one();
		}
		else {
			return scalar.get_zero();
		}
	}
	else if (scalar.expr.type == ExprType::Zero || scalar.expr.type == ExprType::One || scalar.expr.type == ExprType::ConstantFloat) {
		return scalar.get_zero();
	}

	// Traversal
	else {

		// Special operations
		if (scalar.expr.type == ExprType::Branch) {
			return branch(scalar.get_condition(), diff(scalar.left(), wrt, diff_cache), diff(scalar.right(), wrt, diff_cache));
		}
		else if (scalar.expr.type == ExprType::PowN) {
			Scalar u = scalar.left();
			Scalar du = diff(u, wrt, diff_cache);
			int32_t c = scalar.expr.b;
			return (double)(c)*u.powN(c - 1) * du;
		}
		else if (scalar.expr.type == ExprType::PowF) {
			std::cout << "symx error: ExprType::PowF not supported for diff_impl." << std::endl;
			exit(-1);
		}

		// Regular operations
		const Scalar u = scalar.left();
		const Scalar du = diff(u, wrt, diff_cache);
		const Scalar v = (scalar.has_not_right()) ? u : scalar.right();
		const Scalar dv = (scalar.has_not_right()) ? du : diff(v, wrt, diff_cache);

		switch (scalar.expr.type)
		{
		case ExprType::Add:
			return du + dv;
		case ExprType::Sub:
			return du - dv;
		case ExprType::Mul:
			return u * dv + v * du;
		case ExprType::Reciprocal:
			return -du / u.powN(2);
		case ExprType::Sqrt:
			return 0.5 / sqrt(u) * du;
		case ExprType::Ln:
			return du / u;
		case ExprType::Log10:
			return du / (u * std::log(10.0));
		case ExprType::Exp:
			return exp(u) * du;
		case ExprType::Sin:
			return cos(u) * du;
		case ExprType::Cos:
			return -sin(u) * du;
		case ExprType::Tan:
			return (1.0 + tan(u).powN(2)) * du;
		case ExprType::ArcSin:
			return du / (sqrt(1.0 - u.powN(2)));
		case ExprType::ArcCos:
			return -du / (sqrt(1.0 - u.powN(2)));
		case ExprType::ArcTan:
			return du / (1.0 + u.powN(2));
		case ExprType::Print:
			return u.make_constant(0.0);

		default:
			std::cout << "symx error: Unhandled ExprType found in diff_impl." << std::endl;
			exit(-1);
		}
	}
}
// =====================================================================================================


Scalar symx::diff(const Scalar& scalar, const Scalar& wrt)
{
	DiffCache diff_cache;
	return diff_impl(scalar, wrt, diff_cache);
}
Scalar symx::diff(const Scalar& scalar, const Scalar& wrt, DiffCache& diff_cache)
{
	// Check cache
	int32_t id32[2];
	id32[0] = scalar.expr_id;
	id32[1] = wrt.expr_id;
	int64_t id = *reinterpret_cast<int64_t*>(&id32[0]);

	auto it = diff_cache.find(id);
	if (it != diff_cache.end()) {
		return it->second;
	}
	else {
		auto new_diff = diff_impl(scalar, wrt, diff_cache);
		diff_cache.insert({ id, new_diff });
		return new_diff;
	}
}
Vector symx::gradient(const Scalar &expr, const std::vector<Scalar> &wrt)
{
	DiffCache diff_cache;
	return gradient(expr, wrt, diff_cache);
}
Vector symx::gradient(const Scalar &expr, const std::vector<Scalar> &wrt, DiffCache& diff_cache)
{
	std::vector<Scalar> vals;
	for (const Scalar& symbol : wrt) {
		vals.push_back(diff(expr, symbol, diff_cache));
	}
	return Vector(vals);
}

Vector symx::gradient(const Scalar& expr, const Vector& wrt)
{
	DiffCache diff_cache;
	return gradient(expr, wrt, diff_cache);
}
Vector symx::gradient(const Scalar& expr, const Vector& wrt, DiffCache& diff_cache)
{
	const int n = wrt.size();
	std::vector<Scalar> vals;
	for (int i = 0; i < n; i++) {
		vals.push_back(diff(expr, wrt[i], diff_cache));
	}
	return Vector(vals);
}

Matrix symx::gradient(const Vector& vec, const std::vector<Scalar>& wrt, const bool symmetric)
{
	DiffCache diff_cache;
	return gradient(vec, wrt, symmetric, diff_cache);
}
Matrix symx::gradient(const Vector& vec, const std::vector<Scalar>& wrt, const bool symmetric, DiffCache& diff_cache)
{
	const int nrows = vec.size();
	const int ncols = (int)wrt.size();

	Matrix m = Matrix::zero({ nrows, ncols }, vec[0]);

	if (symmetric) {
		for (int i = 0; i < nrows; i++) {
			for (int j = 0; j < ncols; j++) {
				if (j >= i) {
					m(i, j) = diff(vec[i], wrt[j], diff_cache);  // upper triangle
				} else {
					m(i, j) = m(j, i);  // mirror
				}
			}
		}
	} else {
		for (int i = 0; i < nrows; i++) {
			for (int j = 0; j < ncols; j++) {
				m(i, j) = diff(vec[i], wrt[j], diff_cache);
			}
		}
	}

	return m;
}

Matrix symx::gradient(const Vector& vec, const Vector& wrt, const bool symmetric)
{
	DiffCache diff_cache;
	return gradient(vec, wrt, symmetric, diff_cache);
}
Matrix symx::gradient(const Vector& vec, const Vector& wrt, const bool symmetric, DiffCache& diff_cache)
{
	return gradient(vec, wrt.values(), symmetric, diff_cache);
}

Matrix symx::hessian(const Scalar& expr, const std::vector<Scalar>& wrt)
{
	DiffCache diff_cache;
	return hessian(expr, wrt, diff_cache);
}
Matrix symx::hessian(const Scalar& expr, const std::vector<Scalar>& wrt, DiffCache& diff_cache)
{
	Vector g = gradient(expr, wrt, diff_cache);
	return gradient(g, wrt, true, diff_cache);
}

Matrix symx::hessian(const Scalar& expr, const Vector& wrt)
{
	DiffCache diff_cache;
	return hessian(expr, wrt, diff_cache);
}
Matrix symx::hessian(const Scalar& expr, const Vector& wrt, DiffCache& diff_cache)
{
	return hessian(expr, wrt.values(), diff_cache);
}

std::vector<Scalar> symx::value_gradient_hessian(const Scalar& expr, const std::vector<Scalar>& wrt)
{
	DiffCache diff_cache;
	return value_gradient_hessian(expr, wrt, diff_cache);
}
std::vector<Scalar> symx::value_gradient_hessian(const Scalar& expr, const std::vector<Scalar>& wrt, DiffCache& diff_cache)
{
	Vector g = gradient(expr, wrt, diff_cache);
	Matrix h = gradient(g, wrt, true, diff_cache);
	return gather({{expr}, g.values(), h.values()});
}
std::vector<Scalar> symx::value_gradient(const Scalar& expr, const std::vector<Scalar>& wrt)
{
	DiffCache diff_cache;
	return value_gradient(expr, wrt, diff_cache);
}
std::vector<Scalar> symx::value_gradient(const Scalar& expr, const std::vector<Scalar>& wrt, DiffCache& diff_cache)
{
	Vector g = gradient(expr, wrt, diff_cache);
	return gather({{expr}, g.values()});
}
