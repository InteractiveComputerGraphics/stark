#include "diff.h"

symx::Vector symx::gradient(const Scalar& expr, const std::vector<Scalar>& symbols)
{
	std::unordered_map<int64_t, Scalar> diff_map;
	std::vector<Scalar> vals;
	for (const Scalar& symbol : symbols) {
		vals.push_back(symx::diff(expr, symbol, &diff_map));
	}
	return Vector(vals);
}
symx::Matrix symx::hessian(const Scalar& expr, const std::vector<Scalar>& symbols, const bool symmetric)
{
	const int n = (int)symbols.size();
	std::unordered_map<int64_t, Scalar> diff_map;
	std::vector<Scalar> vals;
	if (symmetric) {
		for (int i = 0; i < n; i++) {
			symx::Scalar g = symx::diff(expr, symbols[i], &diff_map);
			for (int j = 0; j < i; j++) {
				vals.push_back(vals[j * n + i]);
			}
			for (int j = i; j < n; j++) {
				Scalar h = symx::diff(g, symbols[j], &diff_map);
				vals.push_back(h);
			}
		}
	}
	else {
		for (int i = 0; i < n; i++) {
			symx::Scalar g = symx::diff(expr, symbols[i], &diff_map);
			for (int j = 0; j < n; j++) {
				vals.push_back(symx::diff(g, symbols[j], &diff_map));
			}
		}
	}
	return Matrix(vals, { n, n });
}
std::vector<symx::Scalar> symx::value_gradient_hessian(const Scalar& expr, const std::vector<Scalar>& symbols, const bool symmetric)
{
	const int n = (int)symbols.size();
	std::unordered_map<int64_t, Scalar> diff_map;
	std::vector<Scalar> vals;
	vals.push_back(expr);
	for (int i = 0; i < n; i++) {
		symx::Scalar g = symx::diff(expr, symbols[i], &diff_map);
		vals.push_back(g);
	}
	if (symmetric) {
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < i; j++) {
				Scalar* matrix = &vals[1 + n];
				vals.push_back(matrix[j * n + i]);
			}
			for (int j = i; j < n; j++) {
				Scalar h = symx::diff(vals[1 + i], symbols[j], &diff_map);
				vals.push_back(h);
			}
		}
	}
	else {
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				Scalar h = symx::diff(vals[1 + i], symbols[j], &diff_map);
				vals.push_back(h);
			}
		}
	}
	return vals;
}
std::vector<symx::Scalar> symx::value_gradient_hessian_blocked(const Scalar& expr, const std::vector<Scalar>& symbols, const int block_size, const bool symmetric)
{
	std::vector<symx::Scalar> vals = symx::value_gradient_hessian(expr, symbols, symmetric);
	symx::reorder_in_blocks(vals.data() + symbols.size() + 1, (int)symbols.size(), block_size);
	return vals;
}
std::vector<symx::Scalar> symx::value_gradient(const Scalar& expr, const std::vector<Scalar>& symbols)
{
	std::unordered_map<int64_t, Scalar> diff_map;
	const int n = (int)symbols.size();
	std::vector<Scalar> vals;
	vals.push_back(expr);
	for (int i = 0; i < n; i++) {
		symx::Scalar g = symx::diff(expr, symbols[i], &diff_map);
		vals.push_back(g);
	}
	return vals;
}
symx::Scalar symx::diff(const Scalar& scalar, const Scalar& wrt, std::unordered_map<int64_t, Scalar>* diff_map)
{
	if (diff_map != nullptr) {
		int32_t id32[2];
		id32[0] = scalar.expr_id;
		id32[1] = wrt.expr_id;
		int64_t id = *reinterpret_cast<int64_t*>(&id32[0]);

		auto it = diff_map->find(id);
		if (it != diff_map->end()) {
			return it->second;
		}
		else {
			auto new_diff = diff_impl(scalar, wrt, diff_map);
			diff_map->insert({ id, new_diff });
			return new_diff;
		}
	}
	else {
		return diff_impl(scalar, wrt, nullptr);
	}
}
symx::Scalar symx::diff_impl(const Scalar& scalar, const Scalar& wrt, std::unordered_map<int64_t, Scalar>* diff_map)
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
			return scalar.make_branch(scalar.get_condition(), diff(scalar.left(), wrt, diff_map), diff(scalar.right(), wrt, diff_map));
		}
		else if (scalar.expr.type == ExprType::PowN) {
			Scalar u = scalar.left();
			Scalar du = diff(u, wrt, diff_map);
			int32_t c = scalar.expr.b;
			return (double)(c)*u.powN(c - 1) * du;
		}
		else if (scalar.expr.type == ExprType::PowF) {
			Scalar u = scalar.left();
			Scalar du = diff(u, wrt, diff_map);
			double v = scalar.expr.unpack_double();
			return v * u.powF(v - 1.0) * du;
		}

		// Regular operations
		const Scalar u = scalar.left();
		const Scalar du = diff(u, wrt, diff_map);
		const Scalar v = (scalar.has_not_right()) ? u : scalar.right();
		const Scalar dv = (scalar.has_not_right()) ? du : diff(v, wrt, diff_map);

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
			std::cout << "symx error: Unhandled ExprType found in diff." << std::endl;
			exit(-1);
		}
	}
}
