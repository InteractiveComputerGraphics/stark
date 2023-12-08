#include "diff.h"

symx::Vector symx::gradient(const Scalar& expr, const std::vector<Scalar>& symbols)
{
	std::vector<Scalar> vals;
	for (const Scalar& symbol : symbols) {
		vals.push_back(symx::diff(expr, symbol));
	}
	return Vector(vals);
}
symx::Matrix symx::hessian(const Scalar& expr, const std::vector<Scalar>& symbols, const bool symmetric)
{
	const int n = (int)symbols.size();
	std::vector<Scalar> vals;
	if (symmetric) {
		for (int i = 0; i < n; i++) {
			symx::Scalar g = symx::diff(expr, symbols[i]);
			for (int j = 0; j < i; j++) {
				vals.push_back(vals[j * n + i]);
			}
			for (int j = i; j < n; j++) {
				Scalar h = symx::diff(g, symbols[j]);
				vals.push_back(h);
			}
		}
	}
	else {
		for (int i = 0; i < n; i++) {
			symx::Scalar g = symx::diff(expr, symbols[i]);
			for (int j = 0; j < n; j++) {
				vals.push_back(symx::diff(g, symbols[j]));
			}
		}
	}
	return Matrix(vals, { n, n });
}
std::vector<symx::Scalar> symx::value_gradient_hessian(const Scalar& expr, const std::vector<Scalar>& symbols, const bool symmetric)
{
	const int n = (int)symbols.size();
	std::vector<Scalar> vals;
	vals.push_back(expr);
	for (int i = 0; i < n; i++) {
		symx::Scalar g = symx::diff(expr, symbols[i]);
		vals.push_back(g);
	}
	if (symmetric) {
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < i; j++) {
				Scalar* matrix = &vals[1 + n];
				vals.push_back(matrix[j*n + i]);
			}
			for (int j = i; j < n; j++) {
				Scalar h = symx::diff(vals[1 + i], symbols[j]);
				vals.push_back(h);
			}
		}
	}
	else {
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				Scalar h = symx::diff(vals[1 + i], symbols[j]);
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
	const int n = (int)symbols.size();
	std::vector<Scalar> vals;
	vals.push_back(expr);
	for (int i = 0; i < n; i++) {
		symx::Scalar g = symx::diff(expr, symbols[i]);
		vals.push_back(g);
	}
	return vals;
}
symx::Scalar symx::diff(const Scalar& scalar, const Scalar& wrt)
{
	// Derivative rules: https://www.cs.utexas.edu/users/novak/asg-symdif.html#:~:text=A%20symbolic%20differentiation%20program%20finds,numeric%20calculations%20based%20on%20formulas.
	
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
			return scalar.make_branch(scalar.get_condition(), diff(scalar.left(), wrt), diff(scalar.right(), wrt));
		}
		else if (scalar.expr.type == ExprType::PowN) {
			Scalar u = scalar.left();
			Scalar du = diff(u, wrt);
			int32_t c = scalar.expr.b;
			return (double)(c) * u.powN(c - 1) * du;
		}
		else if (scalar.expr.type == ExprType::PowF) {
			std::cout << "symx error: ExprType::PowF not supported for diff." << std::endl;
			exit(-1);
			return scalar;
		}

		// Regular operations
		const Scalar u = scalar.left();
		const Scalar du = diff(u, wrt);
		const Scalar v = (scalar.has_not_right()) ? u : scalar.right();
		const Scalar dv = (scalar.has_not_right()) ? du : diff(v, wrt);

		switch (scalar.expr.type)
		{
		case ExprType::Add:
			return du + dv;
		case ExprType::Sub:
			return du - dv;
		case ExprType::Mul:
			return u*dv + v*du;
		case ExprType::Reciprocal:
			return -du/u.powN(2);
		case ExprType::Sqrt:
			return 0.5/sqrt(u)*du;
		case ExprType::Ln:
			return du/u;
		case ExprType::Exp:
			return exp(u) * du;
		case ExprType::Sin:
			return cos(u) * du;
		case ExprType::Cos:
			return -sin(u) * du;
		case ExprType::Tan:
			return (1.0 + tan(u).powN(2)) * du;
		case ExprType::ArcSin:
			return du/(sqrt(1.0 - u.powN(2)));
		case ExprType::ArcCos:
			return -du/(sqrt(1.0 - u.powN(2)));
		case ExprType::ArcTan:
			return du/(1.0 + u.powN(2));

		default:
			std::cout << "symx error: Unhandled ExprType found in diff." << std::endl;
			exit(-1);
			return scalar;
			break;
		}
	}
}
