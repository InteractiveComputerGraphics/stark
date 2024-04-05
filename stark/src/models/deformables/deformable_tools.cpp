#include "deformable_tools.h"

#include "../../utils/include.h"

symx::Matrix stark::triangle_jacobian(const std::vector<symx::Vector>& x)
{
	// Projection matrix
	symx::Vector u = (x[1] - x[0]).normalized();
	symx::Vector n = u.cross3(x[2] - x[0]);
	symx::Vector v = u.cross3(n).normalized();
	symx::Matrix P = symx::Matrix(symx::gather({ u, v }), { 2, 3 });

	// Projection
	std::vector<symx::Vector> Xp = { P * x[0], P * x[1], P * x[2] };

	// Jacobian linear triangle
	symx::Matrix DX = symx::Matrix(symx::gather({ Xp[1] - Xp[0], Xp[2] - Xp[0] }), { 2, 2 }).transpose();
	return DX;
}
symx::Matrix stark::tet_jacobian(const std::vector<symx::Vector>& x)
{
	return symx::Matrix(symx::gather({ x[1] - x[0], x[2] - x[0] , x[3] - x[0] }), { 3, 3 }).transpose();
}
std::array<symx::Scalar, 2> stark::eigenvalues_sym_2x2(const symx::Matrix& A)
{
	// https://hal.science/hal-01501221/document
	const symx::Scalar& a = A(0, 0);
	const symx::Scalar& b = A(1, 1);
	const symx::Scalar& c = A(0, 1);

	const symx::Scalar delta = symx::sqrt(4.0 * c.powN(2) + (a - b).powN(2));
	return { 0.5 * (a + b + delta), 0.5 * (a + b - delta) };
}
std::array<symx::Scalar, 3> stark::eigenvalues_sym_3x3(const symx::Matrix& A)
{
	// https://hal.science/hal-01501221/document
	const symx::Scalar& a = A(0, 0);
	const symx::Scalar& b = A(1, 1);
	const symx::Scalar& c = A(2, 2);
	const symx::Scalar& d = A(1, 0);
	const symx::Scalar& e = A(2, 1);
	const symx::Scalar& f = A(2, 0);

	const symx::Scalar x1 = a.powN(2) + b.powN(2) + c.powN(2) - a * b - a * c - b * c + 3 * (d.powN(2) + e.powN(2) + f.powN(2));
	const symx::Scalar x2 = -(2.0 * a - b - c) * (2.0 * b - a - c) * (2.0 * c - a - b) +
		+9.0 * ((2.0 * c - a - b) * d.powN(2) + (2.0 * b - a - c) * f.powN(2) + (2.0 * a - b - c) * e.powN(2)) +
		-54.0 * d * e * f;
	const symx::Scalar sqrt_arg = 4.0 * x1.powN(3) - x2.powN(2);
	symx::Scalar phi = symx::atan(symx::sqrt(sqrt_arg) / x2);
	phi += symx::branch(x2 > 0.0, 0.0, M_PI); // Warning: Special case for atan(0/0) = 0.0 not handled
	return {
		1.0 / 3.0 * (a + b + c - 2.0 * sqrt(x1) * cos(phi / 3.0)),
		1.0 / 3.0 * (a + b + c + 2.0 * sqrt(x1) * cos((phi - M_PI) / 3.0)),
		1.0 / 3.0 * (a + b + c + 2.0 * sqrt(x1) * cos((phi + M_PI) / 3.0))
	};
}
