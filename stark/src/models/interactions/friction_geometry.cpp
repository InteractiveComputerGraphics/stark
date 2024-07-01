#include "friction_geometry.h"

// Ericson05
std::array<double, 3> stark::barycentric_point_triangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
	const Eigen::Vector3d v0 = b - a;
	const Eigen::Vector3d v1 = c - a;
	const Eigen::Vector3d v2 = p - a;
	const double d00 = v0.dot(v0);
	const double d01 = v0.dot(v1);
	const double d11 = v1.dot(v1);
	const double d20 = v2.dot(v0);
	const double d21 = v2.dot(v1);
	const double denom_inv = 1.0 / (d00 * d11 - d01 * d01);
	const double v = (d11 * d20 - d01 * d21) * denom_inv;
	const double w = (d00 * d21 - d01 * d20) * denom_inv;
	const double u = 1.0 - v - w;
	return { u, v, w };
}
// Ericson05
std::array<double, 2> stark::barycentric_point_edge(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
	const Eigen::Vector3d ab = b - a;
	const Eigen::Vector3d u = p - a;
	const double alpha = u.dot(ab) / ab.squaredNorm();
	return { 1.0 - alpha, alpha };
}
// Ericson05
std::array<double, 2> stark::barycentric_edge_edge(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& P, const Eigen::Vector3d& Q)
{
	const Eigen::Vector3d da = B - A; // Direction vector of segment S0
	const Eigen::Vector3d db = Q - P; // Direction vector of segment S1
	const Eigen::Vector3d r = A - P;
	const double a = da.dot(da); // Squared length of segment S1, always nonnegative
	const double e = db.dot(db); // Squared length of segment S2, always nonnegative
	const double f = db.dot(r);
	const double b = da.dot(db);
	const double c = da.dot(r);
	const double denom = a * e - b * b; // Always nonnegative
	const double cross_sq_norm = denom; // Same as da.cross(db).squaredNorm();

	if (denom < 1e-16) {
		std::cout << "error stark::barycentric_edge_edge(): parallel edge found." << std::endl;
		exit(-1);
	}
	const double s = (b * f - c * e) / denom;  // arc of the closest point on L1 to L2
	const double t = (b * s + f) / e;  // arc of the closest point on L2 to L1
	return {s, t};
}

std::array<double, 6> stark::projection_matrix_triangle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
	const Eigen::Vector3d v01 = a - c;
	const Eigen::Vector3d v02 = b - c;

	const Eigen::Vector3d u = v01.normalized();
	const Eigen::Vector3d normal = v01.cross(v02);
	const Eigen::Vector3d v = normal.cross(u).normalized();

	Eigen::Matrix<double, 2, 3> P;
	P.row(0) = u;
	P.row(1) = v;
	return { P(0, 0), P(0, 1), P(0, 2), P(1, 0), P(1, 1), P(1, 2) };
}
std::array<double, 6> stark::projection_matrix_edge_edge(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q)
{
	const Eigen::Vector3d u = (b - a).normalized();
	const Eigen::Vector3d n = u.cross(q - p);
	const Eigen::Vector3d v = u.cross(n).normalized();

	Eigen::Matrix<double, 2, 3> P;
	P.row(0) = u;
	P.row(1) = v;
	return { P(0, 0), P(0, 1), P(0, 2), P(1, 0), P(1, 1), P(1, 2) };
}
std::array<double, 6> stark::projection_matrix_point_point(const Eigen::Vector3d& p, const Eigen::Vector3d& a)
{
	const Eigen::Vector3d n = (p - a).normalized();
	Eigen::Vector3d e;
	if (n.z() < 0.99) {
		e = Eigen::Vector3d::UnitZ();
	}
	else {
		e = Eigen::Vector3d::UnitX();
	}
	const Eigen::Vector3d u = e.cross(n).normalized();
	const Eigen::Vector3d v = u.cross(n).normalized();
	Eigen::Matrix<double, 2, 3> P;
	P.row(0) = u;
	P.row(1) = v;
	return { P(0, 0), P(0, 1), P(0, 2), P(1, 0), P(1, 1), P(1, 2) };
}
std::array<double, 6> stark::projection_matrix_point_edge(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
	const Eigen::Vector3d u = (b - a).normalized();
	const Eigen::Vector3d t = (p - a);
	const Eigen::Vector3d v = u.cross(t).normalized();

	Eigen::Matrix<double, 2, 3> P;
	P.row(0) = u;
	P.row(1) = v;
	return { P(0, 0), P(0, 1), P(0, 2), P(1, 0), P(1, 1), P(1, 2) };
}

double stark::edge_edge_mollifier(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q, const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& P, const Eigen::Vector3d& Q)
{
	const double eps_x = 1e-3 * (A - B).squaredNorm() * (P - Q).squaredNorm();
	const double x = (b - a).cross(q - p).squaredNorm();
	const double x_div_eps_x = x / eps_x;
	const double f = (-x_div_eps_x + 2.0) * x_div_eps_x;
	if (x > eps_x) {
		return 1.0;
	}
	else {
		return f;
	}
}