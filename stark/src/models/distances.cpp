#include "distances.h"

double stark::sq_distance_point_point(const Eigen::Vector3d& p, const Eigen::Vector3d& q)
{
	return (p - q).squaredNorm();
}
double stark::sq_distance_point_line(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
	const Eigen::Vector3d ab = b - a;
	const Eigen::Vector3d ap = p - a;
	const double e = ap.dot(ab);
	const double ab_dot_ab = ab.dot(ab);
	return ap.dot(ap) - e*e/ab_dot_ab;
}
double stark::sq_distance_point_plane(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
	const Eigen::Vector3d vpa = p - a;
	const Eigen::Vector3d vac = a - c;
	const Eigen::Vector3d vbc = b - c;
	const Eigen::Vector3d n = vac.cross(vbc).normalized();
	const double d = vpa.dot(n);
	return d*d;
}
double stark::sq_distance_point_plane(const Eigen::Vector3d& a, const Eigen::Vector3d& plane_point, const Eigen::Vector3d& plane_normal)
{
	const double d = (a - plane_point).dot(plane_normal);
	return d * d;
}
double stark::sq_distance_line_line(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q)
{
	Eigen::Vector3d n = (b - a).cross(q - p);
	double l = (p - a).dot(n);
	double d2 = std::pow(l, 2)/n.squaredNorm();
	return d2;
}
double stark::distance_point_point(const Eigen::Vector3d& p, const Eigen::Vector3d& q)
{
	return std::sqrt(sq_distance_point_point(p, q));
}
double stark::distance_point_line(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
	return std::sqrt(sq_distance_point_line(p, a, b));
}
double stark::distance_point_plane(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
	return std::sqrt(sq_distance_point_plane(p, a, b, c));
}
double stark::distance_point_plane(const Eigen::Vector3d& a, const Eigen::Vector3d& plane_point, const Eigen::Vector3d& plane_normal)
{
	return std::sqrt(sq_distance_point_plane(a, plane_point, plane_normal));
}
double stark::distance_line_line(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p, const Eigen::Vector3d& q)
{
	return std::sqrt(sq_distance_line_line(a, b, p, q));
}

symx::Scalar stark::sq_distance_point_point(const symx::Vector& p, const symx::Vector& q)
{
	return (p - q).squared_norm();
}
symx::Scalar stark::sq_distance_point_line(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b)
{
	const symx::Vector ab = b - a;
	const symx::Vector ap = p - a;
	const symx::Scalar e = ap.dot(ab);
	const symx::Scalar ab_dot_ab = ab.dot(ab);
	return ap.dot(ap) - e*e/ab_dot_ab;
}
symx::Scalar stark::sq_distance_point_plane(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b, const symx::Vector& c)
{
	const symx::Vector vpa = p - a;
	const symx::Vector vac = a - c;
	const symx::Vector vbc = b - c;
	const symx::Vector n = vac.cross3(vbc).normalized();
	const symx::Scalar d = vpa.dot(n);
	return d*d;
}
symx::Scalar stark::sq_distance_point_plane(const symx::Vector& a, const symx::Vector& plane_point, const symx::Vector& plane_normal)
{
	const symx::Scalar d = (a - plane_point).dot(plane_normal);
	return d * d;
}
symx::Scalar stark::sq_distance_line_line(const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q)
{
	symx::Vector n = (b - a).cross3(q - p);
	symx::Scalar l = (p - a).dot(n);
	symx::Scalar d2 = l.powN(2)/n.squared_norm();
	return d2;
}
symx::Scalar stark::distance_point_point(const symx::Vector& p, const symx::Vector& q)
{
	return symx::sqrt(sq_distance_point_point(p, q));
}
symx::Scalar stark::distance_point_line(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b)
{
	return symx::sqrt(sq_distance_point_line(p, a, b));
}
symx::Scalar stark::distance_point_plane(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b, const symx::Vector& c)
{
	return symx::sqrt(sq_distance_point_plane(p, a, b, c));
}
symx::Scalar stark::distance_point_plane(const symx::Vector& a, const symx::Vector& plane_point, const symx::Vector& plane_normal)
{
	return symx::sqrt(sq_distance_point_plane(a, plane_point, plane_normal));
}
symx::Scalar stark::distance_line_line(const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q)
{
	return symx::sqrt(sq_distance_line_line(a, b, p, q));
}

double stark::signed_distance_point_plane(const Eigen::Vector3d& a, const Eigen::Vector3d& plane_point, const Eigen::Vector3d& plane_normal)
{
	return (a - plane_point).dot(plane_normal);
}
