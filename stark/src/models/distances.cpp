#include "distances.h"

symx::Scalar stark::models::distance_point_point(const symx::Vector& p, const symx::Vector& q)
{
	return (p - q).norm();
}
symx::Scalar stark::models::distance_point_line(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b)
{
	const symx::Vector ab = b - a;
	const symx::Vector ap = p - a;
	const symx::Scalar e = ap.dot(ab);
	const symx::Scalar ab_dot_ab = ab.dot(ab);
	return symx::sqrt(ap.dot(ap) - e*e/ab_dot_ab);
}
symx::Scalar stark::models::distance_point_plane(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b, const symx::Vector& c)
{
	const symx::Vector vpa = p - a;
	const symx::Vector vac = a - c;
	const symx::Vector vbc = b - c;
	const symx::Vector n = vac.cross3(vbc).normalized();
	const symx::Scalar d = vpa.dot(n);
	return symx::sqrt(d*d);
}
symx::Scalar stark::models::distance_line_line(const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q)
{
	symx::Vector n = (b - a).cross3(q - p);
	symx::Scalar l = (p - a).dot(n);
	symx::Scalar d2 = l.powN(2)/n.squared_norm();
	return symx::sqrt(d2);
}

