#pragma once
#include <symx>

namespace stark::models
{
	symx::Scalar sq_distance_point_point(const symx::Vector& p, const symx::Vector& q);
	symx::Scalar sq_distance_point_line(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b);
	symx::Scalar sq_distance_point_plane(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b, const symx::Vector& c);
	symx::Scalar sq_distance_line_line(const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q);
	symx::Scalar distance_point_point(const symx::Vector& p, const symx::Vector& q);
	symx::Scalar distance_point_line(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b);
	symx::Scalar distance_point_plane(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b, const symx::Vector& c);
	symx::Scalar distance_line_line(const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q);

	/*
		Distance between point p and line (a, b) when:
			- u_normalized = (b - a).normalized()
			- r = p - a
	*/
	symx::Scalar distance_sq_point_line(const symx::Vector& r, const symx::Vector& u_normalized);
}
