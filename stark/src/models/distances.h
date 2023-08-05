#pragma once
#include <symx>

namespace stark::models
{
	symx::Scalar distance_point_point(const symx::Vector& p, const symx::Vector& q);
	symx::Scalar distance_point_line(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b);
	symx::Scalar distance_point_plane(const symx::Vector& p, const symx::Vector& a, const symx::Vector& b, const symx::Vector& c);
	symx::Scalar distance_line_line(const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q);
}
