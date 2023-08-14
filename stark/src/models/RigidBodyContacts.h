#pragma once
#include <vector>
#include <cstdint>

#include <Eigen/Dense>


namespace stark::models
{
	struct RigidBodyContacts
	{
		struct PointTriangle
		{
			std::vector<std::array<int, 4>> point_point;    // { "a", "b", "a_p", "b_q" }
			std::vector<std::array<int, 5>> point_edge;     // { "a", "b", "a_p", "b_e0", "b_e1" }
			std::vector<std::array<int, 6>> point_triangle; // { "a", "b", "a_p", "b_t0", "b_t1", "b_t2" }
		};

		PointTriangle point_triangle;

		void clear()
		{
			this->point_triangle.point_point.clear();
			this->point_triangle.point_edge.clear();
			this->point_triangle.point_triangle.clear();
		}
	};
}