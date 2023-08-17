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
			std::vector<std::array<int32_t, 4>> point_point;    // { "a", "b", "a_p", "b_q" }
			std::vector<std::array<int32_t, 5>> point_edge;     // { "a", "b", "a_p", "b_e0", "b_e1" }
			std::vector<std::array<int32_t, 6>> point_triangle; // { "a", "b", "a_p", "b_t0", "b_t1", "b_t2" }
		};
		struct EdgeEdge
		{
			std::vector<std::array<int32_t, 8>> point_point;  // { "a", "b", "a_e0", "a_e1", "a_p", "b_e0", "b_e1", "b_q" }
			std::vector<std::array<int32_t, 7>> point_edge;  // { "a", "b", "a_e0", "a_e1", "a_p", "b_e0", "b_e1" }
			std::vector<std::array<int32_t, 6>> edge_edge;  // { "a", "b", "a_e0", "a_e1", "b_e0", "b_e1" }
		};
		PointTriangle point_triangle;
		EdgeEdge edge_edge;

		void clear()
		{
			this->point_triangle.point_point.clear();
			this->point_triangle.point_edge.clear();
			this->point_triangle.point_triangle.clear();

			this->edge_edge.point_point.clear();
			this->edge_edge.point_edge.clear();
			this->edge_edge.edge_edge.clear();
		}
	};
}