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
			struct PointPointRBC
			{
				std::vector<Eigen::Vector3d> loc_a_p;
				std::vector<Eigen::Vector3d> loc_b_q;
				std::vector<std::array<int, 3>> conn; // { "idx", "a", "b" }
			};
			struct PointEdgeRBC
			{
				std::vector<Eigen::Vector3d> loc_a_p;
				std::vector<Eigen::Vector3d> loc_b_e0;
				std::vector<Eigen::Vector3d> loc_b_e1;
				std::vector<std::array<int, 3>> conn; // { "idx", "a", "b" }
			};
			struct PointTriangleRBC
			{
				std::vector<Eigen::Vector3d> loc_a_p;
				std::vector<Eigen::Vector3d> loc_b_t0;
				std::vector<Eigen::Vector3d> loc_b_t1;
				std::vector<Eigen::Vector3d> loc_b_t2;
				std::vector<std::array<int, 3>> conn; // { "idx", "a", "b" }
			};

			PointPointRBC point_point;
			PointEdgeRBC point_edge;
			PointTriangleRBC point_triangle;
		};

		PointTriangle point_triangle;

		void clear()
		{
			this->point_triangle.point_point.loc_a_p.clear();
			this->point_triangle.point_point.loc_b_q.clear();
			this->point_triangle.point_point.conn.clear();

			this->point_triangle.point_edge.loc_a_p.clear();
			this->point_triangle.point_edge.loc_b_e0.clear();
			this->point_triangle.point_edge.loc_b_e1.clear();
			this->point_triangle.point_edge.conn.clear();

			this->point_triangle.point_triangle.loc_a_p.clear();
			this->point_triangle.point_triangle.loc_b_t0.clear();
			this->point_triangle.point_triangle.loc_b_t1.clear();
			this->point_triangle.point_triangle.loc_b_t2.clear();
			this->point_triangle.point_triangle.conn.clear();
		}
	};
}