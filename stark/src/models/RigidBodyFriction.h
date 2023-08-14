#pragma once
#include <vector>
#include <cstdint>


namespace stark::models
{
	struct RigidBodyFriction
	{
		struct Contact
		{
			std::vector<std::array<double, 6>> T;
			std::vector<double> mu;
			std::vector<double> fn;
			void clear()
			{
				this->T.clear();
				this->mu.clear();
				this->fn.clear();
			}
		};
		struct PointPoint
		{
			Contact contact;
			std::vector<std::array<int32_t, 5>> conn;  // { "idx", "a", "b", "a_p", "b_q" }
		};
		struct PointEdge
		{
			Contact contact;
			std::vector<std::array<int32_t, 6>> conn;  // { "idx", "a", "b", "a_p", "b_e0", "b_e1" }
			std::vector<std::array<double, 2>> bary;
		};
		struct PointTriangle
		{
			Contact contact;
			std::vector<std::array<int32_t, 7>> conn;  // { "idx", "a", "b", "a_p", "b_t0", "b_t1", "b_t2" }
			std::vector<std::array<double, 3>> bary;
		};
		//struct EdgeEdge
		//{
		//	Contact contact;
		//	std::vector<std::array<int32_t, 5>> conn;  // { "idx", "a", "b", "a_p", "b_q" }
		//	std::vector<std::array<double, 2>> bary;
		//};

		PointPoint point_point;
		PointEdge point_edge;
		PointTriangle point_triangle;
		//EdgeEdge edge_edge;

		void clear()
		{
			this->point_point.contact.clear();
			this->point_point.conn.clear();

			this->point_edge.contact.clear();
			this->point_edge.conn.clear();
			this->point_edge.bary.clear();

			this->point_triangle.contact.clear();
			this->point_triangle.conn.clear();
			this->point_triangle.bary.clear();

			//this->edge_edge.contact.clear();
			//this->edge_edge.conn.clear();
			//this->edge_edge.bary.clear();
		}
	};
}