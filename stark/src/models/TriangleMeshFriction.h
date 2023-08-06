#pragma once
#include <vector>
#include <cstdint>


namespace stark::models
{
	struct TriangleMeshFriction
	{
		struct Contact
		{
			std::vector<std::array<double, 6>> T;
			std::vector<double> mu;
			std::vector<double> fn;
			void clear()
			{
				this->T.clear();
				this->fn.clear();
				this->mu.clear();
			}
		};
		struct PointPoint
		{
			Contact contact;
			std::vector<std::array<int32_t, 3>> conn;  // [contact_idx, point_idx, other_point_idx]
		};
		struct PointEdge
		{
			Contact contact;
			std::vector<std::array<int32_t, 4>> conn;  // [contact_idx, point_idx, edge_vertices]
			std::vector<std::array<double, 2>> bary;
		};
		struct PointTriangle
		{
			Contact contact;
			std::vector<std::array<int32_t, 5>> conn;  // [contact_idx, point_idx, triangle_vertices]
			std::vector<std::array<double, 3>> bary;
		};
		struct EdgeEdge
		{
			Contact contact;
			std::vector<std::array<int32_t, 5>> conn;  // [contact_idx, edge_vertices, other_edge_vertices]
			std::vector<std::array<double, 2>> bary;
		};

		PointPoint point_point;
		PointEdge point_edge;
		PointTriangle point_triangle;
		EdgeEdge edge_edge;

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

			this->edge_edge.contact.clear();
			this->edge_edge.conn.clear();
			this->edge_edge.bary.clear();
		}
	};
}