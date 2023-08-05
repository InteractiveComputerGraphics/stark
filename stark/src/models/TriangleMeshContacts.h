#pragma once
#include <vector>
#include <cstdint>


namespace stark::models
{
	struct TriangleMeshContacts
	{
		struct PointTriangle
		{
			std::vector<std::array<int32_t, 2>> point_point;  // [point_idx, point_idx]
			std::vector<std::array<int32_t, 3>> point_edge;  // [point_idx, edge_vertices]
			std::vector<std::array<int32_t, 4>> point_triangle; // [point_idx, triangle_vertices]
		};
		struct EdgeEdge
		{
			std::vector<std::array<int32_t, 6>> point_point;  // [edge_vertices, point_idx, edge_vertices, point_idx]
			std::vector<std::array<int32_t, 5>> point_edge;  // [edge_vertices, point_idx, edge_vertices]
			std::vector<std::array<int32_t, 4>> edge_edge;  // [edge_vertices, edge_vertices]
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