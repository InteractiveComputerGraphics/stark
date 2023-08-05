#pragma once
#include <vector>

#include "types.h"


namespace tmcd
{
	struct TrianglePoint
	{
		Triangle triangle;
		Point point;
	};
	struct TriangleEdge
	{
		struct Edge // Remove the edge_idx as I don't have easyaccess to it in the point_triangle detection
		{
			int32_t set;
			std::array<int32_t, 2> vertices;
		};

		Triangle triangle;
		Edge edge;
	};
	struct EdgePoint
	{
		Edge edge;
		Point point;
	};

	struct ProximityResults
	{
		struct PointTriangle
		{
			std::vector<std::pair<Point, TrianglePoint>> point_point;
			std::vector<std::pair<Point, TriangleEdge>> point_edge;
			std::vector<std::pair<Point, Triangle>> point_triangle;
		};
		struct EdgeEdge
		{
			std::vector<std::pair<EdgePoint, EdgePoint>> point_point;
			std::vector<std::pair<EdgePoint, Edge>> point_edge;
			std::vector<std::pair<Edge, Edge>> edge_edge;
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
		};
	};
}