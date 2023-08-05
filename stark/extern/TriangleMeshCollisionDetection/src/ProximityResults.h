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
	template<typename T1, typename T2>
	struct ProximityPair
	{
		double distance;
		T1 first;
		T2 second;
	};
	struct ProximityResults
	{
		struct PointTriangle
		{
			std::vector<ProximityPair<Point, TrianglePoint>> point_point;
			std::vector<ProximityPair<Point, TriangleEdge>> point_edge;
			std::vector<ProximityPair<Point, Triangle>> point_triangle;
		};
		struct EdgeEdge
		{
			std::vector<ProximityPair<EdgePoint, EdgePoint>> point_point;
			std::vector<ProximityPair<EdgePoint, Edge>> point_edge;
			std::vector<ProximityPair<Edge, Edge>> edge_edge;
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