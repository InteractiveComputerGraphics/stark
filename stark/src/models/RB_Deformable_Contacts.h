#pragma once
#include <vector>
#include <array>
#include <string>
#include <cstdint>

#include <symx>

namespace stark::models
{
	struct RB_Deformable_Contacts
	{
		struct PointTriangle
		{
			symx::LabelledConnectivity<3> rb_d_point_point{ { "rb", "rb_p_loc", "q" } };
			symx::LabelledConnectivity<4> rb_d_point_edge{ { "rb", "rb_p_loc", "e0", "e1" } };
			symx::LabelledConnectivity<5> rb_d_point_triangle{ { "rb", "rb_p_loc", "t0", "t1", "t2" } };
			symx::LabelledConnectivity<4> rb_d_edge_point{ { "rb", "rb_e0_loc", "rb_e1_loc", "p" } };
			symx::LabelledConnectivity<5> rb_d_triangle_point{ { "rb", "rb_t0_loc", "rb_t1_loc", "rb_t2_loc", "p" } };
		};
		//struct EdgeEdge
		//{
		//	std::vector<std::array<int32_t, 6>> point_point;  // [edge_vertices, point_idx, edge_vertices, point_idx]
		//	std::vector<std::array<int32_t, 5>> point_edge;  // [edge_vertices, point_idx, edge_vertices]
		//	std::vector<std::array<int32_t, 4>> edge_edge;  // [edge_vertices, edge_vertices]
		//};
		PointTriangle point_triangle;
		//EdgeEdge edge_edge;

		void clear()
		{
			this->point_triangle.rb_d_point_point.clear();
			this->point_triangle.rb_d_point_edge.clear();
			this->point_triangle.rb_d_point_triangle.clear();
			this->point_triangle.rb_d_edge_point.clear();
			this->point_triangle.rb_d_triangle_point.clear();

			//this->edge_edge.point_point.clear();
			//this->edge_edge.point_edge.clear();
			//this->edge_edge.edge_edge.clear();
		}
	};
}