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
		struct EdgeEdge
		{
			symx::LabelledConnectivity<7> rb_d_point_point{ { "rb", "rb_e0", "rb_e1", "rb_p", "e0", "e1", "q" } };
			symx::LabelledConnectivity<6> rb_d_point_edge{ { "rb", "rb_e0", "rb_e1", "rb_p", "e0", "e1" } };
			symx::LabelledConnectivity<5> rb_d_edge_edge{ { "rb", "rb_e0", "rb_e1", "e0", "e1" } };
			symx::LabelledConnectivity<6> rb_d_edge_point{ { "rb", "rb_e0", "rb_e1", "e0", "e1", "q" } };
		};
		PointTriangle point_triangle;
		EdgeEdge edge_edge;

		void clear()
		{
			this->point_triangle.rb_d_point_point.clear();
			this->point_triangle.rb_d_point_edge.clear();
			this->point_triangle.rb_d_point_triangle.clear();
			this->point_triangle.rb_d_edge_point.clear();
			this->point_triangle.rb_d_triangle_point.clear();

			this->edge_edge.rb_d_point_point.clear();
			this->edge_edge.rb_d_point_edge.clear();
			this->edge_edge.rb_d_edge_edge.clear();
			this->edge_edge.rb_d_edge_point.clear();
		}
	};
}
