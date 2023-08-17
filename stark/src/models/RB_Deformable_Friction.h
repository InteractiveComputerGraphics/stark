#pragma once
#include <vector>
#include <array>
#include <string>
#include <cstdint>

#include <symx>


namespace stark::models
{
	struct RB_Deformable_Friction
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
			symx::LabelledConnectivity<4> conn{ { "idx", "rb", "rb_p", "q" } };
		};
		struct PointEdge
		{
			Contact contact;
			symx::LabelledConnectivity<5> conn{ { "idx", "rb", "p", "e0", "e1"}};
			std::vector<std::array<double, 2>> bary;
		};
		struct PointTriangle
		{
			Contact contact;
			symx::LabelledConnectivity<6> conn{ { "idx", "rb", "p", "t0", "t1", "t2" } };
			std::vector<std::array<double, 3>> bary;
		};
		struct EdgeEdge
		{
			Contact contact;
			symx::LabelledConnectivity<6> conn{ { "idx", "rb", "rb_e0", "rb_e1", "e0", "e1" } };
			std::vector<std::array<double, 2>> bary;
		};
		struct EdgePoint
		{
			Contact contact;
			symx::LabelledConnectivity<5> conn{ { "idx", "rb", "e0", "e1", "p"} };
			std::vector<std::array<double, 2>> bary;
		};
		struct TrianglePoint
		{
			Contact contact;
			symx::LabelledConnectivity<6> conn{ { "idx", "rb", "t0", "t1", "t2", "p" } };
			std::vector<std::array<double, 3>> bary;
		};


		PointPoint rb_d_point_point;
		PointEdge rb_d_point_edge;
		PointTriangle rb_d_point_triangle;
		EdgeEdge rb_d_edge_edge;
		EdgePoint rb_d_edge_point;
		TrianglePoint rb_d_triangle_point;

		void clear()
		{
			this->rb_d_point_point.contact.clear();
			this->rb_d_point_point.conn.clear();
				  
			this->rb_d_point_edge.contact.clear();
			this->rb_d_point_edge.conn.clear();
			this->rb_d_point_edge.bary.clear();
				  
			this->rb_d_point_triangle.contact.clear();
			this->rb_d_point_triangle.conn.clear();
			this->rb_d_point_triangle.bary.clear();
				  
			this->rb_d_edge_edge.contact.clear();
			this->rb_d_edge_edge.conn.clear();
			this->rb_d_edge_edge.bary.clear();
				  
			this->rb_d_edge_point.contact.clear();
			this->rb_d_edge_point.conn.clear();
			this->rb_d_edge_point.bary.clear();
				  
			this->rb_d_triangle_point.contact.clear();
			this->rb_d_triangle_point.conn.clear();
			this->rb_d_triangle_point.bary.clear();
		}
	};
}