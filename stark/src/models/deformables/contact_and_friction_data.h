#pragma once
#include <vector>
#include <array>
#include <string>

#include <symx>

namespace stark::models
{
	/* ================================================================= */
	/* ===========================  CONTACT  =========================== */
	/* ================================================================= */
	struct Contacts_Deformable
	{
		struct PointTriangle
		{
			symx::LabelledConnectivity<2> point_point{ { "p", "q" } };
			symx::LabelledConnectivity<3> point_edge{ { "p", "e0", "e1" } };
			symx::LabelledConnectivity<4> point_triangle{ { "p", "t0", "t1", "t2" } };
		};
		struct EdgeEdge
		{
			symx::LabelledConnectivity<6> point_point{ { "a_e0", "a_e1", "p", "b_e0", "b_e1", "q" } };
			symx::LabelledConnectivity<5> point_edge{ { "a_e0", "a_e1", "p", "b_e0", "b_e1" } };
			symx::LabelledConnectivity<4> edge_edge{ { "a_e0", "a_e1", "b_e0", "b_e1" } };
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

	struct Contacts_RB
	{
		struct PointTriangle
		{
			symx::LabelledConnectivity<4> point_point{ { "a", "b", "p", "q" } };
			symx::LabelledConnectivity<5> point_edge{ { "a", "b", "p", "e0", "e1" } };
			symx::LabelledConnectivity<6> point_triangle{ { "a", "b", "p", "t0", "t1", "t2" } };
		};
		struct EdgeEdge
		{
			symx::LabelledConnectivity<8> point_point{ { "a", "b", "a_e0", "a_e1", "p", "b_e0", "b_e1", "q" } };
			symx::LabelledConnectivity<7> point_edge{ { "a", "b", "a_e0", "a_e1", "p", "b_e0", "b_e1" } };
			symx::LabelledConnectivity<6> edge_edge{ { "a", "b", "a_e0", "a_e1", "b_e0", "b_e1" } };
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

	struct Contacts_RB_Deformable
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


	/* ================================================================== */
	/* ===========================  FRICTION  =========================== */
	/* ================================================================== */
	struct FrictionContact
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

	struct Friction_Deformable
	{
		struct PointPoint
		{
			FrictionContact contact;
			symx::LabelledConnectivity<3> conn{ { "idx", "p", "q" } };
		};
		struct PointEdge
		{
			FrictionContact contact;
			symx::LabelledConnectivity<4> conn{ { "idx", "p", "e0" } };
			std::vector<std::array<double, 2>> bary;
		};
		struct PointTriangle
		{
			FrictionContact contact;
			symx::LabelledConnectivity<5> conn{ { "idx", "p", "t0", "t1", "t2" } };
			std::vector<std::array<double, 3>> bary;
		};
		struct EdgeEdge
		{
			FrictionContact contact;
			symx::LabelledConnectivity<5> conn{ { "idx",  "a_e0", "a_e1", "b_e0", "b_e1" } };
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

	struct Friction_RB
	{
		struct PointPoint
		{
			FrictionContact contact;
			std::vector<std::array<int32_t, 5>> conn;  // { "idx", "a", "b", "a_p", "b_q" }
			symx::LabelledConnectivity<5> conn{ { "idx", "p", "q" } };
		};
		struct PointEdge
		{
			FrictionContact contact;
			symx::LabelledConnectivity<6> conn{ { "idx", "a", "b", "a_p", "b_e0", "b_e1" } };
			std::vector<std::array<double, 2>> bary;
		};
		struct PointTriangle
		{
			FrictionContact contact;
			symx::LabelledConnectivity<7> conn{ { "idx", "a", "b", "a_p", "b_t0", "b_t1", "b_t2" } };
			std::vector<std::array<double, 3>> bary;
		};
		struct EdgeEdge
		{
			FrictionContact contact;
			symx::LabelledConnectivity<7> conn{ { "idx", "a", "b", "a_e0", "a_e1", "b_e0", "b_e1" } };
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

	struct Friction_RB_Deformable
	{
		struct PointPoint
		{
			FrictionContact contact;
			symx::LabelledConnectivity<4> conn{ { "idx", "rb", "rb_p", "q" } };
		};
		struct PointEdge
		{
			FrictionContact contact;
			symx::LabelledConnectivity<5> conn{ { "idx", "rb", "p", "e0", "e1"} };
			std::vector<std::array<double, 2>> bary;
		};
		struct PointTriangle
		{
			FrictionContact contact;
			symx::LabelledConnectivity<6> conn{ { "idx", "rb", "p", "t0", "t1", "t2" } };
			std::vector<std::array<double, 3>> bary;
		};
		struct EdgeEdge
		{
			FrictionContact contact;
			symx::LabelledConnectivity<6> conn{ { "idx", "rb", "rb_e0", "rb_e1", "e0", "e1" } };
			std::vector<std::array<double, 2>> bary;
		};
		struct EdgePoint
		{
			FrictionContact contact;
			symx::LabelledConnectivity<5> conn{ { "idx", "rb", "e0", "e1", "p"} };
			std::vector<std::array<double, 2>> bary;
		};
		struct TrianglePoint
		{
			FrictionContact contact;
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