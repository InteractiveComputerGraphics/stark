#pragma once
#include <vector>
#include <array>
#include <string>

#include <symx>

namespace stark::models
{
	/* ================================================================= */
	/* ===========================  GENERAL  =========================== */
	/* ================================================================= */
	enum class PhysicalSystem { Deformable, Rigidbody };

	struct Mesh
	{
		PhysicalSystem ps;
		int ps_set = -1;  // Index local to the physical system
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 2>> edges;  // Local indices
		std::vector<std::array<int, 3>> triangles;  // Local indices
	};

	struct StaticPlanes
	{
		std::vector<Eigen::Vector3d> point;
		std::vector<Eigen::Vector3d> normal;
	};

	/* ============================================================================= */
	/* ===========================  COLLISION DETECTION  =========================== */
	/* ============================================================================= */
	template<std::size_t N>
	struct ProximityHelper
	{
		PhysicalSystem ps;
		int set = -1;  // Set index relative to all collision sets declared
		int ps_set = -1;  // Index of the object relative to its physical system (eg. rigid body idx, deformable idx, ...)
		std::array<int, N> local_verts ;  // Indices local to the object
		std::array<int, N> verts;  // Indices global to the physical system
		std::array<int, 2> edge;  // Edge corresponding to the vertices (only used in EdgePoint)
	};

	/* ================================================================= */
	/* ===========================  CONTACT  =========================== */
	/* ================================================================= */
	struct Contacts_Deformables
	{
		struct PointTriangle
		{
			symx::LabelledConnectivity<2> point_point{ { "p", "q" } };
			symx::LabelledConnectivity<3> point_edge{ { "p", "e0", "e1" } };
			symx::LabelledConnectivity<4> point_triangle{ { "p", "t0", "t1", "t2" } };
		};
		struct EdgeEdge
		{
			symx::LabelledConnectivity<6> point_point{ { "ea0", "ea1", "p", "eb0", "eb1", "q" } };
			symx::LabelledConnectivity<5> point_edge{ { "ea0", "ea1", "p", "eb0", "eb1" } };
			symx::LabelledConnectivity<4> edge_edge{ { "ea0", "ea1", "eb0", "eb1" } };
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
			symx::LabelledConnectivity<8> point_point{ { "a", "b", "ea0", "ea1", "p", "eb0", "eb1", "q" } };
			symx::LabelledConnectivity<7> point_edge{ { "a", "b", "ea0", "ea1", "p", "eb0", "eb1" } };
			symx::LabelledConnectivity<6> edge_edge{ { "a", "b", "ea0", "ea1", "eb0", "eb1" } };
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

	struct Contacts_RB_Deformables
	{
		struct PointTriangle
		{
			symx::LabelledConnectivity<3> rb_d_point_point{ { "rb", "p", "q" } };
			symx::LabelledConnectivity<4> rb_d_point_edge{ { "rb", "p", "e0", "e1" } };
			symx::LabelledConnectivity<5> rb_d_point_triangle{ { "rb", "p", "t0", "t1", "t2" } };
			symx::LabelledConnectivity<4> rb_d_edge_point{ { "rb", "e0", "e1", "p" } };
			symx::LabelledConnectivity<5> rb_d_triangle_point{ { "rb", "t0", "t1", "t2", "p" } };
		};
		struct EdgeEdge
		{
			symx::LabelledConnectivity<7> rb_d_point_point{ { "rb", "ea0", "ea1", "p", "eb0", "eb1", "q" } };
			symx::LabelledConnectivity<6> rb_d_point_edge{ { "rb", "ea0", "ea1", "p", "eb0", "eb1" } };
			symx::LabelledConnectivity<5> rb_d_edge_edge{ { "rb", "ea0", "ea1", "eb0", "eb1" } };
			symx::LabelledConnectivity<6> rb_d_edge_point{ { "rb", "ea0", "ea1", "eb0", "eb1", "q" } };
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

	struct Contacts_Static
	{
		symx::LabelledConnectivity<2> deformable_point{ { "plane", "a" } };
		symx::LabelledConnectivity<3> rb_point{ { "plane", "rb", "a" } };

		void clear()
		{
			this->deformable_point.clear();
			this->rb_point.clear();
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

	struct FrictionPointEdge
	{
		FrictionContact contact;
		std::vector<std::array<double, 2>> bary;
		void clear()
		{
			this->contact.clear();
			this->bary.clear();
		}
	};
	struct FrictionPointTriangle
	{
		FrictionContact contact;
		std::vector<std::array<double, 3>> bary;
		void clear()
		{
			this->contact.clear();
			this->bary.clear();
		}
	};
	struct FrictionEdgeEdge
	{
		FrictionContact contact;
		std::vector<std::array<double, 2>> bary;
		void clear()
		{
			this->contact.clear();
			this->bary.clear();
		}
	};

	struct Friction_Deformables
	{
		struct PointPoint
		{
			FrictionContact contact;
			symx::LabelledConnectivity<3> conn{ { "idx", "p", "q" } };
		};
		struct PointEdge
		{
			FrictionPointEdge data;
			symx::LabelledConnectivity<4> conn{ { "idx", "p", "e0", "e1" }};
		};
		struct PointTriangle
		{
			FrictionPointTriangle data;
			symx::LabelledConnectivity<5> conn{ { "idx", "p", "t0", "t1", "t2" } };
		};
		struct EdgeEdge
		{
			FrictionEdgeEdge data;
			symx::LabelledConnectivity<5> conn{ { "idx",  "ea0", "ea1", "eb0", "eb1" } };
		};

		PointPoint point_point;
		PointEdge point_edge;
		PointTriangle point_triangle;
		EdgeEdge edge_edge;

		void clear()
		{
			this->point_point.contact.clear();
			this->point_point.conn.clear();

			this->point_edge.data.clear();
			this->point_edge.conn.clear();

			this->point_triangle.data.clear();
			this->point_triangle.conn.clear();

			this->edge_edge.data.clear();
			this->edge_edge.conn.clear();
		}
	};

	struct Friction_RB
	{
		struct PointPoint
		{
			FrictionContact contact;
			symx::LabelledConnectivity<5> conn{ { "idx", "a", "b", "p", "q" } };
		};
		struct PointEdge
		{
			FrictionPointEdge data;
			symx::LabelledConnectivity<6> conn{ { "idx", "a", "b", "p", "e0", "e1" } };
		};
		struct PointTriangle
		{
			FrictionPointTriangle data;
			symx::LabelledConnectivity<7> conn{ { "idx", "a", "b", "p", "t0", "t1", "t2" } };
		};
		struct EdgeEdge
		{
			FrictionEdgeEdge data;
			symx::LabelledConnectivity<7> conn{ { "idx", "a", "b", "ea0", "ea1", "eb0", "eb1" } };
		};

		PointPoint point_point;
		PointEdge point_edge;
		PointTriangle point_triangle;
		EdgeEdge edge_edge;

		void clear()
		{
			this->point_point.contact.clear();
			this->point_point.conn.clear();

			this->point_edge.data.clear();
			this->point_edge.conn.clear();

			this->point_triangle.data.clear();
			this->point_triangle.conn.clear();

			this->edge_edge.data.clear();
			this->edge_edge.conn.clear();
		}
	};

	struct Friction_RB_Deformables
	{
		struct PointPoint
		{
			FrictionContact contact;
			symx::LabelledConnectivity<4> conn{ { "idx", "rb", "p", "q" } };
		};
		struct PointEdge
		{
			FrictionPointEdge data;
			symx::LabelledConnectivity<5> conn{ { "idx", "rb", "p", "e0", "e1"} };
		};
		struct PointTriangle
		{
			FrictionPointTriangle data;
			symx::LabelledConnectivity<6> conn{ { "idx", "rb", "p", "t0", "t1", "t2" } };
		};
		struct EdgeEdge
		{
			FrictionEdgeEdge data;
			symx::LabelledConnectivity<6> conn{ { "idx", "rb", "ea0", "ea1", "eb0", "eb1" } };
		};
		struct EdgePoint
		{
			FrictionPointEdge data;
			symx::LabelledConnectivity<5> conn{ { "idx", "rb", "e0", "e1", "p"} };
		};
		struct TrianglePoint
		{
			FrictionPointTriangle data;
			symx::LabelledConnectivity<6> conn{ { "idx", "rb", "t0", "t1", "t2", "p" } };
		};


		PointPoint point_point;
		PointEdge point_edge;
		PointTriangle point_triangle;
		EdgeEdge edge_edge;
		EdgePoint edge_point;
		TrianglePoint triangle_point;

		void clear()
		{
			this->point_point.contact.clear();
			this->point_point.conn.clear();

			this->point_edge.data.clear();
			this->point_edge.conn.clear();

			this->point_triangle.data.clear();
			this->point_triangle.conn.clear();

			this->edge_edge.data.clear();
			this->edge_edge.conn.clear();

			this->edge_point.data.clear();
			this->edge_point.conn.clear();

			this->triangle_point.data.clear();
			this->triangle_point.conn.clear();
		}
	};

	struct Friction_Static
	{
		struct DeformablePoint
		{
			FrictionContact contact;
			symx::LabelledConnectivity<2> conn{ { "idx", "a" } };
		};
		struct RBPoint
		{
			FrictionContact contact;
			symx::LabelledConnectivity<3> conn{ { "idx", "rb", "a" } };
		};

		DeformablePoint deformable_point;
		RBPoint rb_point;

		void clear()
		{
			this->deformable_point.contact.clear();
			this->deformable_point.conn.clear();

			this->rb_point.contact.clear();
			this->rb_point.conn.clear();
		}
	};

}