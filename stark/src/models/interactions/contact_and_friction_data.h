#pragma once
#include <vector>
#include <array>
#include <string>

#include <symx>

namespace stark
{
	/* ================================================================= */
	/* ===========================  GENERAL  =========================== */
	/* ================================================================= */
	enum class PhysicalSystem { Deformable, Rigidbody };

	struct DeformableBookkeeping
	{
		int collision_group = -1;
		int deformable_collision_group = -1;
		int deformable_idx = -1;

		// Maps the local collision mesh connectivity to the point set vertices (not the global vertices)
		// Needed for subsets of points such as triangle meshes around tet meshes
		std::vector<int> point_set_map; // vertices[i] = ps->get_global_vertex(idx_in_ps, point_set_map[i]);

		inline DeformableBookkeeping() {}
		inline DeformableBookkeeping(int collision_group, int deformable_collision_group, int deformable_idx, const std::vector<int>& point_set_map)
			: collision_group(collision_group), deformable_collision_group(deformable_collision_group), deformable_idx(deformable_idx), point_set_map(point_set_map) {}
	};

	struct RigidBodyBookkeeping
	{
		int collision_group = -1;
		int rb_collision_group = -1;
		int rb_idx = -1;
		inline RigidBodyBookkeeping() {}
		inline RigidBodyBookkeeping(int collision_group, int rb_collision_group, int rb_idx)
			: rb_collision_group(rb_collision_group), rb_idx(rb_idx) {}
	};

	struct ContactMesh
	{
		// Mandatory
		PhysicalSystem ps;
		int idx_in_ps = -1;  // Index local to the physical system
		std::vector<Eigen::Vector3d> vertices; // Used by the collision detection
		std::vector<std::array<int, 2>> loc_edges;  // Used by the collision detection
		
		// Optional: Not used for segment-only meshes
		std::vector<std::array<int, 3>> loc_triangles;  // Used by the collision detection
	};

	/* ============================================================================= */
	/* ===========================  COLLISION DETECTION  =========================== */
	/* ============================================================================= */
	template<std::size_t N>
	struct ProximityHelper
	{
		PhysicalSystem ps;
		int group = -1;  // Set index relative to all collision sets declared
		int idx_in_ps = -1;  // Index of the object relative to its physical system (eg. rigid body idx, deformable idx, ...)
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
			symx::LabelledConnectivity<4> point_point{ { "group_a", "group_b", "p", "q" } };
			symx::LabelledConnectivity<5> point_edge{ { "group_a", "group_b", "p", "e0", "e1" } };
			symx::LabelledConnectivity<6> point_triangle{ { "group_a", "group_b", "p", "t0", "t1", "t2" } };
		};
		struct EdgeEdge
		{
			symx::LabelledConnectivity<8> point_point{ { "group_a", "group_b", "ea0", "ea1", "p", "eb0", "eb1", "q" } };
			symx::LabelledConnectivity<7> point_edge{ { "group_a", "group_b", "ea0", "ea1", "p", "eb0", "eb1" } };
			symx::LabelledConnectivity<6> edge_edge{ { "group_a", "group_b", "ea0", "ea1", "eb0", "eb1" } };
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
			symx::LabelledConnectivity<6> point_point{ { "group_a", "group_b", "a", "b", "p", "q" } };
			symx::LabelledConnectivity<7> point_edge{ { "group_a", "group_b", "a", "b", "p", "e0", "e1" } };
			symx::LabelledConnectivity<8> point_triangle{ { "group_a", "group_b", "a", "b", "p", "t0", "t1", "t2" } };
		};
		struct EdgeEdge
		{
			symx::LabelledConnectivity<10> point_point{ { "group_a", "group_b", "a", "b", "ea0", "ea1", "p", "eb0", "eb1", "q" } };
			symx::LabelledConnectivity<9> point_edge{ { "group_a", "group_b", "a", "b", "ea0", "ea1", "p", "eb0", "eb1" } };
			symx::LabelledConnectivity<8> edge_edge{ { "group_a", "group_b", "a", "b", "ea0", "ea1", "eb0", "eb1" } };
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
			symx::LabelledConnectivity<5> rb_d_point_point{ { "group_a", "group_b", "rb", "p", "q" } };
			symx::LabelledConnectivity<6> rb_d_point_edge{ { "group_a", "group_b", "rb", "p", "e0", "e1" } };
			symx::LabelledConnectivity<7> rb_d_point_triangle{ { "group_a", "group_b", "rb", "p", "t0", "t1", "t2" } };
			symx::LabelledConnectivity<6> rb_d_edge_point{ { "group_a", "group_b", "rb", "e0", "e1", "p" } };
			symx::LabelledConnectivity<7> rb_d_triangle_point{ { "group_a", "group_b", "rb", "t0", "t1", "t2", "p" } };
		};
		struct EdgeEdge
		{
			symx::LabelledConnectivity<9> rb_d_point_point{ { "group_a", "group_b", "rb", "ea0", "ea1", "p", "eb0", "eb1", "q" } };
			symx::LabelledConnectivity<8> rb_d_point_edge{ { "group_a", "group_b", "rb", "ea0", "ea1", "p", "eb0", "eb1" } };
			symx::LabelledConnectivity<7> rb_d_edge_edge{ { "group_a", "group_b", "rb", "ea0", "ea1", "eb0", "eb1" } };
			symx::LabelledConnectivity<8> rb_d_edge_point{ { "group_a", "group_b", "rb", "ea0", "ea1", "eb0", "eb1", "q" } };
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
}