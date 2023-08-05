#pragma once
/*
    The code here is a slightly modified version of some functionalities found in the IPC-Toolkit:
    https://github.com/ipc-sim/ipc-toolkit/blob/main
*/

#include "Vec3.h"

namespace tmcd
{
    /// @brief Closest pair between two edges.
    enum class EdgeEdgeDistanceType {
        EA0_EB0, ///< The edges are closest at vertex 0 of edge A and 0 of edge B.
        EA0_EB1, ///< The edges are closest at vertex 0 of edge A and 1 of edge B.
        EA1_EB0, ///< The edges are closest at vertex 1 of edge A and 0 of edge B.
        EA1_EB1, ///< The edges are closest at vertex 1 of edge A and 1 of edge B.
        /// The edges are closest at the interior of edge A and vertex 0 of edge B.
        EA_EB0,
        /// The edges are closest at the interior of edge A and vertex 1 of edge B.
        EA_EB1,
        /// The edges are closest at vertex 0 of edge A and the interior of edge B.
        EA0_EB,
        /// The edges are closest at vertex 1 of edge A and the interior of edge B.
        EA1_EB,
        EA_EB ///< The edges are closest at an interior point of edge A and B.
    };
    double edge_edge_sq_distance(EdgeEdgeDistanceType& nearest_entity, const Vec3d& ea0, const Vec3d& ea1, const Vec3d& eb0, const Vec3d& eb1, const double parallel_cross_norm_sq_tolerance = 1e-20);
    EdgeEdgeDistanceType edge_edge_distance_type(const Vec3d& ea0, const Vec3d& ea1, const Vec3d& eb0, const Vec3d& eb1, const double parallel_cross_norm_sq_tolerance = 1e-20);
    EdgeEdgeDistanceType edge_edge_parallel_distance_type(const Vec3d& ea0, const Vec3d& ea1, const Vec3d& eb0, const Vec3d& eb1);

    /// @brief Closest pair between a point and triangle.
    enum class PointTriangleDistanceType {
        P_T0, ///< The point is closest to triangle vertex zero.
        P_T1, ///< The point is closest to triangle vertex one.
        P_T2, ///< The point is closest to triangle vertex two.
        P_E0, ///< The point is closest to triangle edge zero (vertex zero to one).
        P_E1, ///< The point is closest to triangle edge one (vertex one to two).
        P_E2, ///< The point is closest to triangle edge two (vertex two to zero).
        P_T,  ///< The point is closest to the interior of the triangle.
    };
    PointTriangleDistanceType point_triangle_distance_type(const Vec3d& p, const Vec3d& t0, const Vec3d& t1, const Vec3d& t2);
    double point_triangle_sq_distance(PointTriangleDistanceType& nearest_entity, const Vec3d& p, const Vec3d& t0, const Vec3d& t1, const Vec3d& t2);
    double point_triangle_sq_unsigned_jan_bender(PointTriangleDistanceType& nearest_entity, const Vec3d& point, const Vec3d& v0, const Vec3d& v1, const Vec3d& v2);
    bool is_edge_intersecting_triangle(const Vec3d& e0, const Vec3d& e1, const Vec3d& t0, const Vec3d& t1, const Vec3d& t2);
}

