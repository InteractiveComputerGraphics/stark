#include "ipc_toolkit_geometry_functions.h"
/*
    The code here is a slightly modified version of some functionalities found in the IPC-Toolkit:
    https://github.com/ipc-sim/ipc-toolkit/blob/main
*/

#include <iostream>
#include <cassert>
#include <array>
#include <cstdint>
#include <limits>

double point_point_sq_distance(const tmcd::Vec3d& p0, const tmcd::Vec3d& p1)
{
    return (p1 - p0).squaredNorm();
}
double point_line_sq_distance(const tmcd::Vec3d& p, const tmcd::Vec3d& e0, const tmcd::Vec3d& e1)
{
    return (e0 - p).cross((e1 - p)).squaredNorm()/(e1 - e0).squaredNorm();
}
double line_line_sq_distance(const tmcd::Vec3d& ea0, const tmcd::Vec3d& ea1, const tmcd::Vec3d& eb0, const tmcd::Vec3d& eb1)
{
    const tmcd::Vec3d normal = (ea1 - ea0).cross(eb1 - eb0);
    const double line_to_line = (eb0 - ea0).dot(normal);
    return line_to_line * line_to_line / normal.squaredNorm();
}
double point_plane_sq_distance(const tmcd::Vec3d& p, const tmcd::Vec3d& origin, const tmcd::Vec3d& normal)
{
    const double point_to_plane = (p - origin).dot(normal);
    return point_to_plane * point_to_plane / normal.squaredNorm();
}
double point_plane_sq_distance(const tmcd::Vec3d& p, const tmcd::Vec3d& t0, const tmcd::Vec3d& t1, const tmcd::Vec3d& t2)
{
    return point_plane_sq_distance(p, t0, (t1 - t0).cross(t2 - t0));
}
double tmcd::edge_edge_sq_distance(EdgeEdgeDistanceType& nearest_entity, const Vec3d& ea0, const Vec3d& ea1, const Vec3d& eb0, const Vec3d& eb1, const double parallel_cross_norm_threshold)
{
	const Vec3d da = ea1 - ea0; // Direction vector of segment S0
	const Vec3d db = eb1 - eb0; // Direction vector of segment S1
	const Vec3d r = ea0 - eb0;
	const double a = da.dot(da); // Squared length of segment S1, always nonnegative
	const double e = db.dot(db); // Squared length of segment S2, always nonnegative
	const double f = db.dot(r);
	const double b = da.dot(db);
	const double c = da.dot(r);
	const double denom = a * e - b * b; // Always nonnegative
	const double s = (b * f - c * e) / denom;  // arc of the closest point on L1 to L2
	const double t = (b * s + f) / e;  // arc of the closest point on L2 to L1

	// Parallel cases (this particular check is relevant for IPC type of contact)
	const double cross_sq_norm = da.cross(db).squaredNorm();
	if (cross_sq_norm < parallel_cross_norm_threshold*parallel_cross_norm_threshold) {

		// Check whether the edges overlap or are in front or behind each other
		// TODO: Avoid sqrt ?
		const double la = da.norm();
		const Vec3d da_ = da/la;
		const double b0_ = (eb0 - ea0).dot(da_)/la;
		const double b1_ = (eb1 - ea0).dot(da_)/la;

		if (b0_ <= 0.0 && b1_ <= 0.0) { // Parallel but eb fully behind a0
			if (b0_ < b1_) {
				nearest_entity = EdgeEdgeDistanceType::EA0_EB1;
				return point_point_sq_distance(ea0, eb1);
			}
			else {
				nearest_entity = EdgeEdgeDistanceType::EA0_EB0;
				return point_point_sq_distance(ea0, eb0);
			}
		}
		else if (b0_ >= 1.0 && b1_ >= 1.0) {  // Parallel but eb fully after a1
			if (b0_ < b1_) {
				nearest_entity = EdgeEdgeDistanceType::EA1_EB0;
				return point_point_sq_distance(ea1, eb0);
			}
			else {
				nearest_entity = EdgeEdgeDistanceType::EA1_EB1;
				return point_point_sq_distance(ea1, eb1);
			}
		}
		else {  // Parallel with an overlap
			nearest_entity = EdgeEdgeDistanceType::Parallel;
			return point_line_sq_distance(ea0, eb0, eb1);
		}
	}

	// Non parallel cases
	if (s <= 0.0) {
		if (t <= 0.0) {
			nearest_entity = EdgeEdgeDistanceType::EA0_EB0;
			return point_point_sq_distance(ea0, eb0);
		}
		else if (t >= 1.0) {
			nearest_entity = EdgeEdgeDistanceType::EA0_EB1;
			return point_point_sq_distance(ea0, eb1);
		}
		else {
			nearest_entity = EdgeEdgeDistanceType::EA0_EB;
			return point_line_sq_distance(ea0, eb0, eb1);
		}
	}
	else if (s >= 1.0) {
		if (t <= 0.0) {
			nearest_entity = EdgeEdgeDistanceType::EA1_EB0;
			return point_point_sq_distance(ea1, eb0);
		}
		else if (t >= 1.0) {
			nearest_entity = EdgeEdgeDistanceType::EA1_EB1;
			return point_point_sq_distance(ea1, eb1);
		}
		else {
			nearest_entity = EdgeEdgeDistanceType::EA1_EB;
			return point_line_sq_distance(ea1, eb0, eb1);
		}
	}
	else {
		if (t <= 0.0) {
			nearest_entity = EdgeEdgeDistanceType::EA_EB0;
			return point_line_sq_distance(eb0, ea0, ea1);
		}
		else if (t >= 1.0) {
			nearest_entity = EdgeEdgeDistanceType::EA_EB1;
			return point_line_sq_distance(eb1, ea0, ea1);
		}
		else {
			nearest_entity = EdgeEdgeDistanceType::EA_EB;
			return line_line_sq_distance(ea0, ea1, eb0, eb1);
		}
	}
}
std::array<double, 2> point_triangle_unrolled_edge_parametrization(const tmcd::Vec3d& p, const tmcd::Vec3d& e0, const tmcd::Vec3d& e1, const tmcd::Vec3d& n)
{
    /* This code has been generated with sympy to reproduce what was originally in the IPC toolkit
        Vec3d basis_0 = t1 - t0;
        Vec3d basis_1 = basis_0.cross(normal);
        param.col(0) = (basis * basis.transpose()).ldlt().solve(basis * (p - t0));

        This avoids linear algebra dependencies and it's faster.
    */
    std::array<double, 2> param;
    const double x0 = e0[0]*e0[0];
    const double x1 = e0[1]*e0[1];
    const double x2 = e0[2]*e0[2];
    const double x3 = e1[0]*e1[0];
    const double x4 = e1[1]*e1[1];
    const double x5 = e1[2]*e1[2];
    const double x6 = 2*e0[0];
    const double x7 = e1[0]*x6;
    const double x8 = 2*e0[1];
    const double x9 = e1[1]*x8;
    const double x10 = 2*e1[2];
    const double x11 = e0[2]*x10;
    const double x12 = -e0[0] + e1[0];
    const double x13 = -e0[0] + p[0];
    const double x14 = -e0[1] + e1[1];
    const double x15 = -e0[1] + p[1];
    const double x16 = -e0[2] + e1[2];
    const double x17 = -e0[2] + p[2];
    const double x18 = n[0]*x6;
    const double x19 = n[1]*x18;
    const double x20 = e0[2]*n[2];
    const double x21 = e1[2]*n[2];
    const double x22 = n[1]*x8;
    const double x23 = e1[0]*n[0];
    const double x24 = 2*x23;
    const double x25 = e1[1]*n[1];
    const double x26 = n[2]*x10;
    const double x27 = n[1]*n[1];
    const double x28 = n[2]*n[2];
    const double x29 = n[0]*n[0];
    param[0] = (x12*x13 + x14*x15 + x16*x17)/(x0 + x1 - x11 + x2 + x3 + x4 + x5 - x7 - x9);
    param[1] = (x13*(-n[1]*x16 + n[2]*x14) + x15*(n[0]*x16 - n[2]*x12) + x17*(-n[0]*x14 + n[1]*x12))/(-e0[1]*x19 + e1[1]*x19 + x0*x27 + x0*x28 + x1*x28 + x1*x29 - x11*x27 - x11*x29 - x18*x20 + x18*x21 + x2*x27 + x2*x29 - x20*x22 + x20*x24 + 2*x20*x25 + x21*x22 + x22*x23 - x23*x26 - x24*x25 - x25*x26 + x27*x3 + x27*x5 - x27*x7 + x28*x3 + x28*x4 - x28*x7 - x28*x9 + x29*x4 + x29*x5 - x29*x9);
    return param;
}
tmcd::PointTriangleDistanceType tmcd::point_triangle_distance_type(const Vec3d& p, const Vec3d& t0, const Vec3d& t1, const Vec3d& t2)
{
    const Vec3d normal = (t1 - t0).cross(t2 - t0);

    const std::array<double, 2> param_0 = point_triangle_unrolled_edge_parametrization(p, t0, t1, normal);
    if (param_0[0] > 0.0 && param_0[0] < 1.0 && param_0[1] >= 0.0) {
        return PointTriangleDistanceType::P_E0; // edge 0 is the closest
    }

    const std::array<double, 2> param_1 = point_triangle_unrolled_edge_parametrization(p, t1, t2, normal);
    if (param_1[0] > 0.0 && param_1[0] < 1.0 && param_1[1] >= 0.0) {
        return PointTriangleDistanceType::P_E1; // edge 1 is the closest
    }

    const std::array<double, 2> param_2 = point_triangle_unrolled_edge_parametrization(p, t2, t0, normal);
    if (param_2[0] > 0.0 && param_2[0] < 1.0 && param_2[1] >= 0.0) {
        return PointTriangleDistanceType::P_E2; // edge 2 is the closest
    }

    if (param_0[0] <= 0.0 && param_2[0] >= 1.0) {
        return PointTriangleDistanceType::P_T0; // vertex 0 is the closest
    }
    else if (param_1[0] <= 0.0 && param_0[0] >= 1.0) {
        return PointTriangleDistanceType::P_T1; // vertex 1 is the closest
    }
    else if (param_2[0] <= 0.0 && param_1[0] >= 1.0) {
        return PointTriangleDistanceType::P_T2; // vertex 2 is the closest
    }
    else {
        return PointTriangleDistanceType::P_T;
    }
}
double tmcd::point_triangle_sq_distance(PointTriangleDistanceType& nearest_entity, const Vec3d & p, const Vec3d & t0, const Vec3d & t1, const Vec3d & t2)
{
    nearest_entity = point_triangle_distance_type(p, t0, t1, t2);
    switch (nearest_entity) {
    case PointTriangleDistanceType::P_T0:
        return point_point_sq_distance(p, t0);

    case PointTriangleDistanceType::P_T1:
        return point_point_sq_distance(p, t1);

    case PointTriangleDistanceType::P_T2:
        return point_point_sq_distance(p, t2);

    case PointTriangleDistanceType::P_E0:
        return point_line_sq_distance(p, t0, t1);

    case PointTriangleDistanceType::P_E1:
        return point_line_sq_distance(p, t1, t2);

    case PointTriangleDistanceType::P_E2:
        return point_line_sq_distance(p, t2, t0);

    case PointTriangleDistanceType::P_T:
        return point_plane_sq_distance(p, t0, t1, t2);

    default:
        return 0.0;  // Supresses warning. This case can't happen.
    }
}
double tmcd::point_triangle_sq_unsigned_jan(PointTriangleDistanceType& nearest_entity, const Vec3d& point, const Vec3d& v0, const Vec3d& v1, const Vec3d& v2)
{
	Vec3d diff = v0 - point;
	Vec3d edge0 = v1 - v0;
	Vec3d edge1 = v2 - v0;
	double a00 = edge0.dot(edge0);
	double a01 = edge0.dot(edge1);
	double a11 = edge1.dot(edge1);
	double b0 = diff.dot(edge0);
	double b1 = diff.dot(edge1);
	double c = diff.dot(diff);
	double det = std::abs(a00 * a11 - a01 * a01);
	double s = a01 * b1 - a11 * b0;
	double t = a01 * b0 - a00 * b1;

	double d2 = -1.0;

	if (s + t <= det)
	{
		if (s < 0)
		{
			if (t < 0)  // region 4
			{
				if (b0 < 0)
				{
					t = 0;
					if (-b0 >= a00)
					{
						nearest_entity = PointTriangleDistanceType::P_T1;
						s = 1;
						d2 = a00 + (2) * b0 + c;
					}
					else
					{
						nearest_entity = PointTriangleDistanceType::P_E0;
						s = -b0 / a00;
						d2 = b0 * s + c;
					}
				}
				else
				{
					s = 0;
					if (b1 >= 0)
					{
						nearest_entity = PointTriangleDistanceType::P_T0;
						t = 0;
						d2 = c;
					}
					else if (-b1 >= a11)
					{
						nearest_entity = PointTriangleDistanceType::P_T2;
						t = 1;
						d2 = a11 + (2) * b1 + c;
					}
					else
					{
						nearest_entity = PointTriangleDistanceType::P_E2;
						t = -b1 / a11;
						d2 = b1 * t + c;
					}
				}
			}
			else  // region 3
			{
				s = 0;
				if (b1 >= 0)
				{
					nearest_entity = PointTriangleDistanceType::P_T0;
					t = 0;
					d2 = c;
				}
				else if (-b1 >= a11)
				{
					nearest_entity = PointTriangleDistanceType::P_T2;
					t = 1;
					d2 = a11 + (2) * b1 + c;
				}
				else
				{
					nearest_entity = PointTriangleDistanceType::P_E2;
					t = -b1 / a11;
					d2 = b1 * t + c;
				}
			}
		}
		else if (t < 0)  // region 5
		{
			t = 0;
			if (b0 >= 0)
			{
				nearest_entity = PointTriangleDistanceType::P_T0;
				s = 0;
				d2 = c;
			}
			else if (-b0 >= a00)
			{
				nearest_entity = PointTriangleDistanceType::P_T1;
				s = 1;
				d2 = a00 + (2) * b0 + c;
			}
			else
			{
				nearest_entity = PointTriangleDistanceType::P_E0;
				s = -b0 / a00;
				d2 = b0 * s + c;
			}
		}
		else  // region 0 
		{
			nearest_entity = PointTriangleDistanceType::P_T;
			// minimum at interior point
			double invDet = (1) / det;
			s *= invDet;
			t *= invDet;
			d2 = s * (a00 * s + a01 * t + (2) * b0) +
				t * (a01 * s + a11 * t + (2) * b1) + c;
		}
	}
	else
	{
		double tmp0, tmp1, numer, denom;

		if (s < 0)  // region 2
		{
			tmp0 = a01 + b0;
			tmp1 = a11 + b1;
			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - (2) * a01 + a11;
				if (numer >= denom)
				{
					nearest_entity = PointTriangleDistanceType::P_T1;
					s = 1;
					t = 0;
					d2 = a00 + (2) * b0 + c;
				}
				else
				{
					nearest_entity = PointTriangleDistanceType::P_E1;
					s = numer / denom;
					t = 1 - s;
					d2 = s * (a00 * s + a01 * t + (2) * b0) +
						t * (a01 * s + a11 * t + (2) * b1) + c;
				}
			}
			else
			{
				s = 0;
				if (tmp1 <= 0)
				{
					nearest_entity = PointTriangleDistanceType::P_T2;
					t = 1;
					d2 = a11 + (2) * b1 + c;
				}
				else if (b1 >= 0)
				{
					nearest_entity = PointTriangleDistanceType::P_T0;
					t = 0;
					d2 = c;
				}
				else
				{
					nearest_entity = PointTriangleDistanceType::P_E2;
					t = -b1 / a11;
					d2 = b1 * t + c;
				}
			}
		}
		else if (t < 0)  // region 6
		{
			tmp0 = a01 + b1;
			tmp1 = a00 + b0;
			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - (2) * a01 + a11;
				if (numer >= denom)
				{
					nearest_entity = PointTriangleDistanceType::P_T2;
					t = 1;
					s = 0;
					d2 = a11 + (2) * b1 + c;
				}
				else
				{
					nearest_entity = PointTriangleDistanceType::P_E1;
					t = numer / denom;
					s = 1 - t;
					d2 = s * (a00 * s + a01 * t + (2) * b0) +
						t * (a01 * s + a11 * t + (2) * b1) + c;
				}
			}
			else
			{
				t = 0;
				if (tmp1 <= 0)
				{
					nearest_entity = PointTriangleDistanceType::P_T1;
					s = 1;
					d2 = a00 + (2) * b0 + c;
				}
				else if (b0 >= 0)
				{
					nearest_entity = PointTriangleDistanceType::P_T0;
					s = 0;
					d2 = c;
				}
				else
				{
					nearest_entity = PointTriangleDistanceType::P_E0;
					s = -b0 / a00;
					d2 = b0 * s + c;
				}
			}
		}
		else  // region 1
		{
			numer = a11 + b1 - a01 - b0;
			if (numer <= 0)
			{
				nearest_entity = PointTriangleDistanceType::P_T2;
				s = 0;
				t = 1;
				d2 = a11 + (2) * b1 + c;
			}
			else
			{
				denom = a00 - (2) * a01 + a11;
				if (numer >= denom)
				{
					nearest_entity = PointTriangleDistanceType::P_T1;
					s = 1;
					t = 0;
					d2 = a00 + (2) * b0 + c;
				}
				else
				{
					nearest_entity = PointTriangleDistanceType::P_E1;
					s = numer / denom;
					t = 1 - s;
					d2 = s * (a00 * s + a01 * t + (2) * b0) +
						t * (a01 * s + a11 * t + (2) * b1) + c;
				}
			}
		}
	}

	// Account for numerical round-off error.
	if (d2 < 0)
	{
		d2 = 0;
	}
	return d2;
}
// https://stackoverflow.com/questions/42740765/intersection-between-line-and-triangle-in-3d
bool tmcd::is_edge_intersecting_triangle(const Vec3d& Q1, const Vec3d& Q2, const Vec3d& A, const Vec3d& B, const Vec3d& C)
{
	/*
		Warning: Coplanarity is reported as false. (Same as IPC toolkit)

		We rely on the chances for coplanar intersection in IPC simulations being virtually impossible to happen.
		Even when they happen, the Newton Solver should handle the case of inf barrier and reduce the step.
	*/
	const Vec3d E1 = B - A;
	const Vec3d E2 = C - A;
	const Vec3d N = E1.cross(E2);
	const Vec3d Dir = Q2 - Q1;
	const double det = -Dir.dot(N);
	const double invdet = 1.0 / det;
	const Vec3d AO = Q1 - A;
	const Vec3d DAO = AO.cross(Dir);
	const double u = E2.dot(DAO) * invdet;
	const double v = -E1.dot(DAO) * invdet;
	const double t = AO.dot(N) * invdet;
	return (std::abs(det) >= 1e-14 && t >= 0.0 && t <= 1.0 && u >= 0.0 && v >= 0.0 && (u + v) <= 1.0);
}
