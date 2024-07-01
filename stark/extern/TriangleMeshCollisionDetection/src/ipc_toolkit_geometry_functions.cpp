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
double clamp(double v, double min, double max)
{
	if (v < min) { return min; }
	if (v > max) { return max; }
	return v;
}
double tmcd::edge_edge_sq_distance(EdgeEdgeDistanceType& nearest_entity, const Vec3d& ea0, const Vec3d& ea1, const Vec3d& eb0, const Vec3d& eb1, const double parallel_cross_norm_sq_tolerance)
{
	nearest_entity = edge_edge_distance_type(ea0, ea1, eb0, eb1, parallel_cross_norm_sq_tolerance);
	switch (nearest_entity) {
	case EdgeEdgeDistanceType::EA0_EB0:
		return point_point_sq_distance(ea0, eb0);

	case EdgeEdgeDistanceType::EA0_EB1:
		return point_point_sq_distance(ea0, eb1);

	case EdgeEdgeDistanceType::EA1_EB0:
		return point_point_sq_distance(ea1, eb0);

	case EdgeEdgeDistanceType::EA1_EB1:
		return point_point_sq_distance(ea1, eb1);

	case EdgeEdgeDistanceType::EA_EB0:
		return point_line_sq_distance(eb0, ea0, ea1);

	case EdgeEdgeDistanceType::EA_EB1:
		return point_line_sq_distance(eb1, ea0, ea1);

	case EdgeEdgeDistanceType::EA0_EB:
		return point_line_sq_distance(ea0, eb0, eb1);

	case EdgeEdgeDistanceType::EA1_EB:
		return point_line_sq_distance(ea1, eb0, eb1);

	case EdgeEdgeDistanceType::EA_EB:
		return line_line_sq_distance(ea0, ea1, eb0, eb1);

	default:
		throw std::invalid_argument(
			"Invalid distance type for edge-edge distance!");
	}
}
tmcd::EdgeEdgeDistanceType tmcd::edge_edge_distance_type(const Vec3d& ea0, const Vec3d& ea1, const Vec3d& eb0, const Vec3d& eb1, const double parallel_cross_norm_sq_tolerance)
{
	const Vec3d u = ea1 - ea0;
	const Vec3d v = eb1 - eb0;
	const Vec3d w = ea0 - eb0;

	const double a = u.squaredNorm(); // always ≥ 0
	const double b = u.dot(v);
	const double c = v.squaredNorm(); // always ≥ 0
	const double d = u.dot(w);
	const double e = v.dot(w);
	const double D = a * c - b * b; // always ≥ 0

	// Degenerate cases should not happen in practice, but we handle them
	if (a == 0.0 && c == 0.0) {
		return EdgeEdgeDistanceType::EA0_EB0;
	}
	else if (a == 0.0) {
		return EdgeEdgeDistanceType::EA0_EB;
	}
	else if (c == 0.0) {
		return EdgeEdgeDistanceType::EA_EB0;
	}

	// Special handling for parallel edges
	// Original tolerance  ->  const double parallel_tolerance = PARALLEL_THRESHOLD * std::max(1.0, a * c);
	if (u.cross(v).squaredNorm() < parallel_cross_norm_sq_tolerance) {
		return edge_edge_parallel_distance_type(ea0, ea1, eb0, eb1);
	}

	EdgeEdgeDistanceType default_case = EdgeEdgeDistanceType::EA_EB;

	// compute the line parameters of the two closest points
	const double sN = (b * e - c * d);
	double tN, tD;   // tc = tN / tD
	if (sN <= 0.0) { // sc < 0 ⟹ the s=0 edge is visible
		tN = e;
		tD = c;
		default_case = EdgeEdgeDistanceType::EA0_EB;
	}
	else if (sN >= D) { // sc > 1 ⟹ the s=1 edge is visible
		tN = e + b;
		tD = c;
		default_case = EdgeEdgeDistanceType::EA1_EB;
	}
	else {
		tN = (a * e - b * d);
		tD = D; // default tD = D ≥ 0
		if (tN > 0.0 && tN < tD
			&& u.cross(v).squaredNorm() < parallel_cross_norm_sq_tolerance) {
			// avoid coplanar or nearly parallel EE
			if (sN < D / 2) {
				tN = e;
				tD = c;
				default_case = EdgeEdgeDistanceType::EA0_EB;
			}
			else {
				tN = e + b;
				tD = c;
				default_case = EdgeEdgeDistanceType::EA1_EB;
			}
		}
		// else default_case stays EdgeEdgeDistanceType::EA_EB
	}

	if (tN <= 0.0) { // tc < 0 ⟹ the t=0 edge is visible
		// recompute sc for this edge
		if (-d <= 0.0) {
			return EdgeEdgeDistanceType::EA0_EB0;
		}
		else if (-d >= a) {
			return EdgeEdgeDistanceType::EA1_EB0;
		}
		else {
			return EdgeEdgeDistanceType::EA_EB0;
		}
	}
	else if (tN >= tD) { // tc > 1 ⟹ the t=1 edge is visible
	 // recompute sc for this edge
		if ((-d + b) <= 0.0) {
			return EdgeEdgeDistanceType::EA0_EB1;
		}
		else if ((-d + b) >= a) {
			return EdgeEdgeDistanceType::EA1_EB1;
		}
		else {
			return EdgeEdgeDistanceType::EA_EB1;
		}
	}

	return default_case;
}
tmcd::EdgeEdgeDistanceType tmcd::edge_edge_parallel_distance_type(const Vec3d& ea0, const Vec3d& ea1, const Vec3d& eb0, const Vec3d& eb1)
{
	const Vec3d ea = ea1 - ea0;
	const double alpha = (eb0 - ea0).dot(ea) / ea.squaredNorm();
	const double beta = (eb1 - ea0).dot(ea) / ea.squaredNorm();

	uint8_t eac; // 0: EA0, 1: EA1, 2: EA
	uint8_t ebc; // 0: EB0, 1: EB1, 2: EB
	if (alpha < 0) {
		eac = (0 <= beta && beta <= 1) ? 2 : 0;
		ebc = (beta <= alpha) ? 0 : (beta <= 1 ? 1 : 2);
	}
	else if (alpha > 1) {
		eac = (0 <= beta && beta <= 1) ? 2 : 1;
		ebc = (beta >= alpha) ? 0 : (0 <= beta ? 1 : 2);
	}
	else {
		eac = 2;
		ebc = 0;
	}

	// f(0, 0) = 0000 = 0 -> EA0_EB0
	// f(0, 1) = 0001 = 1 -> EA0_EB1
	// f(1, 0) = 0010 = 2 -> EA1_EB0
	// f(1, 1) = 0011 = 3 -> EA1_EB1
	// f(2, 0) = 0100 = 4 -> EA_EB0
	// f(2, 1) = 0101 = 5 -> EA_EB1
	// f(0, 2) = 0110 = 6 -> EA0_EB
	// f(1, 2) = 0111 = 7 -> EA1_EB
	// f(2, 2) = 1000 = 8 -> EA_EB

	assert(eac != 2 || ebc != 2); // This case results in a degenerate line-line
	return EdgeEdgeDistanceType(ebc < 2 ? (eac << 1 | ebc) : (6 + eac));
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
double tmcd::point_triangle_sq_unsigned_jan_bender(PointTriangleDistanceType& nearest_entity, const Vec3d& point, const Vec3d& v0, const Vec3d& v1, const Vec3d& v2)
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
