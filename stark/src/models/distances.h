#pragma once
#include <math.h>

namespace sym
{
	double sqrt(double a)
	{
		return std::sqrt(a);
	}

	template<typename SCALAR, typename VECTOR3D>
	SCALAR distance_point_point(const VECTOR3D& p, const VECTOR3D& q)
	{
		return (p - q).norm();
	}
	template<typename SCALAR, typename VECTOR3D>
	SCALAR distance_point_line(const VECTOR3D& p, const VECTOR3D& a, const VECTOR3D& b)
	{
		const VECTOR3D ab = b - a;
		const VECTOR3D ap = p - a;
		const SCALAR e = ap.dot(ab);
		const SCALAR ab_dot_ab = ab.dot(ab);
		return sqrt(ap.dot(ap) - e*e/ab_dot_ab);
	}
	template<typename SCALAR, typename VECTOR3D>
	SCALAR distance_point_plane(const VECTOR3D& p, const VECTOR3D& a, const VECTOR3D& b, const VECTOR3D& c)
	{
		const VECTOR3D vpa = p - a;
		const VECTOR3D vac = a - c;
		const VECTOR3D vbc = b - c;
		const VECTOR3D n = vac.cross3(vbc).normalized();
		const SCALAR d = vpa.dot(n);
		return sqrt(d*d);
	}
	template<typename SCALAR, typename VECTOR3D>
	SCALAR distance_line_line(const VECTOR3D& v11, const VECTOR3D& v12, const VECTOR3D& v21, const VECTOR3D& v22)
	{
		const Scalar signed_d = (v11 - v21).dot((v12 - v11).cross3(v22 - v21).normalized());
		return sqrt(signed_d * signed_d);
	}

	//template<typename POTENTIAL>
	//Scalar potential_edge_edge_mollified(const Vector& p, const Vector& q, const Vector& a, const Vector& b, const Vector& P, const Vector& Q, const Vector& A, const Vector& B, const POTENTIAL psi)
	//{
	//	// Distance between two edges
	//	const Vector n = (q - p).cross3(b - a);
	//	const Scalar n_norm_sq = n.dot(n);
	//	const Scalar n_norm = sqrt(n_norm_sq);
	//	const Vector n_normalized = n/n_norm;
	//	const Scalar signed_d = (p - a).dot(n_normalized);
	//	const Scalar d = sqrt(signed_d * signed_d);

	//	// IPC mollifier that is 1 everywhere but vanishes towards n_norm_sq -> 0
	//	const Scalar eps = (q - p).squared_norm()*(b - a).squared_norm()*1e-3;
	//	const Scalar cond = n_norm_sq - eps;
	//	const Scalar mollifier_f = (-n_norm_sq/eps + 2.0) * n_norm_sq/eps;
	//	const Scalar mollifier = sym::branch(cond, cond.get_one(), mollifier_f);

	//	// IPC mollified edge-edge potential
	//	const Scalar potential = mollifier * psi(d);  // Note: This will evaluate nearly parallel cases in psi

	//	// We don't evaluate anything before if almost parallel (that's the whole point) and return zero directly
	//	const Scalar potential_no_singularity = sym::branch(n_norm - 1e-12, potential, cond.get_zero());

	//	return potential_no_singularity;
	//}

	template<typename POTENTIAL>
	Scalar potential_edge_edge_mollified(const Vector& v11, const Vector& v12, const Vector& v21, const Vector& v22, const Vector& V11, const Vector& V12, const Vector& V21, const Vector& V22, const POTENTIAL psi)
	{
		// IPC mollifier (Eq. 24)
		const Scalar c = ((v12 - v11).cross3(v22 - v21)).squared_norm();
		const Scalar ex = 1e-3*((V12 - V11).cross3(V22 - V21)).squared_norm();
		const Scalar mollifier = sym::branch(c - ex, c.get_one(), -1.0/ex.powN(2)*c.powN(2) + 2.0/ex*c);

		// Distance
		const Scalar d = distance_line_line<Scalar>(v11, v12, v21, v22);

		// IPC mollified edge-edge potential
		const Scalar potential = mollifier * psi(d);  // Note: This will evaluate *nearly parallel* cases in psi

		//// We don't evaluate anything before if almost parallel (that's the whole point) and return zero directly
		//const Scalar potential_no_singularity = sym::branch(n_norm - 1e-12, potential, cond.get_zero());
		//return potential_no_singularity;
		
		return potential;
	}
}