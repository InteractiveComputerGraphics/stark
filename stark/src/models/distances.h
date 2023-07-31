#pragma once
#include <math.h>
#include <functional>

#include <symx>

namespace stark::models
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
	SCALAR distance_line_line(const VECTOR3D& a, const VECTOR3D& b, const VECTOR3D& p, const VECTOR3D& q)
	{
		symx::Vector n = (b - a).cross3(q - p);
		symx::Scalar l = (p - a).dot(normal);
		symx::Scalar d2 = l*l/n.dot(n);
		return sqrt(d2);
	}

	//template<typename POTENTIAL>
	//symx::Scalar potential_edge_edge_mollified(const symx::Vector& p, const symx::Vector& q, const symx::Vector& a, const symx::Vector& b, const symx::Vector& P, const symx::Vector& Q, const symx::Vector& A, const symx::Vector& B, const POTENTIAL psi)
	//{
	//	// Distance between two edges
	//	const symx::Vector n = (q - p).cross3(b - a);
	//	const symx::Scalar n_norm_sq = n.dot(n);
	//	const symx::Scalar n_norm = sqrt(n_norm_sq);
	//	const symx::Vector n_normalized = n/n_norm;
	//	const symx::Scalar signed_d = (p - a).dot(n_normalized);
	//	const symx::Scalar d = sqrt(signed_d * signed_d);

	//	// IPC mollifier that is 1 everywhere but vanishes towards n_norm_sq -> 0
	//	const symx::Scalar eps = (q - p).squared_norm()*(b - a).squared_norm()*1e-3;
	//	const symx::Scalar cond = n_norm_sq - eps;
	//	const symx::Scalar mollifier_f = (-n_norm_sq/eps + 2.0) * n_norm_sq/eps;
	//	const symx::Scalar mollifier = sym::branch(cond, cond.get_one(), mollifier_f);

	//	// IPC mollified edge-edge potential
	//	const symx::Scalar potential = mollifier * psi(d);  // Note: This will evaluate nearly parallel cases in psi

	//	// We don't evaluate anything before if almost parallel (that's the whole point) and return zero directly
	//	const symx::Scalar potential_no_singularity = sym::branch(n_norm - 1e-12, potential, cond.get_zero());

	//	return potential_no_singularity;
	//}

	symx::Scalar mollifier_cubic(const symx::Scalar& x, const symx::Scalar& cutoff, const symx::Scalar& threshold)
	{
		const symx::Scalar a = 2.0/(cutoff.powN(3) - 3.0*cutoff.powN(2)*threshold + 3.0*cutoff*threshold.powN(2) - threshold.powN(3));
		const symx::Scalar b = (-3.0*cutoff - 3.0*threshold)/(cutoff.powN(3) - 3.0*cutoff.powN(2)*threshold + 3.0*cutoff*threshold.powN(2) - threshold.powN(3));
		const symx::Scalar c = 6.0*cutoff*threshold/(cutoff.powN(3) - 3.0*cutoff.powN(2)*threshold + 3.0*cutoff*threshold.powN(2) - threshold.powN(3));
		const symx::Scalar d = (cutoff.powN(3) - 3.0*cutoff.powN(2)*threshold)/(cutoff.powN(3) - 3.0*cutoff.powN(2)*threshold + 3.0*cutoff*threshold.powN(2) - threshold.powN(3));
		const symx::Scalar f = a*x.powN(3) + b*x.powN(2) + c*x + d;
		return symx::branch(x - threshold, x.get_one(), f);
	}
	//symx::Scalar edge_edge_cos_angle(
	//	const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q,
	//	const symx::Vector& A, const symx::Vector& B, const symx::Vector& P, const symx::Vector& Q)
	//{
	//	const symx::Vector u = b - a;
	//	const symx::Vector v = q - p;
	//	const symx::Vector U = B - A;
	//	const symx::Vector V = Q - P;
	//	return u.dot(v) / (U.norm() * V.norm());
	//}


	//symx::Scalar potential_edge_edge_mollified(
	//	const symx::Vector& a, const symx::Vector& b, const symx::Vector& p, const symx::Vector& q, 
	//	const symx::Vector& A, const symx::Vector& B, const symx::Vector& P, const symx::Vector& Q, 
	//	const symx::Scalar& cutoff, const symx::Scalar& threshold, const std::function<symx::Scalar(symx::Scalar)> barrier)
	//{
	//	// Mollifier
	//	const symx::Vector u = b - a;
	//	const symx::Vector v = q - p;
	//	const symx::Vector U = B - A;
	//	const symx::Vector V = Q - P;
	//	const symx::Scalar cos = u.dot(v)/(U.norm()*V.norm());
	//	const symx::Scalar mollifier = mollifier_cubic(cos, cutoff, threshold);

	//	// Distance
	//	const symx::Scalar d = distance_line_line<symx::Scalar>(a, b, p, q);

	//	// IPC mollified edge-edge potential
	//	const symx::Scalar potential = mollifier * barrier(d);  // Note: This will evaluate *nearly parallel* cases in psi

	//	//// We don't evaluate anything before if almost parallel (that's the whole point) and return zero directly
	//	//const symx::Scalar potential_no_singularity = sym::branch(n_norm - 1e-12, potential, cond.get_zero());
	//	//return potential_no_singularity;
	//	
	//	return potential;
	//}
}
