#pragma once
#include "SymbolicWorkSpace.h"


namespace symx
{
    class Quaternion {
    public:
        symx::Vector coeffs;

        static Quaternion from_vec4(const Vector& q);

        /// Returns the vector (or imaginary) part of the quaternion
        Vector to_vec3(SymbolicWorkSpace& sws) const;
        /// Converts the quaternion to a rotation matrix (does not normalize)
        Matrix to_rotation(SymbolicWorkSpace& sws) const;

        Quaternion normalized() const;
        Quaternion inverse() const;

        /// Computes the quaternion product with the other quaternion
        Quaternion prod(const Quaternion& other) const;

        Quaternion time_integration_with_global_w(const Vector& w_glob, const Scalar& dt) const;

        Quaternion operator+(const Quaternion& other) const;
        Quaternion operator*(const Quaternion& other) const;
        Quaternion operator*(const Scalar& other) const;
    };

    Quaternion operator*(const Scalar& scalar, const Quaternion& quat);

    Vector quat_to_vec3(const Vector& q, SymbolicWorkSpace& sws);
	Matrix quat_to_rotation(const Vector& q, SymbolicWorkSpace& sws);
	Vector normalize_quaternion(const Vector& q);
	Vector quat_dot_product(const Vector& q1, const Vector& q2);
	Vector quat_conjugated(const Vector& q1);
	Vector quat_time_integration_with_global_w(const Vector& q_start, const Vector& w_glob, const Scalar& dt);

	Vector loc_to_glob_point(const symx::Vector& p_loc, const symx::Vector& t, const symx::Vector& q, SymbolicWorkSpace& sws);
	Vector integrate_loc_point(const symx::Vector& p_loc, const symx::Vector& t0, const symx::Vector& q0, const symx::Vector& v1, const symx::Vector& w1, const symx::Scalar& dt, SymbolicWorkSpace& sws);
	Vector integrate_loc_direction(const symx::Vector& d_loc, const symx::Vector& q0, const symx::Vector& w1, const symx::Scalar& dt, SymbolicWorkSpace& sws);
}
