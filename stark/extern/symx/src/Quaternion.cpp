#include "Quaternion.h"

symx::Quaternion symx::Quaternion::from_vec4(const Vector& q) {
    assert(q.nrows == 4);
    return Quaternion { q };
}

symx::Vector symx::Quaternion::to_vec3(SymbolicWorkSpace& sws) const {
    return symx::quat_to_vec3(this->coeffs, sws);
}

symx::Matrix symx::Quaternion::to_rotation(SymbolicWorkSpace& sws) const {
    return symx::quat_to_rotation(this->coeffs, sws);
}

symx::Quaternion symx::Quaternion::normalized() const {
    return Quaternion::from_vec4(symx::normalize_quaternion(this->coeffs));
}

symx::Quaternion symx::Quaternion::inverse() const {
    return Quaternion::from_vec4(symx::quat_conjugated(this->coeffs));
}

symx::Quaternion symx::Quaternion::prod(const Quaternion& q2) const {
    return Quaternion::from_vec4(symx::quat_dot_product(this->coeffs, q2.coeffs));
}

symx::Quaternion symx::Quaternion::time_integration_with_global_w(const Vector& w_glob, const Scalar& dt) const {
    return Quaternion::from_vec4(symx::quat_time_integration_with_global_w(this->coeffs, w_glob, dt));
}

symx::Quaternion symx::Quaternion::operator*(const Quaternion& other) const {
    return this->prod(other);
}

symx::Quaternion symx::Quaternion::operator+(const symx::Quaternion& other) const {
    return Quaternion::from_vec4(this->coeffs + other.coeffs);
}

symx::Quaternion symx::Quaternion::operator*(const symx::Scalar& other) const {
    return Quaternion::from_vec4(this->coeffs * other);
}

symx::Quaternion symx::operator*(const symx::Scalar& scalar, const symx::Quaternion& quat) {
    return symx::Quaternion::from_vec4(scalar * quat.coeffs);
}

symx::Vector symx::quat_to_vec3(const Vector& q, SymbolicWorkSpace& sws) {
    Vector v = sws.get_zero_vector(3);
    v[0] = q[1];
    v[1] = q[2];
    v[2] = q[3];
    return v;
}

// Normalized quaternion to rotation matrix as in Eigen
symx::Matrix symx::quat_to_rotation(const Vector& q, SymbolicWorkSpace& sws)
{
	Matrix R = sws.get_zero_matrix({ 3, 3 });

	const Scalar& qw = q[0];
	const Scalar& qx = q[1];
	const Scalar& qy = q[2];
	const Scalar& qz = q[3];

	Scalar tx = 2.0 * qx;
	Scalar ty = 2.0 * qy;
	Scalar tz = 2.0 * qz;
	Scalar twx = tx * qw;
	Scalar twy = ty * qw;
	Scalar twz = tz * qw;
	Scalar txx = tx * qx;
	Scalar txy = ty * qx;
	Scalar txz = tz * qx;
	Scalar tyy = ty * qy;
	Scalar tyz = tz * qy;
	Scalar tzz = tz * qz;

	R(0, 0) = 1.0 - (tyy + tzz);
	R(0, 1) = txy - twz;
	R(0, 2) = txz + twy;
	R(1, 0) = txy + twz;
	R(1, 1) = 1.0 - (txx + tzz);
	R(1, 2) = tyz - twx;
	R(2, 0) = txz - twy;
	R(2, 1) = tyz + twx;
	R(2, 2) = 1.0 - (txx + tyy);

	return R;
}
symx::Vector symx::quat_dot_product(const Vector& q1, const Vector& q2)
{
	// w, x, y, z = q
	const Scalar& a = q1[0];
	const Scalar& b = q1[1];
	const Scalar& c = q1[2];
	const Scalar& d = q1[3];

	const Scalar& e = q2[0];
	const Scalar& f = q2[1];
	const Scalar& g = q2[2];
	const Scalar& h = q2[3];

	return Vector({
		a*e - b*f - c*g - d*h,
		b*e + a*f + c*h - d*g,
		a*g - b*h + c*e + d*f,
		a*h + b*g - c*f + d*e 
	});
}
symx::Vector symx::quat_conjugated(const Vector& q1)
{
	// w, x, y, z = q
	const Scalar& a = q1[0];
	const Scalar& b = q1[1];
	const Scalar& c = q1[2];
	const Scalar& d = q1[3];
	return Vector({ a, -b, -c, -d });
}
symx::Vector symx::quat_time_integration_with_global_w(const Vector& q_start, const Vector& w_glob, const Scalar& dt)
{
	Vector w_({ dt.get_zero(), w_glob[0], w_glob[1], w_glob[2] });
	return q_start + 0.5 * dt * quat_dot_product(w_, q_start);
}
symx::Vector symx::normalize_quaternion(const Vector& q)
{
	return q / q.norm();
}
symx::Vector symx::loc_to_glob_point(const symx::Vector& p_loc, const symx::Vector& t, const symx::Vector& q, SymbolicWorkSpace& sws)
{
	return t + symx::quat_to_rotation(q, sws) * p_loc;
}
symx::Vector symx::integrate_loc_point(const symx::Vector& p_loc, const symx::Vector& t0, const symx::Vector& q0, const symx::Vector& v1, const symx::Vector& w1, const symx::Scalar& dt, SymbolicWorkSpace& sws)
{
	symx::Vector q1 = symx::normalize_quaternion(symx::quat_time_integration_with_global_w(q0, w1, dt));
	symx::Matrix R1 = symx::quat_to_rotation(q1, sws);
	symx::Vector t1 = t0 + dt*v1;
	return t1 + R1*p_loc;
}
symx::Vector symx::integrate_loc_direction(const symx::Vector& d_loc, const symx::Vector& q0, const symx::Vector& w1, const symx::Scalar& dt, SymbolicWorkSpace& sws)
{
	symx::Vector q1 = symx::normalize_quaternion(symx::quat_time_integration_with_global_w(q0, w1, dt));
	symx::Matrix R1 = symx::quat_to_rotation(q1, sws);
	return R1 * d_loc;
}
