#include "rigidbody_transformations.h"

#include "../time_integration.h"

// Eigen
Eigen::Vector3d stark::local_to_global_point(const Eigen::Vector3d& x, const Eigen::Matrix3d& R, const Eigen::Vector3d& translation)
{
	return translation + R * x;
}
Eigen::Vector3d stark::local_to_global_direction(const Eigen::Vector3d& x, const Eigen::Matrix3d& R)
{
	return R * x;
}
Eigen::Matrix3d stark::local_to_global_matrix(const Eigen::Matrix3d& A, const Eigen::Matrix3d& R)
{
	return R * A * R.transpose();
}
Eigen::Vector3d stark::global_to_local_point(const Eigen::Vector3d& x, const Eigen::Matrix3d& R, const Eigen::Vector3d& translation)
{
	return R.transpose() * (x - translation);
}
Eigen::Vector3d stark::global_to_local_direction(const Eigen::Vector3d& x, const Eigen::Matrix3d& R)
{
	return R.transpose() * x;
}
Eigen::Matrix3d stark::global_to_local_matrix(const Eigen::Matrix3d& A, const Eigen::Matrix3d& R)
{
	return R.transpose() * A * R;
}
Eigen::Quaterniond stark::quat_time_integration(const Eigen::Quaterniond& q_start, const Eigen::Vector3d& w_glob, const double dt)
{
	// https:// stackoverflow.com/questions/46908345/integrate-angular-velocity-as-quaternion-rotation?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
	Eigen::Quaterniond w_ = Eigen::Quaterniond(0, w_glob.x(), w_glob.y(), w_glob.z());
	Eigen::Quaterniond q_end = q_start;
	q_end.coeffs() += 0.5 * dt * (w_ * q_start).coeffs();
	return q_end.normalized();
}
Eigen::Vector3d stark::integrate_loc_point(const Eigen::Vector3d& p_loc, const Eigen::Vector3d& t0, const Eigen::Quaterniond& q0, const Eigen::Vector3d& v1, const Eigen::Vector3d& w1, const double& dt)
{
	Eigen::Vector3d t1 = time_integration(t0, v1, dt);
	Eigen::Matrix3d R1 = quat_time_integration(q0, w1, dt).toRotationMatrix();
	return local_to_global_point(p_loc, R1, t1);
}
Eigen::Vector3d stark::integrate_loc_direction(const Eigen::Vector3d& d_loc, const Eigen::Quaterniond& q0, const Eigen::Vector3d& w1, const double& dt)
{
	Eigen::Matrix3d R1 = quat_time_integration(q0, w1, dt).toRotationMatrix();
	return local_to_global_direction(d_loc, R1);
}




// SymX
symx::Matrix stark::quat_to_rotation(const symx::Vector& q)
{
	symx::Matrix R = symx::Matrix::identity(3, q[0]);

	const symx::Scalar& qw = q[0];
	const symx::Scalar& qx = q[1];
	const symx::Scalar& qy = q[2];
	const symx::Scalar& qz = q[3];

	const symx::Scalar tx = 2.0 * qx;
	const symx::Scalar ty = 2.0 * qy;
	const symx::Scalar tz = 2.0 * qz;
	const symx::Scalar twx = tx * qw;
	const symx::Scalar twy = ty * qw;
	const symx::Scalar twz = tz * qw;
	const symx::Scalar txx = tx * qx;
	const symx::Scalar txy = ty * qx;
	const symx::Scalar txz = tz * qx;
	const symx::Scalar tyy = ty * qy;
	const symx::Scalar tyz = tz * qy;
	const symx::Scalar tzz = tz * qz;

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
symx::Vector stark::quat_dot_product(const symx::Vector& q1, const symx::Vector& q2)
{
	// w, x, y, z = q
	const symx::Scalar& a = q1[0];
	const symx::Scalar& b = q1[1];
	const symx::Scalar& c = q1[2];
	const symx::Scalar& d = q1[3];

	const symx::Scalar& e = q2[0];
	const symx::Scalar& f = q2[1];
	const symx::Scalar& g = q2[2];
	const symx::Scalar& h = q2[3];

	return symx::Vector({
		a * e - b * f - c * g - d * h,
		b * e + a * f + c * h - d * g,
		a * g - b * h + c * e + d * f,
		a * h + b * g - c * f + d * e
		}
	);
}
symx::Vector stark::quat_conjugated(const symx::Vector& q1)
{
	// w, x, y, z = q
	const symx::Scalar& a = q1[0];
	const symx::Scalar& b = q1[1];
	const symx::Scalar& c = q1[2];
	const symx::Scalar& d = q1[3];
	return symx::Vector({ a, -b, -c, -d });
}
symx::Vector stark::quat_time_integration(const symx::Vector& q_start, const symx::Vector& w_glob, const symx::Scalar& dt)
{
	const symx::Vector w_({ dt.get_zero(), w_glob[0], w_glob[1], w_glob[2] });
	const symx::Vector q1 = q_start + 0.5 * dt * quat_dot_product(w_, q_start);
	return q1.normalized();
}
symx::Matrix stark::quat_time_integration_as_rotation_matrix(const symx::Vector& q_start, const symx::Vector& w_glob, const symx::Scalar& dt)
{
	symx::Vector q1 = quat_time_integration(q_start, w_glob, dt);
	return quat_to_rotation(q1);
}
symx::Vector stark::local_to_global_point(const symx::Vector& p_loc, const symx::Vector& t, const symx::Matrix& R)
{
	return t + R*p_loc;
}
symx::Vector stark::local_to_global_point(const symx::Vector& p_loc, const symx::Vector& t, const symx::Vector& q)
{
	return local_to_global_point(p_loc, t, quat_to_rotation(q));
}
symx::Vector stark::local_to_global_direction(const symx::Vector& d_loc, const symx::Matrix& R)
{
	return R*d_loc;
}
symx::Vector stark::local_to_global_direction(const symx::Vector& d_loc, const symx::Vector& q)
{
	return local_to_global_direction(d_loc, quat_to_rotation(q));
}
symx::Vector stark::integrate_loc_point(const symx::Vector& p_loc, const symx::Vector& t0, const symx::Vector& q0, const symx::Vector& v1, const symx::Vector& w1, const symx::Scalar& dt)
{
	symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
	symx::Vector t1 = t0 + dt * v1;
	return local_to_global_point(p_loc, t1, R1);
}
symx::Vector stark::integrate_loc_direction(const symx::Vector& d_loc, const symx::Vector& q0, const symx::Vector& w1, const symx::Scalar& dt)
{
	symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
	return local_to_global_direction(d_loc, R1);
}

symx::Vector stark::global_point_velocity_in_rigib_body(const symx::Vector& v_body, const symx::Vector& w_body, const symx::Vector& r_glob)
{
	return v_body + w_body.cross3(r_glob);
}
