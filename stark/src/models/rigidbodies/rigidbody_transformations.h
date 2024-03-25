#pragma once
#include <array>

#include <Eigen/Dense>
#include <symx>


namespace stark
{
    // Eigen
    Eigen::Vector3d local_to_global_point(const Eigen::Vector3d& x, const Eigen::Matrix3d& R, const Eigen::Vector3d& translation);
    Eigen::Vector3d local_to_global_direction(const Eigen::Vector3d& x, const Eigen::Matrix3d& R);
    Eigen::Matrix3d local_to_global_matrix(const Eigen::Matrix3d& A, const Eigen::Matrix3d& R);

    Eigen::Vector3d global_to_local_point(const Eigen::Vector3d& x, const Eigen::Matrix3d& R, const Eigen::Vector3d& translation);
    Eigen::Vector3d global_to_local_direction(const Eigen::Vector3d& x, const Eigen::Matrix3d& R);
    Eigen::Matrix3d global_to_local_matrix(const Eigen::Matrix3d& A, const Eigen::Matrix3d& R);

    Eigen::Quaterniond quat_time_integration(const Eigen::Quaterniond& q_start, const Eigen::Vector3d& w_glob, const double dt);
    Eigen::Vector3d integrate_loc_point(const Eigen::Vector3d& p_loc, const Eigen::Vector3d& t0, const Eigen::Quaterniond& q0, const Eigen::Vector3d& v1, const Eigen::Vector3d& w1, const double& dt);
    Eigen::Vector3d integrate_loc_direction(const Eigen::Vector3d& d_loc, const Eigen::Quaterniond& q0, const Eigen::Vector3d& w1, const double& dt);


    // SymX
    symx::Vector local_to_global_point(const symx::Vector& p_loc, const symx::Vector& t, const symx::Matrix& R);
    symx::Vector local_to_global_point(const symx::Vector& p_loc, const symx::Vector& t, const symx::Vector& q);
    symx::Vector local_to_global_direction(const symx::Vector& d_loc, const symx::Matrix& R);
    symx::Vector local_to_global_direction(const symx::Vector& d_loc, const symx::Vector& q);

    symx::Matrix quat_to_rotation(const symx::Vector& q);
    symx::Vector quat_dot_product(const symx::Vector& q1, const symx::Vector& q2);
    symx::Vector quat_conjugated(const symx::Vector& q1);
    symx::Vector quat_time_integration(const symx::Vector& q_start, const symx::Vector& w_glob, const symx::Scalar& dt);
    symx::Matrix quat_time_integration_as_rotation_matrix(const symx::Vector& q_start, const symx::Vector& w_glob, const symx::Scalar& dt);
    symx::Vector integrate_loc_point(const symx::Vector& p_loc, const symx::Vector& t0, const symx::Vector& q0, const symx::Vector& v1, const symx::Vector& w1, const symx::Scalar& dt);
    symx::Vector integrate_loc_direction(const symx::Vector& d_loc, const symx::Vector& q0, const symx::Vector& w1, const symx::Scalar& dt);

    symx::Vector global_point_velocity_in_rigib_body(const symx::Vector& v_body, const symx::Vector& w_body, const symx::Vector& r_glob);
}
