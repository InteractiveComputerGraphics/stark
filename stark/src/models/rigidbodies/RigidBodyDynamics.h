#pragma once
#include <string>
#include <vector>
#include <array>
#include <memory>

#include <Eigen/Dense>

#include "../../core/Stark.h"


namespace stark
{
	class RigidBodyDynamics
	{
	private:

	public:
		/* Fields */
		std::vector<Eigen::Vector3d> t0;  // [m] Translation at time n
		std::vector<Eigen::Vector3d> t1;  // [m] Translation at time n+1
		std::vector<Eigen::Quaterniond> q0;  // Quaternion at time n
		std::vector<std::array<double, 4>> q0_;  // Quaternion at time n (for SymX)
		std::vector<Eigen::Quaterniond> q1;  // Quaternion at time n+1
		std::vector<Eigen::Matrix3d> R0;  // Rotation matrix at time n
		std::vector<Eigen::Matrix3d> R1;  // Rotation matrix at time n+1
		std::vector<Eigen::Vector3d> v0;  // [m/s] Velocity at time n
		std::vector<Eigen::Vector3d> v1;  // [m/s] Velocity at time n+1
		std::vector<Eigen::Vector3d> w0;  // [rad/s] Angular velocity at time n
		std::vector<Eigen::Vector3d> w1;  // [rad/s] Angular velocity at time n+1
		std::vector<Eigen::Vector3d> a;  // [m/s^2] Acceleration
		std::vector<Eigen::Vector3d> aa;  // [rad/s^2] Angular acceleration
		std::vector<Eigen::Vector3d> force;  // [N] Force
		std::vector<Eigen::Vector3d> torque;  // [Nm] Torque
		symx::DoF dof_v;
		symx::DoF dof_w;
		std::vector<std::string> labels;

		/* Methods */
		RigidBodyDynamics(core::Stark& stark);
		int add(const std::string& label = "");
		int get_n_bodies() const;

		// Position and direction getters
		//// With time integration (used during minimization)
		symx::Vector get_x1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Scalar& dt);
		std::vector<symx::Vector> get_x1(symx::Energy& energy, const symx::Index& rb_idx, const std::vector<symx::Vector>& x_loc, const symx::Scalar& dt);
		symx::Vector get_v1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Scalar& dt);
		std::vector<symx::Vector> get_v1(symx::Energy& energy, const symx::Index& rb_idx, const std::vector<symx::Vector>& x_loc, const symx::Scalar& dt);
		symx::Vector get_d1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& d_loc, const symx::Scalar& dt);
		std::array<symx::Vector, 2> get_x1_d1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Vector& d_loc, const symx::Scalar& dt);
		std::array<symx::Vector, 2> get_x0_x1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Scalar& dt);

		Eigen::Vector3d get_x1(int rb, const Eigen::Vector3d& x_loc, double dt) const;
		Eigen::Vector3d get_d1(int rb, const Eigen::Vector3d& d_loc, double dt) const;

		//// With final positions
		Eigen::Vector3d get_position_at(int rb, const Eigen::Vector3d& x_loc) const;
		Eigen::Vector3d get_velocity_at(int rb, const Eigen::Vector3d& x_loc) const;
		Eigen::Vector3d get_direction(int rb, const Eigen::Vector3d& d_loc) const;

	private:
		void _before_time_step(stark::core::Stark& stark);
		void _on_time_step_accepted(stark::core::Stark& stark);
	};
	using spRigidBodyDynamics = std::shared_ptr<RigidBodyDynamics>;
}
