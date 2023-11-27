#pragma once
#include <vector>
#include <array>
#include <memory>

#include <Eigen/Dense>
#include <symx>

#include "../solver/Stark.h"
#include "deformables/Id.h"


namespace stark::models
{
	class RigidBodyDynamics
	{
	private:

	public:
		/* Fields */
		std::vector<Eigen::Vector3d> t0;  // Translation at time n
		std::vector<Eigen::Vector3d> t1;  // Translation at time n+1
		std::vector<Eigen::Quaterniond> q0;  // Quaternion at time n
		std::vector<std::array<double, 4>> q0_;  // Quaternion at time n (for SymX)
		std::vector<Eigen::Quaterniond> q1;  // Quaternion at time n+1
		std::vector<Eigen::Matrix3d> R0;  // Rotation matrix at time n
		std::vector<Eigen::Matrix3d> R1;  // Rotation matrix at time n+1
		std::vector<Eigen::Vector3d> v0;  // Velocity at time n
		std::vector<Eigen::Vector3d> v1;  // Velocity at time n+1
		std::vector<Eigen::Vector3d> w0;  // Angular velocity at time n
		std::vector<Eigen::Vector3d> w1;  // Angular velocity at time n+1
		std::vector<Eigen::Vector3d> a;  // Acceleration
		std::vector<Eigen::Vector3d> aa;  // Angular acceleration
		std::vector<Eigen::Vector3d> force;  // Forces
		std::vector<Eigen::Vector3d> torque;  // Torques
		symx::DoF dof_v;
		symx::DoF dof_w;

		/* Methods */
		RigidBodyDynamics(Stark& stark);
		int add(const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });
		int get_n_bodies() const;

	private:
		void _before_time_step(Stark& stark);
		void _after_time_step(Stark& stark);
	};
	using spRigidBodyDynamics = std::shared_ptr<RigidBodyDynamics>;
}
