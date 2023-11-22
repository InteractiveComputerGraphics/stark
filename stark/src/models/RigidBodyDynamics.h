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
		Id add(const double mass, const Eigen::Matrix3d& inertia_loc);
		Id add_and_transform(const double mass, const Eigen::Matrix3d& inertia_loc, const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = { 0, 0, 1 });

		void set_translation(const Id& id, const Eigen::Vector3d& translation);
		void add_displacement(const Id& id, const Eigen::Vector3d& displacement);
		void set_rotation(const Id& id, const Eigen::Quaterniond& q);
		void set_rotation(const Id& id, const double& angle_deg, const Eigen::Vector3d& axis);
		void add_rotation(const Id& id, const Eigen::Quaterniond& q);
		void add_rotation(const Id& id, const double& angle_deg, const Eigen::Vector3d& axis);
		void set_velocity(const Id& id, const Eigen::Vector3d& vel_glob_coords);
		void add_velocity(const Id& id, const Eigen::Vector3d& vel_glob_coords);
		void set_angular_velocity(const Id& id, const Eigen::Vector3d& angular_vel_glob_coords);
		void add_angular_velocity(const Id& id, const Eigen::Vector3d& angular_vel_glob_coords);
		void add_force_at(const Id& id, const Eigen::Vector3d& force_glob_coords, const Eigen::Vector3d& application_point_glob_coords);
		void set_force_at_centroid(const Id& id, const Eigen::Vector3d& force_glob_coords);
		void add_force_at_centroid(const Id& id, const Eigen::Vector3d& force_glob_coords);
		void set_torque(const Id& id, const Eigen::Vector3d& torque_glob_coords);
		void add_torque(const Id& id, const Eigen::Vector3d& torque_glob_coords);
		void set_acceleration(const Id& id, const Eigen::Vector3d& acc_glob_coords);
		void add_acceleration(const Id& id, const Eigen::Vector3d& acc_glob_coords);
		void set_angular_acceleration(const Id& id, const Eigen::Vector3d& ang_acc_glob_coords);
		void add_angular_acceleration(const Id& id, const Eigen::Vector3d& ang_acc_glob_coords);

		Eigen::Vector3d get_point_in_global_coordinates(const Id& id, const Eigen::Vector3d& p);
		int get_n_bodies() const;
		bool is_empty() const;
		bool is_body_declared(const Id& id) const;

	private:
		void _before_time_step(Stark& stark);
		void _after_time_step(Stark& stark);
	};
	using spRigidBodyDynamics = std::shared_ptr<RigidBodyDynamics>;
}
