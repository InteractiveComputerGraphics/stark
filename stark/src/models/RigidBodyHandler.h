#pragma once

#include "RigidBodiesData.h"

namespace stark::models
{
	class RigidBodyHandler
	{
	private:
		spRigidBodiesData data;
		int idx;

	public:
		RigidBodyHandler(spRigidBodiesData data, int idx);

		int index() const;

		double get_mass() const;
		void set_mass();

		Eigen::Matrix3d get_inertia_tensor() const;
		void set_inertia_tensor(const Eigen::Vector3d& inertia_tensor);

		Eigen::Vector3d get_translation() const;
		void set_translation(const Eigen::Vector3d& translation);
		void add_displacement(const Eigen::Vector3d& displacement);

		Eigen::Vector3d get_rotation() const;
		void set_rotation(const Eigen::Quaterniond& q);
		void set_rotation(const double& angle_deg, const Eigen::Vector3d& axis);
		void add_rotation(const Eigen::Quaterniond& q);
		void add_rotation(const double& angle_deg, const Eigen::Vector3d& axis);

		Eigen::Vector3d get_velocity() const;
		void set_velocity(const Eigen::Vector3d& vel_glob_coords);
		void add_velocity(const Eigen::Vector3d& vel_glob_coords);
		
		Eigen::Vector3d get_angular_velocity() const;
		void set_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords);
		void add_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords);

		Eigen::Vector3d get_force() const;
		void add_force_at(const Eigen::Vector3d& force_glob_coords, const Eigen::Vector3d& application_point_glob_coords);
		void set_force_at_centroid(const Eigen::Vector3d& force_glob_coords);
		void add_force_at_centroid(const Eigen::Vector3d& force_glob_coords);

		Eigen::Vector3d get_torque() const;
		void set_torque(const Eigen::Vector3d& torque_glob_coords);
		void add_torque(const Eigen::Vector3d& torque_glob_coords);
		
		Eigen::Vector3d get_acceleration() const;
		void set_acceleration(const Eigen::Vector3d& acc_glob_coords);
		void add_acceleration(const Eigen::Vector3d& acc_glob_coords);

		Eigen::Vector3d get_angular_acceleration() const;
		void set_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords);
		void add_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords);

		Eigen::Vector3d local_to_global_point(const Eigen::Vector3d& x);
		Eigen::Vector3d local_to_global_direction(const Eigen::Vector3d& x);
		Eigen::Matrix3d local_to_global_matrix(const Eigen::Matrix3d& A);

		Eigen::Vector3d global_to_local_point(const Eigen::Vector3d& x);
		Eigen::Vector3d global_to_local_direction(const Eigen::Vector3d& x);
		Eigen::Matrix3d global_to_local_matrix(const Eigen::Matrix3d& A);

		void add_to_output_label(const std::string label, Id& id);
		void set_collision_mesh(const Id& id, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles);
		void set_render_mesh(const Id& id, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles);
	};
}