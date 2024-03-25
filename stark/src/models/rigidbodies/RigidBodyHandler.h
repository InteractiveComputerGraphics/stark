#pragma once
#include "RigidBodyDynamics.h"
#include "EnergyRigidBodyInertia.h"

namespace stark
{
	/**
	* @brief Handler to a rigid body and its inertia.
	* 
	* The handler of the inertia energy and rigid body itself are merged into one class to simplify the API.
	* A rigid body must have exactly one inertia energy. This is enforced by the API.
	*/
	class RigidBodyHandler
	{
	private:
		int idx = -1;
		RigidBodyDynamics* rb = nullptr;
		EnergyRigidBodyInertia* inertia = nullptr;

	public:
		RigidBodyHandler() = default;
		RigidBodyHandler(RigidBodyDynamics* rb, EnergyRigidBodyInertia* inertia, int idx);

		int get_idx() const;
		bool is_valid() const;
		void exit_if_not_valid(const std::string& where_) const;

		std::string get_label() const;
		RigidBodyHandler& set_label(const std::string& label);

		Eigen::Vector3d get_translation() const;
		RigidBodyHandler& set_translation(const Eigen::Vector3d& translation);
		RigidBodyHandler& add_translation(const Eigen::Vector3d& displacement);

		Eigen::Quaterniond get_quaternion() const;
		Eigen::Matrix3d get_rotation_matrix() const;
		RigidBodyHandler& set_rotation(const Eigen::Quaterniond& q);
		RigidBodyHandler& set_rotation(double angle_deg, const Eigen::Vector3d& axis);
		RigidBodyHandler& add_rotation(const Eigen::Quaterniond& q);
		RigidBodyHandler& add_rotation(double angle_deg, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot = { 0.0, 0.0, 0.0 });

		Eigen::Vector3d get_velocity() const;
		Eigen::Vector3d get_velocity_at(const Eigen::Vector3d& x_loc) const;
		RigidBodyHandler& set_velocity(const Eigen::Vector3d& vel_glob_coords);
		RigidBodyHandler& add_velocity(const Eigen::Vector3d& vel_glob_coords);
		
		Eigen::Vector3d get_angular_velocity() const;
		RigidBodyHandler& set_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords);
		RigidBodyHandler& add_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords);

		Eigen::Vector3d get_force() const;
		RigidBodyHandler& add_force_at(const Eigen::Vector3d& force_glob_coords, const Eigen::Vector3d& application_point_glob_coords);
		RigidBodyHandler& set_force_at_centroid(const Eigen::Vector3d& force_glob_coords);
		RigidBodyHandler& add_force_at_centroid(const Eigen::Vector3d& force_glob_coords);

		Eigen::Vector3d get_torque() const;
		RigidBodyHandler& set_torque(const Eigen::Vector3d& torque_glob_coords);
		RigidBodyHandler& add_torque(const Eigen::Vector3d& torque_glob_coords);
		
		Eigen::Vector3d get_acceleration() const;
		RigidBodyHandler& set_acceleration(const Eigen::Vector3d& acc_glob_coords);
		RigidBodyHandler& add_acceleration(const Eigen::Vector3d& acc_glob_coords);

		Eigen::Vector3d get_angular_acceleration() const;
		RigidBodyHandler& set_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords);
		RigidBodyHandler& add_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords);

		Eigen::Vector3d transform_local_to_global_point(const Eigen::Vector3d& x) const;
		Eigen::Vector3d transform_local_to_global_direction(const Eigen::Vector3d& d) const;
		Eigen::Matrix3d transform_local_to_global_matrix(const Eigen::Matrix3d& A) const;
		std::vector<Eigen::Vector3d> transform_local_to_global_points(const std::vector<Eigen::Vector3d>& loc_points) const;

		Eigen::Vector3d transform_global_to_local_point(const Eigen::Vector3d& x) const;
		Eigen::Vector3d transform_global_to_local_direction(const Eigen::Vector3d& d) const;
		Eigen::Matrix3d transform_global_to_local_matrix(const Eigen::Matrix3d& A) const;
		std::vector<Eigen::Vector3d> transform_global_to_local_points(const std::vector<Eigen::Vector3d>& glob_points) const;

		// Inertia energy
		double get_mass() const;
		RigidBodyHandler& set_mass(double mass);

		Eigen::Matrix3d get_local_inertia_tensor() const;
		Eigen::Matrix3d get_global_inertia_tensor() const;
		RigidBodyHandler& set_local_inertia_tensor(const Eigen::Matrix3d& inertia_tensor);

		double get_linear_damping() const;
		RigidBodyHandler& set_linear_damping(double damping);

		double get_angular_damping() const;
		RigidBodyHandler& set_angular_damping(double damping);
	};
}
