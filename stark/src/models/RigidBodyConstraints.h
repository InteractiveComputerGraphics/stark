#pragma once
#include <vector>
#include <array>

#include <Eigen/Dense>
#include <symx>

#include "deformables/Id.h"


namespace stark::models
{
	class RigidBodyConstraintBase
	{
	private:
		std::vector<bool> is_enabled;
		std::vector<double> stiffness;
	public:
		void enable(int idx, bool enable) { this->is_enabled[idx] = enable; };
		void set_stiffness(int idx, double stiffness) { this->stiffness[idx] = stiffness; };
	};


	class AnchorPoints 
		: public RigidBodyConstraintBase
	{
	friend class RigidBodies;
	private:
		std::vector<Eigen::Vector3d> loc;
		std::vector<Eigen::Vector3d> target;
		symx::LabelledConnectivity<2> conn{ { "idx", "rb" } };

	public:
		int add(const Id& id, const Eigen::Vector3d& p_glob);
	};

	class AngularVelocityMotors
	{
	friend class RigidBodies;
	private:
		std::vector<Eigen::Vector3d> loc_da;
		std::vector<double> max_torque;
		std::vector<double> target_w;
		std::vector<double> delay;
		symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
	public:
		int add(const Id& rb_a, const Id& rb_b, double target_angular_velocity, double max_torque, double delay = 0.01);
		void set_target_angular_velocity(int idx, double target_angular_velocity);
	};



	// --------------------------------------------------------------------------------------------------------------------------------------------
	struct RigidBodyConstraints
	{

	};
}
