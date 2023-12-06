#pragma once
#include <string>
#include <vector>
#include <array>
#include <memory>

#include <Eigen/Dense>
#include <symx>

#include "../../core/Logger.h"
#include "RigidBodyDynamics.h"

namespace stark::models
{
	/*
	*	Disables absolute displacement of a point on an object.
	*/
	class EnergyRigidBodyConstraint_AnchorPoints
	{
	public:
		/* Fields */
		const spRigidBodyDynamics dyn;
		std::shared_ptr<core::Logger> rb_logger;
		const RigidBodyCommonConstraintParameters common_params;
		symx::LabelledConnectivity<2> conn{ { "idx", "rb" } };
		std::vector<Eigen::Vector3d> loc;
		std::vector<Eigen::Vector3d> target_glob;
		std::vector<double> stiffness;
		std::vector<double> tolerance;
		std::vector<double> is_active;
		std::vector<std::string> labels;

		/* Methods */
		EnergyRigidBodyConstraint_AnchorPoints(core::Stark& stark, const spRigidBodyDynamics dyn, const RigidBodyCommonConstraintParameters& params, std::shared_ptr<core::Logger> rb_logger);
		int add(int rb, const Eigen::Vector3d& loc, const Eigen::Vector3d& target_glob, double stiffness, double tolerance);
		std::array<double, 2> get_violation_and_force(int idx);

	private:
		bool _adjust_constraints_stiffness_and_log(core::Stark& stark, double cap, double multiplier, bool log);

		// SymX callbacks
		bool _is_converged_state_valid(core::Stark& stark);
		void _on_time_step_accepted(core::Stark& stark);
	};
}