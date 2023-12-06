#pragma once
#include <array>
#include <vector>
#include <memory>

#include <symx>

#include "RigidBodyDynamics.h"
#include "BaseRigidBodyConstraints.h"

namespace stark::models
{
	class EnergyRigidBodyConstraints
	{
	public:
        EnergyRigidBodyConstraints(core::Stark& stark, spRigidBodyDynamics dyn);

        /* Fields */
        spRigidBodyDynamics dyn;
		double stiffness_hard_multiplier = 2.0;
		double stiffness_soft_multiplier = 1.05;
		double soft_constraint_capacity_hardening_point = 0.75;

		// Constraint containers
		std::shared_ptr<BaseRigidBodyConstraints::AnchorPoints> anchor_points;
		std::shared_ptr<BaseRigidBodyConstraints::AbsoluteDirectionLocks> absolute_direction_locks;
		std::shared_ptr<BaseRigidBodyConstraints::BallJoints> ball_joints;
		std::shared_ptr<BaseRigidBodyConstraints::RelativeDirectionLocks> relative_direction_locks;
		std::shared_ptr<BaseRigidBodyConstraints::PointOnAxis> point_on_axis;
		std::shared_ptr<BaseRigidBodyConstraints::DampedSprings> damped_springs;
		std::shared_ptr<BaseRigidBodyConstraints::DistanceLimits> distance_limits;
		std::shared_ptr<BaseRigidBodyConstraints::AngleLimits> angle_limits;
		std::shared_ptr<BaseRigidBodyConstraints::RelativeLinearVelocityMotors> relative_linear_velocity_motors;
		std::shared_ptr<BaseRigidBodyConstraints::RelativeAngularVelocityMotors> relative_angular_velocity_motors;

		// Output
		core::Logger logger;

	private:
        /* Methods */
        void _set_c1_controller_energy(symx::Energy& energy, const symx::Scalar& v, const symx::Scalar& target_v, const symx::Scalar& max_force, const symx::Scalar& delay, const symx::Scalar& dt, const symx::Scalar& is_active);
		bool _adjust_constraints_stiffness(core::Stark& stark, double cap, double multiplier, bool log);

		// SymX callbacks
		bool _is_converged_state_valid(core::Stark& stark);
		void _on_time_step_accepted(core::Stark& stark);
		void _write_frame(core::Stark& stark);
	};
    using spEnergyRigidBodyConstraints = std::shared_ptr<EnergyRigidBodyConstraints>;
}
