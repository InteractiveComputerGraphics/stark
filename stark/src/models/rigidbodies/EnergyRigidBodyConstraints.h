#pragma once
#include <array>
#include <vector>
#include <memory>

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

        /* Methods */
        void _set_c1_controller_energy(symx::Energy& energy, const symx::Scalar& v, const symx::Scalar& target_v, const symx::Scalar& max_force, const symx::Scalar& delay, const symx::Scalar& dt, const symx::Scalar& is_active);

        symx::Vector _get_x1(symx::Energy& energy, const stark::core::Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc);
        symx::Vector _get_d1(symx::Energy& energy, const stark::core::Stark& stark, const symx::Index& rb_idx, const symx::Vector& d_loc);
        std::array<symx::Vector, 2> _get_x1_d1(symx::Energy& energy, const stark::core::Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Vector& d_loc);
        std::array<symx::Vector, 2> _get_x0_x1(symx::Energy& energy, const stark::core::Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc);

        Eigen::Vector3d _get_x1(int rb_idx, const Eigen::Vector3d& x_loc, double dt);
        Eigen::Vector3d _get_d1(int rb_idx, const Eigen::Vector3d& d_loc, double dt);

	private:
		bool _is_converged_state_valid(core::Stark& stark);
		void _after_time_step(core::Stark& stark);
		bool _adjust_constraints_stiffness(core::Stark& stark, double cap, double multiplier);
	};
    using spEnergyRigidBodyConstraints = std::shared_ptr<EnergyRigidBodyConstraints>;
}
