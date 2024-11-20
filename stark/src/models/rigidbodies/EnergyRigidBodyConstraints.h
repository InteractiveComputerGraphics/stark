#pragma once
#include <array>
#include <vector>
#include <memory>

#include <symx>

#include "RigidBodyDynamics.h"
#include "RigidBodyConstraints.h"

namespace stark
{
	/*
		All constraints
		---------------

		Absolute
			absolute_point
			absolute_direction

		Relative
			Hard Constraints
				Positional
					point (ball_joint)
					point_on_axes
					distance
					distance_limits

				Directional
					direction
					angle_limits

			Soft Constraints
				spring_linear
				spring_angular
				motor_linear_velocity
				motor_angular_velocity
	*/

	class EnergyRigidBodyConstraints
	{
	public:
        EnergyRigidBodyConstraints(core::Stark& stark, const spRigidBodyDynamics rb);

        /* Fields */
        const spRigidBodyDynamics rb;
		double stiffness_hard_multiplier = 2.0;
		double stiffness_soft_multiplier = 1.05;
		double soft_constraint_capacity_hardening_point = 0.75;

		// Constraint containers
		std::shared_ptr<RigidBodyConstraints::GlobalPoints> global_points;
		std::shared_ptr<RigidBodyConstraints::GlobalDirections> global_directions;
		std::shared_ptr<RigidBodyConstraints::Points> points;
		std::shared_ptr<RigidBodyConstraints::PointOnAxes> point_on_axes;
		std::shared_ptr<RigidBodyConstraints::Distance> distances;
		std::shared_ptr<RigidBodyConstraints::DistanceLimits> distance_limits;
		std::shared_ptr<RigidBodyConstraints::Directions> directions;
		std::shared_ptr<RigidBodyConstraints::AngleLimits> angle_limits;
		std::shared_ptr<RigidBodyConstraints::DampedSprings> damped_springs;
		std::shared_ptr<RigidBodyConstraints::LinearVelocity> linear_velocity;
		std::shared_ptr<RigidBodyConstraints::AngularVelocity> angular_velocity;

	private:
        /* Methods */
		bool _adjust_constraints_stiffness_and_log(core::Stark& stark, double cap, double multiplier, bool are_positions_set);

		// SymX callbacks
		bool _is_converged_state_valid(core::Stark& stark);
		void _on_time_step_accepted(core::Stark& stark);
	};
}
