#pragma once
#include "Simulation.h"

namespace stark
{
	using models::Simulation;

	// Rigid bodies
	using models::RigidBodyHandler;
	using models::AnchorPointHandler;
	using models::AbsoluteDirectionLockHandler;
	using models::BallJointHandler;
	using models::RelativeDirectionLockHandler;
	using models::PointOnAxisConstraintHandler;
	using models::DampedSpringHandler;
	using models::DistanceLimitHandler;
	using models::AngleLimitHandler;
	using models::RelativeLinearVelocityMotorHandler;
	using models::RelativeAngularVelocityMotorHandler;
	using models::FixedConstraintHandler;
	using models::HingeJointHandler;
	using models::HingeJointWithLimitsHandler;
	using models::SpringWithLimitsHandler;
	using models::SliderHandler;
	using models::PrismaticSliderHandler;
	using models::MotorHandler;
	using models::PrismaticPressHandler;
}