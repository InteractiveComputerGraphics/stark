#pragma once
#include "Simulation.h"

namespace stark
{
	using models::Simulation;

	// Rigid bodies
	using models::RigidBodyHandler;
	using models::RBCGlobalPointHandler;
	using models::RBCGlobalDirectionHandler;
	using models::RBCPointHandler;
	using models::RBCPointOnAxisHandler;
	using models::RBCDistanceHandler;
	using models::RBCDistanceLimitHandler;
	using models::RBCDirectionHandler;
	using models::RBCAngleLimitHandler;
	using models::RBCDampedSpringHandler;
	using models::RBCLinearVelocityHandler;
	using models::RBCAngularVelocityHandler;

	using models::RBCFixHandler;
	using models::RBCAttachmentHandler;
	using models::RBCPointWithAngleLimitHandler;
	using models::RBCHingeJointHandler;
	using models::RBCHingeJointWithAngleLimitHandler;
	using models::RBCSpringWithLimitsHandler;
	using models::RBCSliderHandler;
	using models::RBCPrismaticSliderHandler;
	using models::RBCPrismaticPressHandler;
	using models::RBCMotorHandler;
}
