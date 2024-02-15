#pragma once
#include "Simulation.h"

namespace stark
{
	// General
	using models::Simulation;
	using models::EventDrivenScript;
	using models::EventInfo;

	// Deformables
	using models::Deformables;
	using models::DeformableSurfaceHandler;
	using models::DeformableVolumeHandler;
	//using models::MaterialLine;
	using models::MaterialSurface;
	using models::MaterialVolume;

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

	// Interactions
	using models::Interactions;
	using models::StaticPlaneHandler;
}
