#pragma once
#include <memory>

#include "../core/Stark.h"

#include "deformables/DeformableSolidsLines.h"
#include "deformables/DeformableSolidsSurfaces.h"
#include "deformables/DeformableSolidsVolumes.h"
#include "rigidbodies/RigidBodies.h"


namespace stark::models
{
	class Simulation
	{
	public:
		core::Stark stark;
		std::shared_ptr<DeformableSolidsLines> lines;
		std::shared_ptr<DeformableSolidsSurfaces> surfaces;
		std::shared_ptr<DeformableSolidsVolumes> volumes;
		std::shared_ptr<RigidBodies> rigidbodies;

		Simulation(const core::Settings& settings);
	};
}