#pragma once

#include "../core/Stark.h"

#include "deformables/OneDimensionalDeformableSolids.h"
#include "deformables/SurfaceDeformableSolids.h"
#include "deformables/VolumetricDeformableSolids.h"
#include "rigidbodies/RigidBodies.h"


namespace stark::models
{
	class Simulation
	{
	public:
		core::Stark stark;
		spOneDimensionalDeformableSolids lines;
		spSurfaceDeformableSolids surfaces;
		spVolumetricDeformableSolids volumes;
		spRigidBodies rigidbodies;

		Simulation(const core::Settings& settings);
	};
}