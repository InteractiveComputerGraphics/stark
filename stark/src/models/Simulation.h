#pragma once

#include "../solver/Stark.h"

#include "deformables/OneDimensionalDeformableSolids.h"
#include "deformables/SurfaceDeformableSolids.h"
#include "deformables/VolumetricDeformableSolids.h"
#include "RigidBodies.h"

namespace stark::models
{
	class Simulation
	{
	public:
		Stark stark;
		spOneDimensionalDeformableSolids lines;
		spSurfaceDeformableSolids surfaces;
		spVolumetricDeformableSolids volumes;
		spRigidBodies rigidbodies;

		Simulation(const Settings& settings);
	};
}