#pragma once

#include "../solver/Stark.h"

#include "deformables/SurfaceDeformableSolids.h"
#include "deformables/VolumetricDeformableSolids.h"

namespace stark::models
{
	class Simulation
	{
	public:
		Stark stark;
		spSurfaceDeformableSolids surfaces;
		spVolumetricDeformableSolids volumes;

		Simulation(const Settings& settings);
	};
}