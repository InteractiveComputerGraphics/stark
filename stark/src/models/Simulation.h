#pragma once

#include "../solver/Stark.h"

#include "deformables/OneDimensionalDeformableSolids.h"
#include "deformables/SurfaceDeformableSolids.h"
#include "deformables/VolumetricDeformableSolids.h"

namespace stark::models
{
	class Simulation
	{
	public:
		Stark stark;
		spOneDimensionalDeformableSolids lines;
		spSurfaceDeformableSolids surfaces;
		spVolumetricDeformableSolids volumes;

		Simulation(const Settings& settings);
	};
}