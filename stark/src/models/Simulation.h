#pragma once

#include "../solver/Stark.h"
#include "Cloth.h"
#include "RigidBodies.h"
#include "DeformableSolids.h"
#include "Interactions.h"

namespace stark::models
{
	class Simulation
	{
	public:
		Stark stark;
		Cloth cloth;
		DeformableSolids deformables;
		RigidBodies rigid_bodies;
		Interactions interactions;

		Simulation(const Settings& settings);
	};
}