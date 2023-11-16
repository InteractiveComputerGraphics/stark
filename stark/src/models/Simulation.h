#pragma once

#include "../solver/Stark.h"
#include "Cloth.h"
#include "RigidBodies.h"
#include "DeformableSolids.h"
#include "Interactions.h"

#include "deformables/Shells.h"

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

		spShells shells;

		Simulation(const Settings& settings);
	};
}