#pragma once

#include "../solver/Stark.h"
#include "Cloth.h"
#include "RigidBodies.h"
#include "Interactions.h"

namespace stark::models
{
	class Simulation
	{
	public:
		Stark stark;
		Cloth cloth;
		RigidBodies rigid_bodies;
		Interactions interactions;

		Simulation(const Settings& settings);
	};
}