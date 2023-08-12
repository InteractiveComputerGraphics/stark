#pragma once

#include "../solver/Stark.h"
#include "Cloth.h"
#include "RigidBodies.h"

namespace stark::models
{
	class Simulation
	{
	public:
		Stark stark;
		Cloth cloth;
		RigidBodies rigid_bodies;

		Simulation(const Settings& settings);
	};
}