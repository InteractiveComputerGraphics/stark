#pragma once

#include "../solver/Stark.h"

#include "deformables/Shells.h"

namespace stark::models
{
	class Simulation
	{
	public:
		Stark stark;
		spShells shells;

		Simulation(const Settings& settings);
	};
}