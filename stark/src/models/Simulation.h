#pragma once

#include "../solver/Stark.h"
#include "Cloth.h"

namespace stark::models
{
	class Simulation
	{
	public:
		Stark stark;
		Cloth cloth;

		Simulation(const Settings& settings);
	};
}