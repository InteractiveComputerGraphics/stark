#pragma once
#include <memory>

#include "../core/Stark.h"
#include "deformables/Deformables.h"
#include "rigidbodies/RigidBodies.h"


namespace stark::models
{
	class Simulation
	{
	public:
		core::Stark stark;
		std::shared_ptr<Deformables> deformables;
		std::shared_ptr<RigidBodies> rigidbodies;

		Simulation(const core::Settings& settings);
	};
}