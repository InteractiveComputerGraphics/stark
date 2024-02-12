#pragma once
#include <memory>

#include "../core/Stark.h"
#include "deformables/Deformables.h"
#include "rigidbodies/RigidBodies.h"
#include "interactions/Interactions.h"
#include "EventDrivenScript.h"


namespace stark::models
{
	class Simulation
	{
	public:
		/* Methods */
		Simulation(const core::Settings& settings);
		const double& get_time() const;
		//void run(std::function<void()> callback = nullptr);
		//void run(EventDrivenScript& script, std::function<void()> callback = nullptr);

		/* Fields */
		core::Stark stark;
		std::shared_ptr<Deformables> deformables;
		std::shared_ptr<RigidBodies> rigidbodies;
		std::shared_ptr<Interactions> interactions;

	};
}
