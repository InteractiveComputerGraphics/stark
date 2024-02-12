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
		const double& get_time_step_size() const;
		core::Logger& get_logger();
		core::Console& get_console();
		void run(std::function<void()> callback = nullptr);

		/* Fields */
		std::shared_ptr<Deformables> deformables;
		std::shared_ptr<RigidBodies> rigidbodies;
		std::shared_ptr<Interactions> interactions;
		EventDrivenScript script;

	private:
		core::Stark stark;
	};
}
