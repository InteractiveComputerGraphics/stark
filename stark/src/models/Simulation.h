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
		void add_time_event(double t0, double t1, std::function<void(double)> action);
		void add_time_event(double t0, double t1, std::function<void(double, EventInfo&)> action);
		EventDrivenScript& get_script();
		void run(std::function<void()> callback = nullptr);

		/* Fields */
		std::shared_ptr<Deformables> deformables;
		std::shared_ptr<RigidBodies> rigidbodies;
		std::shared_ptr<Interactions> interactions;

	private:
		core::Stark stark;
		EventDrivenScript script;
	};
}
