#pragma once
#include <memory>

#include "../core/Stark.h"
#include "deformables/Deformables.h"
#include "rigidbodies/RigidBodies.h"
#include "interactions/Interactions.h"
#include "presets/Presets.h"


namespace stark
{
	class Simulation
	{
	public:
		/* Methods */
		Simulation(const core::Settings& settings);
		double get_time() const;
		double get_time_step_size() const;
		int get_frame() const;
		Eigen::Vector3d get_gravity() const;
		void set_gravity(const Eigen::Vector3d& gravity);
		core::Logger& get_logger();
		core::Console& get_console();
		const core::Settings& get_settings() const;
		EventDrivenScript& get_script();

		void add_time_event(double t0, double t1, std::function<void(double)> action);
		void add_time_event(double t0, double t1, std::function<void(double, EventInfo&)> action);
		void run(std::function<void()> callback = nullptr);
		void run(double duration, std::function<void()> callback = nullptr);
		void run_one_time_step();

		/* Fields */
		std::shared_ptr<Deformables> deformables;
		std::shared_ptr<RigidBodies> rigidbodies;
		std::shared_ptr<Interactions> interactions;
		std::shared_ptr<Presets> presets;

	private:
		core::Stark stark;
	};
}
