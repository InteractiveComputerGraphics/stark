#include "Simulation.h"


double stark::Simulation::get_time() const
{
	return this->stark.current_time;
}

double stark::Simulation::get_time_step_size() const
{
	return this->stark.dt;
}

int stark::Simulation::get_frame() const
{
	return this->stark.current_frame;
}

Eigen::Vector3d stark::Simulation::get_gravity() const
{
	return this->stark.gravity;
}

void stark::Simulation::set_gravity(const Eigen::Vector3d& gravity)
{
	this->stark.gravity = gravity;
}

stark::core::Logger& stark::Simulation::get_logger()
{
	return this->stark.logger;
}

stark::core::Console& stark::Simulation::get_console()
{
	return this->stark.console;
}

const stark::core::Settings& stark::Simulation::get_settings() const
{
	return this->stark.settings;
}

void stark::Simulation::add_time_event(double t0, double t1, std::function<void(double)> action)
{
	this->add_time_event(t0, t1, [action](double t, EventInfo& event_info) { action(t); });
}
void stark::Simulation::add_time_event(double t0, double t1, std::function<void(double, EventInfo&)> action)
{
	this->stark.script.add_event(
		/* action = */ [action, this](EventInfo& event_info) { action(this->get_time(), event_info); },
		/* run_when = */ [t0, t1, this](EventInfo& event_info) { return this->get_time() >= t0 && this->get_time() < t1; },
		/* delete_when = */ [t1, this](EventInfo& event_info) { return this->get_time() >= t1; }
	);
}

void stark::Simulation::run(std::function<void()> callback)
{
	this->run(std::numeric_limits<double>::max(), callback);
}

stark::EventDrivenScript& stark::Simulation::get_script()
{
	return this->stark.script;
}

void stark::Simulation::run(double duration, std::function<void()> user_callback)
{
	this->stark.run(duration, 
		[user_callback, this]()
		{
			this->stark.script.run_a_cycle(this->get_time());
			if (user_callback != nullptr) user_callback();
		}
	);
}

void stark::Simulation::run_one_time_step()
{
	this->stark.script.run_a_cycle(this->get_time());
	this->stark.run_one_step();
}

stark::Simulation::Simulation(const core::Settings& settings)
	: stark(settings)
{
	// Base dynamics
	spPointDynamics point_dynamics = std::make_shared<PointDynamics>(this->stark);
	spRigidBodyDynamics rb_dynamics = std::make_shared<RigidBodyDynamics>(this->stark);

	// Physical Systems
	this->deformables = std::make_shared<Deformables>(this->stark, point_dynamics);
	this->rigidbodies = std::make_shared<RigidBodies>(this->stark, rb_dynamics);

	// Interactions
	this->interactions = std::make_shared<Interactions>(this->stark, point_dynamics, rb_dynamics);

	// Presets
	this->presets = std::make_shared<Presets>(this->stark, this->deformables, this->rigidbodies, this->interactions);
}
