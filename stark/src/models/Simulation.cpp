#include "Simulation.h"


const double& stark::models::Simulation::get_time() const
{
	return this->stark.current_time;
}

const double& stark::models::Simulation::get_time_step_size() const
{
	return this->stark.settings.simulation.adaptive_time_step.value;
}

stark::core::Logger& stark::models::Simulation::get_logger()
{
	return this->stark.logger;
}

stark::core::Console& stark::models::Simulation::get_console()
{
	return this->stark.console;
}

void stark::models::Simulation::add_time_event(double t0, double t1, std::function<void(double)> action)
{
	this->add_time_event(t0, t1, [action](double t, EventInfo& event_info) { action(t); });
}
void stark::models::Simulation::add_time_event(double t0, double t1, std::function<void(double, EventInfo&)> action)
{
	this->script.add_independent_event(
		[t0, t1, this](EventInfo& event_info) { return this->get_time() >= t0 && this->get_time() < t1; },
		[action, this](EventInfo& event_info) { action(this->get_time(), event_info); },
		stark::models::EventDrivenScript::Permanence::PERMANENT,
		[t1, this](EventInfo& event_info) { return this->get_time() >= t1; }
	);
}

stark::models::EventDrivenScript& stark::models::Simulation::get_script()
{
	return this->script;
}

void stark::models::Simulation::run(std::function<void()> user_callback)
{
	this->stark.run(
		[user_callback, this]()
		{
			this->script.run_a_cycle(this->get_time());
			if (user_callback != nullptr) user_callback();
		}
	);
}

stark::models::Simulation::Simulation(const core::Settings& settings)
	: stark(settings)
{
	// Base dynamics
	spPointDynamics point_dynamics = std::make_shared<PointDynamics>(this->stark);
	spRigidBodyDynamics rb_dynamics = std::make_shared<RigidBodyDynamics>(this->stark);

	// Common energies
	spEnergyFrictionalContact contact = std::make_shared<EnergyFrictionalContact>(this->stark, point_dynamics, rb_dynamics);

	// Physical Systems
	this->deformables = std::make_shared<Deformables>(this->stark, point_dynamics, contact);
	this->rigidbodies = std::make_shared<RigidBodies>(this->stark, rb_dynamics, contact);

	// Interactions
	this->interactions = std::make_shared<Interactions>(this->stark, contact);
}
