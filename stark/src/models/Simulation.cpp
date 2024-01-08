#include "Simulation.h"


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
