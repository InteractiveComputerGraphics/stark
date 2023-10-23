#include "Simulation.h"

stark::models::Simulation::Simulation(const Settings& settings)
	: stark(settings)
{
	this->cloth.init(this->stark);
	this->deformables.init(this->stark);
	this->rigid_bodies.init(this->stark);
	this->interactions.init(this->stark, &this->cloth, &this->rigid_bodies);
}