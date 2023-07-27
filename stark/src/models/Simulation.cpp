#include "Simulation.h"

stark::models::Simulation::Simulation(const Settings& settings)
	: stark(settings)
{
	this->cloth.init(this->stark);
}