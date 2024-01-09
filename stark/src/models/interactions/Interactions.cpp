#include "Interactions.h"

using namespace stark::models;

stark::models::Interactions::Interactions(core::Stark& stark, spEnergyFrictionalContact contact)
	: contact(contact)
{
}

void stark::models::Interactions::set_friction(const int global_idx0, const int global_idx1, double coulombs_mu)
{
	this->contact->set_coulomb_friction_pair(global_idx0, global_idx1, coulombs_mu);
}

void stark::models::Interactions::disable_collision(const int global_idx0, const int global_idx1)
{
	this->contact->disable_collision(global_idx0, global_idx1);
}
