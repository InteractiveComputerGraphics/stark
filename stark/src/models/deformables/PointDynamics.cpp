#include "PointDynamics.h"

stark::models::PointDynamics::PointDynamics(Stark& sim)
{
	this->dof = sim.global_energy.add_dof_array(this->v1.data, "PointDynamics.v1");
}

int stark::models::PointDynamics::get_begin(const Id& id) const
{
	assert(id.get_physical_system() == PhysicalSystem::Deformables && "PointDynamics only works with Ids of type 'Deformables'");
	return this->X.get_begin(id.get_global_idx());
}

int stark::models::PointDynamics::size(const Id& id) const
{
	assert(id.get_physical_system() == PhysicalSystem::Deformables && "PointDynamics only works with Ids of type 'Deformables'");
	return this->X.get_set_size(id.get_global_idx());
}

int stark::models::PointDynamics::size() const
{
	return this->X.size();
}
