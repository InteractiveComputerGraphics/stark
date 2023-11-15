#include "PointDynamics.h"

stark::models::PointDynamics::PointDynamics(Stark& stark)
{
	this->dof = stark.global_energy.add_dof_array(this->v1.data, "PointDynamics.v1");
	stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step(stark); });
	stark.callbacks.after_time_step.push_back([&]() { this->_after_time_step(stark); });
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

void stark::models::PointDynamics::_before_time_step(Stark& stark)
{
	// Set next time velocities estimation to zero to avoid invalid state outside of the minimzer
	std::fill(this->v1.data.begin(), this->v1.data.end(), Eigen::Vector3d::Zero());
}

void stark::models::PointDynamics::_after_time_step(Stark& stark)
{
	// Set final positions with solved velocities
	const double dt = stark.settings.simulation.adaptive_time_step.value;
	for (int i = 0; i < this->size(); i++) {
		this->x1[i] = this->x0[i] + dt*this->v1[i];
	}

	// x0 <- x1
	this->x0 = this->x1;
	this->v0 = this->v1;
}
