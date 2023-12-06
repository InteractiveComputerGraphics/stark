#include "PointDynamics.h"

#include "../time_integration.h"

stark::models::PointDynamics::PointDynamics(stark::core::Stark& stark)
{
	this->dof = stark.global_energy.add_dof_array(this->v1.data, "PointDynamics.v1");
	stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step(stark); });
	stark.callbacks.on_time_step_accepted.push_back([&]() { this->_on_time_step_accepted(stark); });
}

stark::models::Id stark::models::PointDynamics::add(const std::vector<Eigen::Vector3d>& x, const std::vector<Eigen::Vector3d>& v)
{
	std::vector<Eigen::Vector3d> zero(x.size(), Eigen::Vector3d::Zero());
	const std::vector<Eigen::Vector3d>& v_ = (v.size() != 0) ? v : zero;
	
	if (x.size() != v_.size()) {
		std::cout << "stark error: PointDynamics::add() x and v must be of same length." << std::endl;
		exit(-1);
	}

	const int glob_idx = this->X.append(x);
	this->x0.append(x);
	this->x1.append(x);
	this->v0.append(v_);
	this->v1.append(v_);
	this->a.append(zero);
	return Id(PhysicalSystem::Deformables, glob_idx);
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

void stark::models::PointDynamics::_before_time_step(stark::core::Stark& stark)
{
	// Set next time velocities estimation to zero to avoid invalid state outside of the minimzer
	std::fill(this->v1.data.begin(), this->v1.data.end(), Eigen::Vector3d::Zero());
}

void stark::models::PointDynamics::_on_time_step_accepted(stark::core::Stark& stark)
{
	// Set final positions with solved velocities
	const double dt = stark.settings.simulation.adaptive_time_step.value;
	for (int i = 0; i < this->size(); i++) {
		this->x1[i] = time_integration(this->x0[i], this->v1[i], dt);
	}

	// x0 <- x1
	this->x0 = this->x1;
	this->v0 = this->v1;
}
