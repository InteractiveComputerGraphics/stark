#include "DeformableHandler.h"

using namespace stark::models;

DeformableHandler::DeformableHandler(const Id& id, spPointDynamics dyn, spEnergyPointInertia inertia, spEnergyPointPrescribedPositions prescribed_positions)
: id(id), dyn(dyn), inertia(inertia), prescribed_positions(prescribed_positions)
{
}

int DeformableHandler::get_deformable_idx() const
{
	return this->id.get_global_idx();
}
int DeformableHandler::get_global_vertex_idx(const int local_vertex) const
{
	return this->dyn->get_begin(this->id) + local_vertex;
}
const std::unordered_map<std::string, int>& DeformableHandler::get_local_indices() const
{
	return this->id.local_indices;
}
void DeformableHandler::set_velocity(const int local_vertex, const Eigen::Vector3d& v)
{
	const int global_idx = this->get_global_vertex_idx(local_vertex);
	this->dyn->v0[global_idx] = v;
	this->dyn->v1[global_idx] = v;
}
void DeformableHandler::set_position(const int local_vertex, const Eigen::Vector3d& x)
{
	const int global_idx = this->get_global_vertex_idx(local_vertex);
	this->dyn->x0[global_idx] = x;
	this->dyn->x1[global_idx] = x;
}
void DeformableHandler::set_acceleration(const int local_vertex, const Eigen::Vector3d& a)
{
	const int global_idx = this->get_global_vertex_idx(local_vertex);
	this->dyn->a[global_idx] = a;
}
void DeformableHandler::set_force(const int local_vertex, const Eigen::Vector3d& f)
{
	const int global_idx = this->get_global_vertex_idx(local_vertex);
	this->dyn->f[global_idx] = f;
}
void DeformableHandler::clear_forces()
{
	const int set_id = this->get_deformable_idx();
	std::fill(this->dyn->f.get_begin_ptr(set_id), this->dyn->f.get_end_ptr(set_id), Eigen::Vector3d::Zero());
}
void DeformableHandler::clear_accelerations()
{
	const int set_id = this->get_deformable_idx();
	std::fill(this->dyn->f.get_begin_ptr(set_id), this->dyn->f.get_end_ptr(set_id), Eigen::Vector3d::Zero());
}
Eigen::Vector3d DeformableHandler::get_velocity(const int local_vertex)
{
	return this->dyn->v1[this->get_global_vertex_idx(local_vertex)];
}
Eigen::Vector3d DeformableHandler::get_position(const int local_vertex)
{
	return this->dyn->x1[this->get_global_vertex_idx(local_vertex)];
}
Eigen::Vector3d DeformableHandler::get_acceleration(const int local_vertex)
{
	return this->dyn->a[this->get_global_vertex_idx(local_vertex)];
}
Eigen::Vector3d stark::models::DeformableHandler::get_force(const int local_vertex)
{
	return this->dyn->f[this->get_global_vertex_idx(local_vertex)];
}

std::shared_ptr<PrescribedPointGroup> DeformableHandler::create_prescribed_positions_group(const std::string label)
{
	return this->prescribed_positions->create_group(id, label);
}
std::shared_ptr<PrescribedPointGroupWithTransformation> DeformableHandler::create_prescribed_positions_group_with_transformation(const std::string label)
{
	return this->prescribed_positions->create_group_with_transformation(id, label);
}

const Id& stark::models::DeformableHandler::get_id() const
{
	return this->id;
}
