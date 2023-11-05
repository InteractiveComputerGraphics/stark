#include "PrescribedPointGroup.h"

#include <cassert>

stark::models::PrescribedPointGroup::PrescribedPointGroup(const int obj_idx, const std::string label)
	: obj_idx(obj_idx), label(label)
{
}
void stark::models::PrescribedPointGroup::clear()
{
	this->loc_indices.clear();
	this->target_positions.clear();
}
void stark::models::PrescribedPointGroup::set_stiffness(const double stiffness)
{
	this->stiffness = stiffness;
}
void stark::models::PrescribedPointGroup::add(const int loc_idx, const Eigen::Vector3d& target_position)
{
	this->loc_indices.push_back(loc_idx);
	this->target_positions.push_back(target_position);
}
void stark::models::PrescribedPointGroup::set_target_position(const int i, const Eigen::Vector3d& target_position)
{
	assert(i >= 0 && i < this->size() && "PrescribedPointGroup::set_target_position i out of range");
	this->target_positions[i] = target_position;
}
const Eigen::Vector3d& stark::models::PrescribedPointGroup::get_target_position(const int i) const
{
	assert(i >= 0 && i < this->size() && "PrescribedPointGroup::get_target_position i out of range");
	return this->target_positions[i];
}
int stark::models::PrescribedPointGroup::size() const
{
	return (int)this->loc_indices.size();
}
int stark::models::PrescribedPointGroup::get_point_idx(const int i) const
{
	assert(i >= 0 && i < this->size() && "PrescribedPointGroup::get_point_idx i out of range");
	return this->loc_indices[i];
}
double stark::models::PrescribedPointGroup::get_stiffness() const
{
	return this->stiffness;
}
std::string stark::models::PrescribedPointGroup::get_label() const
{
	return this->label;
}
int stark::models::PrescribedPointGroup::get_obj_idx() const
{
	return this->obj_idx;
}
