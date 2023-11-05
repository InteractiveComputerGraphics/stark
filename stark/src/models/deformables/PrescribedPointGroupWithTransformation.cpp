#include "PrescribedPointGroupWithTransformation.h"

stark::models::PrescribedPointGroupWithTransformation::PrescribedPointGroupWithTransformation(const int obj_idx, const std::string label)
	: obj_idx(obj_idx), label(label)
{
}

void stark::models::PrescribedPointGroupWithTransformation::clear()
{
	this->loc_indices.clear();
}

void stark::models::PrescribedPointGroupWithTransformation::set_stiffness(const double stiffness)
{
	this->stiffness = stiffness;
}

void stark::models::PrescribedPointGroupWithTransformation::set_time_bounds(const double begin, const double end)
{
	this->t_begin = begin;
	this->t_end = end;
}

void stark::models::PrescribedPointGroupWithTransformation::set_linear_velocity(const Eigen::Vector3d& v)
{
	this->linear_velocity = v;
}

void stark::models::PrescribedPointGroupWithTransformation::set_angular_velocity(const Eigen::Vector3d& w, const Eigen::Vector3d& rotation_center)
{
	this->angular_velocity = w;
	this->rotation_center = rotation_center;
}

void stark::models::PrescribedPointGroupWithTransformation::add(const int loc_idx)
{
	this->loc_indices.push_back(loc_idx);
}

void stark::models::PrescribedPointGroupWithTransformation::add_from_range(const int loc_idx_begin, const int loc_idx_end)
{
	for (int i = loc_idx_begin; i < loc_idx_end; i++) {
		this->add(i);
	}
}

void stark::models::PrescribedPointGroupWithTransformation::add_from_aabb(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& center, const Eigen::Vector3d& size)
{
	const Eigen::AlignedBox3d aabb = Eigen::AlignedBox3d(center - 0.5 * size, center + 0.5 * size);
	for (int i = 0; i < (int)points.size(); i++) {
		if (aabb.contains(points[i])) {
			this->add(i);
		}
	}
}

Eigen::Vector3d stark::models::PrescribedPointGroupWithTransformation::get_transformed(const Eigen::Vector3d& rest_position, const double sim_time)
{
	const double dt = std::min(std::max(0.0, sim_time - this->t_begin), 1.0);
	Eigen::Vector3d linear_displacement = this->linear_velocity * dt;
	Eigen::AngleAxisd rotation(dt * this->angular_velocity.norm(), this->angular_velocity.normalized());
	Eigen::Vector3d angular_displacement = this->rotation_center + rotation * (rest_position - this->rotation_center) - rest_position;
	return rest_position + linear_displacement + angular_displacement;
}

int stark::models::PrescribedPointGroupWithTransformation::size() const
{
	return (int)this->loc_indices.size();
}
int stark::models::PrescribedPointGroupWithTransformation::get_point_idx(const int i) const
{
	assert(i >= 0 && i < this->size() && "PrescribedPointGroupWithTransformation::get_point_idx i out of range");
	return this->loc_indices[i];
}
double stark::models::PrescribedPointGroupWithTransformation::get_stiffness() const
{
	return this->stiffness;
}
std::string stark::models::PrescribedPointGroupWithTransformation::get_label() const
{
	return this->label;
}
int stark::models::PrescribedPointGroupWithTransformation::get_obj_idx() const
{
	return this->obj_idx;
}
