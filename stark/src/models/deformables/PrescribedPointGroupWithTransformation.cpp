#include "PrescribedPointGroupWithTransformation.h"

stark::models::PrescribedPointGroupWithTransformation::PrescribedPointGroupWithTransformation(const spPointDynamics& dyn, const Id& id, const std::string label)
	: dyn(dyn), obj_idx(id.get_global_idx()), label(label)
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

void stark::models::PrescribedPointGroupWithTransformation::add_vertex(const int loc_idx)
{
	this->loc_indices.push_back(loc_idx);
}

void stark::models::PrescribedPointGroupWithTransformation::add_vertices_from_range(const int loc_idx_begin, const int loc_idx_end)
{
	for (int i = loc_idx_begin; i < loc_idx_end; i++) {
		this->add_vertex(i);
	}
}

void stark::models::PrescribedPointGroupWithTransformation::add_vertices_in_aabb(const Eigen::Vector3d& center, const double size, const bool at_rest_pose)
{
	this->add_vertices_in_aabb(center, { size, size, size }, at_rest_pose);
}

void stark::models::PrescribedPointGroupWithTransformation::add_vertices_in_aabb(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const bool at_rest_pose)
{
	const IntervalVector<Eigen::Vector3d> x = (at_rest_pose) ? this->dyn->X : this->dyn->x1;
	const Eigen::AlignedBox3d aabb = Eigen::AlignedBox3d(center - 0.5 * size, center + 0.5 * size);

	const int begin = this->dyn->X.get_begin(this->obj_idx);
	const int end = this->dyn->X.get_end(this->obj_idx);
	for (int i = begin; i < end; i++) {
		if (aabb.contains(x[i])) {
			this->add_vertex(i - begin);
		}
	}
}

Eigen::Vector3d stark::models::PrescribedPointGroupWithTransformation::get_transformed(const Eigen::Vector3d& rest_position, const double sim_time)
{
	const double dt = std::min(std::max(0.0, sim_time - this->t_begin), this->t_end);
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
