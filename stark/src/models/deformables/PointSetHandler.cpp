#include "PointSetHandler.h"

#include "PointDynamics.h"
#include "../../utils/include.h"

using namespace stark;

PointSetHandler::PointSetHandler(PointDynamics* dyn, int idx)
	: dyn(dyn), idx(idx)
{
}

int PointSetHandler::get_idx() const
{
    return this->idx;
}
bool stark::PointSetHandler::is_valid() const
{
	return this->dyn != nullptr;
}
void stark::PointSetHandler::exit_if_not_valid(const std::string& where_) const
{
	if (!this->is_valid()) {
		std::cout << "stark error: Invalid handler found in " << where_ << std::endl;
		exit(-1);
	}
}
std::string stark::PointSetHandler::get_label() const
{
	return this->dyn->labels[this->get_idx()];
}
PointSetHandler& stark::PointSetHandler::set_label(const std::string& label)
{
	this->dyn->labels[this->get_idx()] = label;
	return *this;
}
int PointSetHandler::get_begin() const
{
	return this->dyn->get_begin(this->get_idx());
}
int PointSetHandler::get_end() const
{
	return this->dyn->get_end(this->get_idx());
}
int PointSetHandler::size() const
{
	return this->dyn->get_set_size(this->get_idx());
}
int PointSetHandler::get_global_index(int local_index) const
{
	return this->dyn->get_global_index(this->get_idx(), local_index);
}
std::vector<int> stark::PointSetHandler::all() const
{
	std::vector<int> indices(this->size());
	std::iota(indices.begin(), indices.end(), 0);
	return indices;
}

Eigen::Vector3d PointSetHandler::get_position(int local_index) const
{
	return this->dyn->x1[this->get_global_index(local_index)];
}
Eigen::Vector3d PointSetHandler::get_rest_position(int local_index) const
{
	return this->dyn->X[this->get_global_index(local_index)];
}
Eigen::Vector3d PointSetHandler::get_velocity(int local_index) const
{
	return this->dyn->v1[this->get_global_index(local_index)];
}
Eigen::Vector3d PointSetHandler::get_acceleration(int local_index) const
{
	return this->dyn->a[this->get_global_index(local_index)];
}
Eigen::Vector3d PointSetHandler::get_force(int local_index) const
{
	return this->dyn->f[this->get_global_index(local_index)];
}

void PointSetHandler::set_position(int local_index, const Eigen::Vector3d& v)
{
	this->dyn->x1[this->get_global_index(local_index)] = v;
	this->dyn->x0[this->get_global_index(local_index)] = v;
}
void PointSetHandler::set_rest_position(int local_index, const Eigen::Vector3d& v)
{
	this->dyn->X[this->get_global_index(local_index)] = v;
}
void PointSetHandler::set_velocity(int local_index, const Eigen::Vector3d& v)
{
	this->dyn->v1[this->get_global_index(local_index)] = v;
	this->dyn->v0[this->get_global_index(local_index)] = v;
}
void PointSetHandler::set_acceleration(int local_index, const Eigen::Vector3d& v)
{
	this->dyn->a[this->get_global_index(local_index)] = v;
}
void PointSetHandler::set_force(int local_index, const Eigen::Vector3d& v)
{
	this->dyn->f[this->get_global_index(local_index)] = v;
}

PointSetHandler& PointSetHandler::add_displacement(const Eigen::Vector3d& displacement, bool also_at_rest_pose)
{
	for (int i = this->get_begin(); i < this->get_end(); i++)
	{
		this->dyn->x1[i] += displacement;
		this->dyn->x0[i] += displacement;
		if (also_at_rest_pose) {
			this->dyn->X[i] += displacement;
		}
	}

	return *this;
}
PointSetHandler& PointSetHandler::add_rotation(const double angle_deg, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot, bool also_at_rest_pose)
{
	const Eigen::Matrix3d R = Eigen::AngleAxis<double>(deg2rad(angle_deg), axis.normalized()).toRotationMatrix();
	for (int i = this->get_begin(); i < this->get_end(); i++) {
		this->dyn->x0[i] = rotate_deg(this->dyn->x0[i], R, pivot);
		this->dyn->x1[i] = this->dyn->x0[i];
		if (also_at_rest_pose) {
			this->dyn->X[i] = this->dyn->x0[i];
		}
	}

	return *this;
}
