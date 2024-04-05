#include "RigidBodyHandler.h"

#include "RigidBodyDynamics.h"
#include "rigidbody_transformations.h"
#include "../../utils/include.h"

stark::RigidBodyHandler::RigidBodyHandler(RigidBodyDynamics* rb, EnergyRigidBodyInertia* inertia, int idx)
	: rb(rb), idx(idx), inertia(inertia)
{
}
int stark::RigidBodyHandler::get_idx() const
{
	return this->idx;
}
bool stark::RigidBodyHandler::is_valid() const
{
	return this->rb != nullptr;
}
void stark::RigidBodyHandler::exit_if_not_valid(const std::string& where_) const
{
	if (!this->is_valid()) {
		std::cout << "Stark error: " << where_ << " found an invalid handler." << std::endl;
		exit(-1);
	}
}
std::string stark::RigidBodyHandler::get_label() const
{
	return this->rb->labels[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_label(const std::string& label)
{
	this->rb->labels[this->idx] = label;
	return (*this);
}
Eigen::Vector3d stark::RigidBodyHandler::get_translation() const
{
	return this->rb->t1[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_translation(const Eigen::Vector3d& t)
{
	this->rb->t1[this->idx] = t;
	this->rb->t0[this->idx] = t;
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_translation(const Eigen::Vector3d& t)
{
	this->rb->t1[this->idx] += t;
	this->rb->t0[this->idx] += t;
	return (*this);
}
Eigen::Quaterniond stark::RigidBodyHandler::get_quaternion() const
{
	return this->rb->q1[this->idx];
}
Eigen::Matrix3d stark::RigidBodyHandler::get_rotation_matrix() const
{
	return this->rb->R1[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_rotation(const Eigen::Quaterniond& q)
{
	this->rb->q0[this->idx] = q.normalized();
	this->rb->q1[this->idx] = q.normalized();
	this->rb->R0[this->idx] = this->rb->q0[this->idx].toRotationMatrix();
	this->rb->R1[this->idx] = this->rb->q1[this->idx].toRotationMatrix();
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_rotation(double angle_deg, const Eigen::Vector3d& axis)
{
	this->set_rotation(Eigen::Quaterniond(Eigen::AngleAxis<double>(deg2rad(angle_deg), axis.normalized())));
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_rotation(const Eigen::Quaterniond& q)
{
	const Eigen::Quaterniond q_ = q.normalized();
	const Eigen::Matrix3d R = q_.toRotationMatrix();
	this->rb->t0[this->idx] = R * this->rb->t0[this->idx];
	this->rb->t1[this->idx] = R * this->rb->t1[this->idx];
	this->set_rotation(q_ * this->rb->q0[this->idx]);
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_rotation(double angle_deg, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot)
{
	this->add_translation(-pivot);
	this->add_rotation(Eigen::Quaterniond(Eigen::AngleAxis<double>(deg2rad(angle_deg), axis.normalized())));
	this->add_translation(pivot);
	return (*this);
}
Eigen::Vector3d stark::RigidBodyHandler::get_force() const
{
	return this->rb->force[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_force_at(const Eigen::Vector3d& force_glob_coords, const Eigen::Vector3d& application_point_glob_coords)
{
	this->rb->force[this->idx] = force_glob_coords;
	this->rb->torque[this->idx] = (application_point_glob_coords - this->rb->t1[this->idx]).cross(force_glob_coords);
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_force_at_centroid(const Eigen::Vector3d& force_glob_coords)
{
	this->rb->force[this->idx] = force_glob_coords;
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_force_at_centroid(const Eigen::Vector3d& force_glob_coords)
{
	this->rb->force[this->idx] += force_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::RigidBodyHandler::get_torque() const
{
	return this->rb->torque[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_torque(const Eigen::Vector3d& torque_glob_coords)
{
	this->rb->torque[this->idx] = torque_glob_coords;
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_torque(const Eigen::Vector3d& torque_glob_coords)
{
	this->rb->torque[this->idx] += torque_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::RigidBodyHandler::get_velocity() const
{
	return this->rb->v1[this->idx];
}
Eigen::Vector3d stark::RigidBodyHandler::get_velocity_at(const Eigen::Vector3d& x_loc) const
{
	return this->rb->get_velocity_at(this->get_idx(), x_loc);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_velocity(const Eigen::Vector3d& vel_glob_coords)
{
	this->rb->v0[this->idx] = vel_glob_coords;
	this->rb->v1[this->idx] = vel_glob_coords;
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_velocity(const Eigen::Vector3d& vel_glob_coords)
{
	this->rb->v0[this->idx] += vel_glob_coords;
	this->rb->v1[this->idx] += vel_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::RigidBodyHandler::get_angular_velocity() const
{
	return this->rb->w1[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->rb->w0[this->idx] = angular_vel_glob_coords;
	this->rb->w1[this->idx] = angular_vel_glob_coords;
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->rb->w0[this->idx] += angular_vel_glob_coords;
	this->rb->w1[this->idx] += angular_vel_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::RigidBodyHandler::get_acceleration() const
{
	return this->rb->a[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_acceleration(const Eigen::Vector3d& acc_glob_coords)
{
	this->rb->a[this->idx] = acc_glob_coords;
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_acceleration(const Eigen::Vector3d& acc_glob_coords)
{
	this->rb->a[this->idx] += acc_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::RigidBodyHandler::get_angular_acceleration() const
{
	return this->rb->aa[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords)
{
	this->rb->aa[this->idx] = ang_acc_glob_coords;
	return (*this);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::add_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords)
{
	this->rb->aa[this->idx] += ang_acc_glob_coords;
	return (*this);
}

Eigen::Vector3d stark::RigidBodyHandler::transform_local_to_global_point(const Eigen::Vector3d& x) const
{
	return stark::local_to_global_point(x, this->rb->R1[this->idx], this->rb->t1[this->idx]);
}
Eigen::Vector3d stark::RigidBodyHandler::transform_local_to_global_direction(const Eigen::Vector3d& d) const
{
	return stark::local_to_global_direction(d, this->rb->R1[this->idx]);
}
Eigen::Matrix3d stark::RigidBodyHandler::transform_local_to_global_matrix(const Eigen::Matrix3d& A) const
{
	return stark::local_to_global_matrix(A, this->rb->R1[this->idx]);
}

std::vector<Eigen::Vector3d> stark::RigidBodyHandler::transform_local_to_global_points(const std::vector<Eigen::Vector3d>& loc_points) const
{
	std::vector<Eigen::Vector3d> glob_points(loc_points.size());
	for (int i = 0; i < loc_points.size(); i++) {
		glob_points[i] = this->transform_local_to_global_point(loc_points[i]);
	}
	return glob_points;
}

Eigen::Vector3d stark::RigidBodyHandler::transform_global_to_local_point(const Eigen::Vector3d& x) const
{
	return stark::global_to_local_point(x, this->rb->R1[this->idx], this->rb->t1[this->idx]);
}
Eigen::Vector3d stark::RigidBodyHandler::transform_global_to_local_direction(const Eigen::Vector3d& d) const
{
	return stark::global_to_local_direction(d, this->rb->R1[this->idx]);
}
Eigen::Matrix3d stark::RigidBodyHandler::transform_global_to_local_matrix(const Eigen::Matrix3d& A) const
{
	return stark::global_to_local_matrix(A, this->rb->R1[this->idx]);
}

std::vector<Eigen::Vector3d> stark::RigidBodyHandler::transform_global_to_local_points(const std::vector<Eigen::Vector3d>& glob_points) const
{
	std::vector<Eigen::Vector3d> loc_points(glob_points.size());
	for (int i = 0; i < glob_points.size(); i++) {
		loc_points[i] = this->transform_global_to_local_point(glob_points[i]);
	}
	return loc_points;
}

double stark::RigidBodyHandler::get_mass() const
{
	return this->inertia->mass[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_mass(double mass)
{
	this->inertia->mass[this->idx] = mass;
	return (*this);
}
Eigen::Matrix3d stark::RigidBodyHandler::get_local_inertia_tensor() const
{
	return this->inertia->J_loc[this->idx];
}
Eigen::Matrix3d stark::RigidBodyHandler::get_global_inertia_tensor() const
{
	return this->transform_local_to_global_matrix(this->inertia->J_loc[this->idx]);
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_local_inertia_tensor(const Eigen::Matrix3d& inertia_tensor)
{
	this->inertia->J_loc[this->idx] = inertia_tensor;
	return (*this);
}
double stark::RigidBodyHandler::get_linear_damping() const
{
	return this->inertia->linear_damping[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_linear_damping(double damping)
{
	this->inertia->linear_damping[this->idx] = damping;
	return (*this);
}
double stark::RigidBodyHandler::get_angular_damping() const
{
	return this->inertia->angular_damping[this->idx];
}
stark::RigidBodyHandler& stark::RigidBodyHandler::set_angular_damping(double damping)
{
	this->inertia->angular_damping[this->idx] = damping;
	return (*this);
}
