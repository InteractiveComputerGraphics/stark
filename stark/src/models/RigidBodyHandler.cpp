#include "RigidBodyHandler.h"

#include "rigidbody_transformations.h"
#include "../utils/mesh_utils.h"

stark::models::RigidBodyHandler::RigidBodyHandler(spRigidBodiesData data, int idx)
	: data(data), idx(idx)
{
}

int stark::models::RigidBodyHandler::index() const
{
	return this->idx;
}

Eigen::Vector3d stark::models::RigidBodyHandler::get_translation() const
{
	return this->data->dyn->t1[this->idx];
}

void stark::models::RigidBodyHandler::set_translation(const Eigen::Vector3d& t)
{
	this->data->dyn->t1[this->idx] = t;
	this->data->dyn->t0[this->idx] = t;
}
void stark::models::RigidBodyHandler::add_displacement(const Eigen::Vector3d& t)
{
	this->data->dyn->t1[this->idx] += t;
	this->data->dyn->t0[this->idx] += t;
}
void stark::models::RigidBodyHandler::set_rotation(const Eigen::Quaterniond& q)
{
	this->data->dyn->q0[this->idx] = q.normalized();
	this->data->dyn->q1[this->idx] = q.normalized();
	this->data->dyn->R0[this->idx] = this->data->dyn->q0[this->idx].toRotationMatrix();
	this->data->dyn->R1[this->idx] = this->data->dyn->q1[this->idx].toRotationMatrix();
}
void stark::models::RigidBodyHandler::set_rotation(const double& angle_deg, const Eigen::Vector3d& axis)
{
	this->set_rotation(Eigen::Quaterniond(Eigen::AngleAxis<double>(utils::deg2rad(angle_deg), axis.normalized())));
}
void stark::models::RigidBodyHandler::add_rotation(const Eigen::Quaterniond& q)
{
	const Eigen::Quaterniond q_ = q.normalized();
	const Eigen::Matrix3d R = q_.toRotationMatrix();
	this->data->dyn->t0[this->idx] = R * this->data->dyn->t0[this->idx];
	this->data->dyn->t1[this->idx] = R * this->data->dyn->t1[this->idx];
	this->set_rotation(this->data->dyn->q0[this->idx] * q_);
}
void stark::models::RigidBodyHandler::add_rotation(const double& angle_deg, const Eigen::Vector3d& axis)
{
	this->add_rotation(Eigen::Quaterniond(Eigen::AngleAxis<double>(utils::deg2rad(angle_deg), axis.normalized())));
}
void stark::models::RigidBodyHandler::add_force_at(const Eigen::Vector3d& force_glob_coords, const Eigen::Vector3d& application_point_glob_coords)
{
	this->data->dyn->force[this->idx] = force_glob_coords;
	this->data->dyn->torque[this->idx] = (application_point_glob_coords - this->data->dyn->t1[this->idx]).cross(force_glob_coords);
}
void stark::models::RigidBodyHandler::set_force_at_centroid(const Eigen::Vector3d& force_glob_coords)
{
	this->data->dyn->force[this->idx] = force_glob_coords;
}
void stark::models::RigidBodyHandler::add_force_at_centroid(const Eigen::Vector3d& force_glob_coords)
{
	this->data->dyn->force[this->idx] += force_glob_coords;
}
void stark::models::RigidBodyHandler::set_torque(const Eigen::Vector3d& torque_glob_coords)
{
	this->data->dyn->torque[this->idx] = torque_glob_coords;
}
void stark::models::RigidBodyHandler::add_torque(const Eigen::Vector3d& torque_glob_coords)
{
	this->data->dyn->torque[this->idx] += torque_glob_coords;
}
void stark::models::RigidBodyHandler::set_velocity(const Eigen::Vector3d& vel_glob_coords)
{
	this->data->dyn->v0[this->idx] = vel_glob_coords;
	this->data->dyn->v1[this->idx] = vel_glob_coords;
}
void stark::models::RigidBodyHandler::add_velocity(const Eigen::Vector3d& vel_glob_coords)
{
	this->data->dyn->v0[this->idx] += vel_glob_coords;
	this->data->dyn->v1[this->idx] += vel_glob_coords;
}
void stark::models::RigidBodyHandler::set_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->data->dyn->w0[this->idx] = angular_vel_glob_coords;
	this->data->dyn->w1[this->idx] = angular_vel_glob_coords;
}
void stark::models::RigidBodyHandler::add_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->data->dyn->w0[this->idx] += angular_vel_glob_coords;
	this->data->dyn->w1[this->idx] += angular_vel_glob_coords;
}
void stark::models::RigidBodyHandler::set_acceleration(const Eigen::Vector3d& acc_glob_coords)
{
	this->data->dyn->a[this->idx] = acc_glob_coords;
}
void stark::models::RigidBodyHandler::add_acceleration(const Eigen::Vector3d& acc_glob_coords)
{
	this->data->dyn->a[this->idx] += acc_glob_coords;
}
void stark::models::RigidBodyHandler::set_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords)
{
	this->data->dyn->aa[this->idx] = ang_acc_glob_coords;
}
void stark::models::RigidBodyHandler::add_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords)
{
	this->data->dyn->aa[this->idx] += ang_acc_glob_coords;
}

Eigen::Vector3d stark::models::RigidBodyHandler::local_to_global_point(const Eigen::Vector3d& x)
{
	return stark::models::local_to_global_point(x, this->data->dyn->R1[this->idx], this->data->dyn->t1[this->idx]);
}
Eigen::Vector3d stark::models::RigidBodyHandler::local_to_global_direction(const Eigen::Vector3d& d)
{
	return stark::models::local_to_global_direction(d, this->data->dyn->R1[this->idx]);
}
Eigen::Matrix3d stark::models::RigidBodyHandler::local_to_global_matrix(const Eigen::Matrix3d& A)
{
	return stark::models::local_to_global_matrix(A, this->data->dyn->R1[this->idx]);
}

Eigen::Vector3d stark::models::RigidBodyHandler::global_to_local_point(const Eigen::Vector3d& x)
{
	return stark::models::global_to_local_point(x, this->data->dyn->R1[this->idx], this->data->dyn->t1[this->idx]);
}
Eigen::Vector3d stark::models::RigidBodyHandler::global_to_local_direction(const Eigen::Vector3d& d)
{
	return stark::models::global_to_local_direction(d, this->data->dyn->R1[this->idx]);
}
Eigen::Matrix3d stark::models::RigidBodyHandler::global_to_local_matrix(const Eigen::Matrix3d& A)
{
	return stark::models::global_to_local_matrix(A, this->data->dyn->R1[this->idx]);
}
