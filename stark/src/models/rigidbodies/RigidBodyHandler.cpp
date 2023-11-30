#include "RigidBodyHandler.h"

#include "rigidbody_transformations.h"
#include "../../utils/mesh_utils.h"

stark::models::RigidBodyHandler::RigidBodyHandler(spRigidBodiesInternal rb, int idx)
	: rb(rb), idx(idx)
{
}
int stark::models::RigidBodyHandler::index() const
{
	return this->idx;
}
std::string stark::models::RigidBodyHandler::get_label() const
{
	return this->rb->dyn->labels[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_label(std::string label)
{
	this->rb->dyn->labels[this->idx] = label;
	return (*this);
}
double stark::models::RigidBodyHandler::get_mass() const
{
	return this->rb->inertia->mass[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_mass(double mass)
{
	this->rb->inertia->mass[this->idx] = mass;
	return (*this);
}
Eigen::Matrix3d stark::models::RigidBodyHandler::get_local_inertia_tensor() const
{
	return this->rb->inertia->J_loc[this->idx];
}
Eigen::Matrix3d stark::models::RigidBodyHandler::get_global_inertia_tensor() const
{
	return this->local_to_global_matrix(this->rb->inertia->J_loc[this->idx]);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_local_inertia_tensor(const Eigen::Matrix3d& inertia_tensor)
{
	this->rb->inertia->J_loc[this->idx] = inertia_tensor;
	return (*this);
}
double stark::models::RigidBodyHandler::get_linear_damping() const
{
	return this->rb->inertia->linear_damping[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_linear_damping(double damping)
{
	this->rb->inertia->linear_damping[this->idx] = damping;
	return (*this);
}
double stark::models::RigidBodyHandler::get_angular_damping() const
{
	return this->rb->inertia->angular_damping[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_angular_damping(double damping)
{
	this->rb->inertia->angular_damping[this->idx] = damping;
	return (*this);
}
Eigen::Vector3d stark::models::RigidBodyHandler::get_translation() const
{
	return this->rb->dyn->t1[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_translation(const Eigen::Vector3d& t)
{
	this->rb->dyn->t1[this->idx] = t;
	this->rb->dyn->t0[this->idx] = t;
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_displacement(const Eigen::Vector3d& t)
{
	this->rb->dyn->t1[this->idx] += t;
	this->rb->dyn->t0[this->idx] += t;
	return (*this);
}
Eigen::Quaterniond stark::models::RigidBodyHandler::get_quaternion() const
{
	return this->rb->dyn->q1[this->idx];
}
Eigen::Matrix3d stark::models::RigidBodyHandler::get_rotation_matrix() const
{
	return this->rb->dyn->R1[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_rotation(const Eigen::Quaterniond& q)
{
	this->rb->dyn->q0[this->idx] = q.normalized();
	this->rb->dyn->q1[this->idx] = q.normalized();
	this->rb->dyn->R0[this->idx] = this->rb->dyn->q0[this->idx].toRotationMatrix();
	this->rb->dyn->R1[this->idx] = this->rb->dyn->q1[this->idx].toRotationMatrix();
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_rotation(const double& angle_deg, const Eigen::Vector3d& axis)
{
	this->set_rotation(Eigen::Quaterniond(Eigen::AngleAxis<double>(utils::deg2rad(angle_deg), axis.normalized())));
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_rotation(const Eigen::Quaterniond& q)
{
	const Eigen::Quaterniond q_ = q.normalized();
	const Eigen::Matrix3d R = q_.toRotationMatrix();
	this->rb->dyn->t0[this->idx] = R * this->rb->dyn->t0[this->idx];
	this->rb->dyn->t1[this->idx] = R * this->rb->dyn->t1[this->idx];
	this->set_rotation(this->rb->dyn->q0[this->idx] * q_);
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_rotation(const double& angle_deg, const Eigen::Vector3d& axis)
{
	this->add_rotation(Eigen::Quaterniond(Eigen::AngleAxis<double>(utils::deg2rad(angle_deg), axis.normalized())));
	return (*this);
}
Eigen::Vector3d stark::models::RigidBodyHandler::get_force() const
{
	return this->rb->dyn->force[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_force_at(const Eigen::Vector3d& force_glob_coords, const Eigen::Vector3d& application_point_glob_coords)
{
	this->rb->dyn->force[this->idx] = force_glob_coords;
	this->rb->dyn->torque[this->idx] = (application_point_glob_coords - this->rb->dyn->t1[this->idx]).cross(force_glob_coords);
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_force_at_centroid(const Eigen::Vector3d& force_glob_coords)
{
	this->rb->dyn->force[this->idx] = force_glob_coords;
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_force_at_centroid(const Eigen::Vector3d& force_glob_coords)
{
	this->rb->dyn->force[this->idx] += force_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::models::RigidBodyHandler::get_torque() const
{
	return this->rb->dyn->torque[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_torque(const Eigen::Vector3d& torque_glob_coords)
{
	this->rb->dyn->torque[this->idx] = torque_glob_coords;
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_torque(const Eigen::Vector3d& torque_glob_coords)
{
	this->rb->dyn->torque[this->idx] += torque_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::models::RigidBodyHandler::get_velocity() const
{
	return this->rb->dyn->v1[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_velocity(const Eigen::Vector3d& vel_glob_coords)
{
	this->rb->dyn->v0[this->idx] = vel_glob_coords;
	this->rb->dyn->v1[this->idx] = vel_glob_coords;
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_velocity(const Eigen::Vector3d& vel_glob_coords)
{
	this->rb->dyn->v0[this->idx] += vel_glob_coords;
	this->rb->dyn->v1[this->idx] += vel_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::models::RigidBodyHandler::get_angular_velocity() const
{
	return this->rb->dyn->w1[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->rb->dyn->w0[this->idx] = angular_vel_glob_coords;
	this->rb->dyn->w1[this->idx] = angular_vel_glob_coords;
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_angular_velocity(const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->rb->dyn->w0[this->idx] += angular_vel_glob_coords;
	this->rb->dyn->w1[this->idx] += angular_vel_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::models::RigidBodyHandler::get_acceleration() const
{
	return this->rb->dyn->a[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_acceleration(const Eigen::Vector3d& acc_glob_coords)
{
	this->rb->dyn->a[this->idx] = acc_glob_coords;
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_acceleration(const Eigen::Vector3d& acc_glob_coords)
{
	this->rb->dyn->a[this->idx] += acc_glob_coords;
	return (*this);
}
Eigen::Vector3d stark::models::RigidBodyHandler::get_angular_acceleration() const
{
	return this->rb->dyn->aa[this->idx];
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords)
{
	this->rb->dyn->aa[this->idx] = ang_acc_glob_coords;
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_angular_acceleration(const Eigen::Vector3d& ang_acc_glob_coords)
{
	this->rb->dyn->aa[this->idx] += ang_acc_glob_coords;
	return (*this);
}

Eigen::Vector3d stark::models::RigidBodyHandler::local_to_global_point(const Eigen::Vector3d& x) const
{
	return stark::models::local_to_global_point(x, this->rb->dyn->R1[this->idx], this->rb->dyn->t1[this->idx]);
}
Eigen::Vector3d stark::models::RigidBodyHandler::local_to_global_direction(const Eigen::Vector3d& d) const
{
	return stark::models::local_to_global_direction(d, this->rb->dyn->R1[this->idx]);
}
Eigen::Matrix3d stark::models::RigidBodyHandler::local_to_global_matrix(const Eigen::Matrix3d& A) const
{
	return stark::models::local_to_global_matrix(A, this->rb->dyn->R1[this->idx]);
}

Eigen::Vector3d stark::models::RigidBodyHandler::global_to_local_point(const Eigen::Vector3d& x) const
{
	return stark::models::global_to_local_point(x, this->rb->dyn->R1[this->idx], this->rb->dyn->t1[this->idx]);
}
Eigen::Vector3d stark::models::RigidBodyHandler::global_to_local_direction(const Eigen::Vector3d& d) const
{
	return stark::models::global_to_local_direction(d, this->rb->dyn->R1[this->idx]);
}
Eigen::Matrix3d stark::models::RigidBodyHandler::global_to_local_matrix(const Eigen::Matrix3d& A) const
{
	return stark::models::global_to_local_matrix(A, this->rb->dyn->R1[this->idx]);
}

stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_collision_mesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles)
{
	this->rb->collision_meshes[this->idx].vertices = vertices;
	this->rb->collision_meshes[this->idx].conn = triangles;
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::set_render_mesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles)
{
	this->rb->render_meshes[this->idx].vertices = vertices;
	this->rb->render_meshes[this->idx].conn = triangles;
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::add_to_output_label(const std::string label)
{
	this->rb->output_groups.add_to_group(label, this->idx);
	return (*this);
}
stark::models::RigidBodyHandler& stark::models::RigidBodyHandler::enable_writing_transformation_sequence(const std::string label)
{
	this->rb->transformation_sequences.push_back({});
	this->rb->transformation_sequences.back().body_idx = this->index();
	this->rb->transformation_sequences.back().label = label;
	return (*this);
}
