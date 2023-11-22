#include "RigidBodyDynamics.h"

#include <string>

#include "rigidbody_transformations.h"
#include "time_integration.h"
#include "../utils/mesh_utils.h"


// Helper -------------------------------------------------------------------------
void error_if_different_ps(const stark::models::Id& id, std::string label) 
{
	if (id.get_physical_system() != stark::models::PhysicalSystem::RigidBodies) {
		std::cout << "stark error: RigidBodyDynamics::" + label + "() cannot be used on an Id with different PhysicalSystem than RigidBodies." << std::endl;
		exit(-1);
	}
}
// --------------------------------------------------------------------------------

stark::models::RigidBodyDynamics::RigidBodyDynamics(Stark& stark)
{
	this->dof_v = stark.global_energy.add_dof_array(this->v1, "rb_v1");
	this->dof_w = stark.global_energy.add_dof_array(this->w1, "rb_w1");

	stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step(stark); });
	stark.callbacks.after_time_step.push_back([&]() { this->_after_time_step(stark); });
}
stark::models::Id stark::models::RigidBodyDynamics::add(const double mass, const Eigen::Matrix3d & inertia_loc)
{
	this->t0.push_back(Eigen::Vector3d::Zero());
	this->t1.push_back(Eigen::Vector3d::Zero());
	this->q0.push_back(Eigen::Quaterniond::Identity());
	this->q1.push_back(Eigen::Quaterniond::Identity());
	this->R0.push_back(Eigen::Matrix3d::Identity());
	this->R1.push_back(Eigen::Matrix3d::Identity());
	this->v0.push_back(Eigen::Vector3d::Zero());
	this->v1.push_back(Eigen::Vector3d::Zero());
	this->w0.push_back(Eigen::Vector3d::Zero());
	this->w1.push_back(Eigen::Vector3d::Zero());
	this->a.push_back(Eigen::Vector3d::Zero());
	this->aa.push_back(Eigen::Vector3d::Zero());
	this->force.push_back(Eigen::Vector3d::Zero());
	this->torque.push_back(Eigen::Vector3d::Zero());

	return Id(PhysicalSystem::RigidBodies, this->t0.size() - 1);
}
stark::models::Id stark::models::RigidBodyDynamics::add_and_transform(const double mass, const Eigen::Matrix3d& inertia_loc, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
{
	const Id id = this->add(mass, inertia_loc);
	this->add_rotation(id, rotate_deg, rotation_axis);
	this->add_displacement(id, displacement);
	return id;
}
void stark::models::RigidBodyDynamics::set_translation(const Id& id, const Eigen::Vector3d& t)
{
	error_if_different_ps(id, "set_translation");
	this->t1[id.get_global_idx()] = t;
	this->t0[id.get_global_idx()] = t;
}
void stark::models::RigidBodyDynamics::add_displacement(const Id& id, const Eigen::Vector3d& t)
{
	error_if_different_ps(id, "add_displacement");
	this->t1[id.get_global_idx()] += t;
	this->t0[id.get_global_idx()] += t;
}
void stark::models::RigidBodyDynamics::set_rotation(const Id& id, const Eigen::Quaterniond& q)
{
	error_if_different_ps(id, "set_rotation");
	this->q0[id.get_global_idx()] = q.normalized();
	this->q1[id.get_global_idx()] = q.normalized();
	this->R0[id.get_global_idx()] = this->q0[id.get_global_idx()].toRotationMatrix();
	this->R1[id.get_global_idx()] = this->q1[id.get_global_idx()].toRotationMatrix();
}
void stark::models::RigidBodyDynamics::set_rotation(const Id& id, const double& angle_deg, const Eigen::Vector3d& axis)
{
	error_if_different_ps(id, "set_rotation");
	this->set_rotation(id, Eigen::Quaterniond(Eigen::AngleAxis<double>(utils::deg2rad(angle_deg), axis.normalized())));
}
void stark::models::RigidBodyDynamics::add_rotation(const Id& id, const Eigen::Quaterniond& q)
{
	error_if_different_ps(id, "add_rotation");
	const Eigen::Quaterniond q_ = q.normalized();
	const Eigen::Matrix3d R = q_.toRotationMatrix();
	this->t0[id.get_global_idx()] = R * this->t0[id.get_global_idx()];
	this->t1[id.get_global_idx()] = R * this->t1[id.get_global_idx()];
	this->set_rotation(id, this->q0[id.get_global_idx()] * q_);
}
void stark::models::RigidBodyDynamics::add_rotation(const Id& id, const double& angle_deg, const Eigen::Vector3d& axis)
{
	error_if_different_ps(id, "add_rotation");
	this->add_rotation(id, Eigen::Quaterniond(Eigen::AngleAxis<double>(utils::deg2rad(angle_deg), axis.normalized())));
}
void stark::models::RigidBodyDynamics::add_force_at(const Id& id, const Eigen::Vector3d& force_glob_coords, const Eigen::Vector3d& application_point_glob_coords)
{
	error_if_different_ps(id, "add_force_at");
	this->force[id.get_global_idx()] = force_glob_coords;
	this->torque[id.get_global_idx()] = (application_point_glob_coords - this->t1[id.get_global_idx()]).cross(force_glob_coords);
}
void stark::models::RigidBodyDynamics::set_force_at_centroid(const Id& id, const Eigen::Vector3d& force_glob_coords)
{
	error_if_different_ps(id, "set_force_at_centroid");
	this->force[id.get_global_idx()] = force_glob_coords;
}
void stark::models::RigidBodyDynamics::add_force_at_centroid(const Id& id, const Eigen::Vector3d& force_glob_coords)
{
	error_if_different_ps(id, "add_force_at_centroid");
	this->force[id.get_global_idx()] += force_glob_coords;
}
void stark::models::RigidBodyDynamics::set_torque(const Id& id, const Eigen::Vector3d& torque_glob_coords)
{
	error_if_different_ps(id, "set_torque");
	this->torque[id.get_global_idx()] = torque_glob_coords;
}
void stark::models::RigidBodyDynamics::add_torque(const Id& id, const Eigen::Vector3d& torque_glob_coords)
{
	error_if_different_ps(id, "add_torque");
	this->torque[id.get_global_idx()] += torque_glob_coords;
}
void stark::models::RigidBodyDynamics::set_velocity(const Id& id, const Eigen::Vector3d& vel_glob_coords)
{
	error_if_different_ps(id, "set_velocity");
	this->v0[id.get_global_idx()] = vel_glob_coords;
	this->v1[id.get_global_idx()] = vel_glob_coords;
}
void stark::models::RigidBodyDynamics::add_velocity(const Id& id, const Eigen::Vector3d& vel_glob_coords)
{
	error_if_different_ps(id, "add_velocity");
	this->v0[id.get_global_idx()] += vel_glob_coords;
	this->v1[id.get_global_idx()] += vel_glob_coords;
}
void stark::models::RigidBodyDynamics::set_angular_velocity(const Id& id, const Eigen::Vector3d& angular_vel_glob_coords)
{
	error_if_different_ps(id, "set_angular_velocity");
	this->w0[id.get_global_idx()] = angular_vel_glob_coords;
	this->w1[id.get_global_idx()] = angular_vel_glob_coords;
}
void stark::models::RigidBodyDynamics::add_angular_velocity(const Id& id, const Eigen::Vector3d& angular_vel_glob_coords)
{
	error_if_different_ps(id, "add_angular_velocity");
	this->w0[id.get_global_idx()] += angular_vel_glob_coords;
	this->w1[id.get_global_idx()] += angular_vel_glob_coords;
}
void stark::models::RigidBodyDynamics::set_acceleration(const Id& id, const Eigen::Vector3d& acc_glob_coords)
{
	error_if_different_ps(id, "set_acceleration");
	this->a[id.get_global_idx()] = acc_glob_coords;
}
void stark::models::RigidBodyDynamics::add_acceleration(const Id& id, const Eigen::Vector3d& acc_glob_coords)
{
	error_if_different_ps(id, "add_acceleration");
	this->a[id.get_global_idx()] += acc_glob_coords;
}
void stark::models::RigidBodyDynamics::set_angular_acceleration(const Id& id, const Eigen::Vector3d& ang_acc_glob_coords)
{
	error_if_different_ps(id, "set_angular_acceleration");
	this->aa[id.get_global_idx()] = ang_acc_glob_coords;
}
void stark::models::RigidBodyDynamics::add_angular_acceleration(const Id& id, const Eigen::Vector3d& ang_acc_glob_coords)
{
	error_if_different_ps(id, "add_angular_acceleration");
	this->aa[id.get_global_idx()] += ang_acc_glob_coords;
}
Eigen::Vector3d stark::models::RigidBodyDynamics::get_point_in_global_coordinates(const Id& id, const Eigen::Vector3d& p)
{
	error_if_different_ps(id, "get_point_in_global_coordinates");
	return local_to_global_point(p, this->R1[id.get_global_idx()], this->t1[id.get_global_idx()]);
}
int stark::models::RigidBodyDynamics::get_n_bodies() const
{
	return (int)this->t0.size();
}
bool stark::models::RigidBodyDynamics::is_empty() const
{
	return this->get_n_bodies() == 0;
}
bool stark::models::RigidBodyDynamics::is_body_declared(const Id& id) const
{
	error_if_different_ps(id, "is_body_declared");
	return id.get_global_idx() < this->get_n_bodies();
}

void stark::models::RigidBodyDynamics::_before_time_step(Stark& stark)
{
	if (this->is_empty()) { return; }

	// Quaternions
	this->q0_.resize(this->q0.size());
	for (int i = 0; i < (int)this->q0.size(); i++) {
		Eigen::Quaterniond& q = this->q0[i];
		this->q0_[i] = { q.w(), q.x(), q.y(), q.z() };
	}

	// Set next time velocities estimation to zero to avoid invalid state outside of the minimzer
	std::fill(this->v1.begin(), this->v1.end(), Eigen::Vector3d::Zero());
	std::fill(this->w1.begin(), this->w1.end(), Eigen::Vector3d::Zero());
}
void stark::models::RigidBodyDynamics::_after_time_step(Stark& stark)
{
	if (this->is_empty()) { return; }

	const double dt = stark.settings.simulation.adaptive_time_step.value;

	// Set final positions with solved velocities
	for (int i = 0; i < this->get_n_bodies(); i++) {
		this->t1[i] = time_integration(this->t0[i], this->v1[i], dt);
		this->q1[i] = quat_time_integration(this->q0[i], this->w1[i], dt);
		this->R1[i] = this->q1[i].toRotationMatrix();
	}

	// x0 <- x1
	this->t0 = this->t1;
	this->q0 = this->q1;
	this->R0 = this->R1;
	this->v0 = this->v1;
	this->w0 = this->w1;
}
