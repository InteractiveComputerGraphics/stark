#include "RigidBodyDynamics.h"

#include "rigidbody_transformations.h"
#include "../time_integration.h"
#include "../../utils/mesh_utils.h"

stark::models::RigidBodyDynamics::RigidBodyDynamics(stark::core::Stark& stark)
{
	this->dof_v = stark.global_energy.add_dof_array(this->v1, "rb_v1");
	this->dof_w = stark.global_energy.add_dof_array(this->w1, "rb_w1");

	stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step(stark); });
	stark.callbacks.after_time_step.push_back([&]() { this->_after_time_step(stark); });
}
int stark::models::RigidBodyDynamics::add()
{
	const int id = (int)this->t0.size();
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
	this->labels.push_back(std::to_string(id));

	return id;
}
int stark::models::RigidBodyDynamics::get_n_bodies() const
{
	return (int)this->t0.size();
}

void stark::models::RigidBodyDynamics::_before_time_step(stark::core::Stark& stark)
{
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
void stark::models::RigidBodyDynamics::_after_time_step(stark::core::Stark& stark)
{
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
