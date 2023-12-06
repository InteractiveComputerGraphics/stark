#include "RigidBodyDynamics.h"

#include "rigidbody_transformations.h"
#include "../time_integration.h"
#include "../../utils/mesh_utils.h"

stark::models::RigidBodyDynamics::RigidBodyDynamics(stark::core::Stark& stark)
{
	this->dof_v = stark.global_energy.add_dof_array(this->v1, "rb_v1");
	this->dof_w = stark.global_energy.add_dof_array(this->w1, "rb_w1");

	stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step(stark); });
	stark.callbacks.on_time_step_accepted.push_back([&]() { this->_on_time_step_accepted(stark); });
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
void stark::models::RigidBodyDynamics::_on_time_step_accepted(stark::core::Stark& stark)
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


symx::Vector stark::models::RigidBodyDynamics::get_x1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Scalar& dt)
{
	symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);
	return integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
}
symx::Vector stark::models::RigidBodyDynamics::get_d1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& d_loc, const symx::Scalar& dt)
{
	symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
	symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);
	return integrate_loc_direction(d_loc, q0, w1, dt);
}
std::array<symx::Vector, 2> stark::models::RigidBodyDynamics::get_x1_d1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Vector& d_loc, const symx::Scalar& dt)
{
	symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

	symx::Vector x1 = integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
	symx::Vector d1 = integrate_loc_direction(d_loc, q0, w1, dt);
	return { x1, d1 };
}
std::array<symx::Vector, 2> stark::models::RigidBodyDynamics::get_x0_x1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Scalar& dt)
{
	symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

	symx::Vector x1 = integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
	symx::Vector x0 = local_to_global_point(x_loc, t0, q0);
	return { x0, x1 };
}

Eigen::Vector3d stark::models::RigidBodyDynamics::get_x1(int rb_idx, const Eigen::Vector3d& x_loc, double dt)
{
	return integrate_loc_point(x_loc, this->t0[rb_idx], this->q0[rb_idx], this->v1[rb_idx], this->w1[rb_idx], dt);
}
Eigen::Vector3d stark::models::RigidBodyDynamics::get_d1(int rb_idx, const Eigen::Vector3d& d_loc, double dt)
{
	return integrate_loc_direction(d_loc, this->q0[rb_idx], this->w1[rb_idx], dt);
}

Eigen::Vector3d stark::models::RigidBodyDynamics::get_x1(int rb_idx, const Eigen::Vector3d& x_loc)
{
	return local_to_global_point(x_loc, this->R1[rb_idx], this->t1[rb_idx]);
}
Eigen::Vector3d stark::models::RigidBodyDynamics::get_d1(int rb_idx, const Eigen::Vector3d& d_loc)
{
	return local_to_global_direction(d_loc, this->R1[rb_idx]);
}
