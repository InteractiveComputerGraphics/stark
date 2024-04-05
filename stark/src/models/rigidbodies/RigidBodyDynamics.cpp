#include "RigidBodyDynamics.h"

#include "rigidbody_transformations.h"
#include "../time_integration.h"
#include "../../utils/include.h"

stark::RigidBodyDynamics::RigidBodyDynamics(stark::core::Stark& stark)
{
	this->dof_v = stark.global_energy.add_dof_array(this->v1, "rb_v1");
	this->dof_w = stark.global_energy.add_dof_array(this->w1, "rb_w1");

	stark.callbacks.add_before_time_step([&]() { this->_before_time_step(stark); });
	stark.callbacks.add_on_time_step_accepted([&]() { this->_on_time_step_accepted(stark); });
}
int stark::RigidBodyDynamics::add(const std::string& label)
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
	this->labels.push_back( label.empty() ? "rb_" + std::to_string(id) : label );

	return id;
}
int stark::RigidBodyDynamics::get_n_bodies() const
{
	return (int)this->t0.size();
}

symx::Vector stark::RigidBodyDynamics::get_x1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Scalar& dt)
{
	return this->get_x1(energy, rb_idx, std::vector<symx::Vector>({ x_loc }), dt)[0];
}
std::vector<symx::Vector> stark::RigidBodyDynamics::get_x1(symx::Energy& energy, const symx::Index& rb_idx, const std::vector<symx::Vector>& x_loc, const symx::Scalar& dt)
{
	symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

	symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
	symx::Vector t1 = time_integration(t0, v1, dt);

	std::vector<symx::Vector> x1;
	for (const symx::Vector& x_loc_a : x_loc) {
		x1.push_back(local_to_global_point(x_loc_a, t1, R1));
	}
	return x1;
}
symx::Vector stark::RigidBodyDynamics::get_v1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Scalar& dt)
{
	return this->get_v1(energy, rb_idx, std::vector<symx::Vector>({ x_loc }), dt)[0];
}
std::vector<symx::Vector> stark::RigidBodyDynamics::get_v1(symx::Energy& energy, const symx::Index& rb_idx, const std::vector<symx::Vector>& x_loc, const symx::Scalar& dt)
{
	symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

	symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
	symx::Vector t1 = time_integration(t0, v1, dt);

	std::vector<symx::Vector> v1_glob;
	for (const symx::Vector& x_loc_a : x_loc) {
		symx::Vector x1a = local_to_global_point(x_loc_a, t1, R1);
		symx::Vector r1a = x1a - t1;
		symx::Vector v1a = global_point_velocity_in_rigib_body(v1, w1, r1a);
		v1_glob.push_back(v1a);
	}
	return v1_glob;
}
symx::Vector stark::RigidBodyDynamics::get_d1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& d_loc, const symx::Scalar& dt)
{
	symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
	symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);
	return integrate_loc_direction(d_loc, q0, w1, dt);
}
std::array<symx::Vector, 2> stark::RigidBodyDynamics::get_x1_d1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Vector& d_loc, const symx::Scalar& dt)
{
	symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

	symx::Vector x1 = integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
	symx::Vector d1 = integrate_loc_direction(d_loc, q0, w1, dt);
	return { x1, d1 };
}
std::array<symx::Vector, 2> stark::RigidBodyDynamics::get_x0_x1(symx::Energy& energy, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Scalar& dt)
{
	symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

	symx::Vector x1 = integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
	symx::Vector x0 = local_to_global_point(x_loc, t0, q0);
	return { x0, x1 };
}

Eigen::Vector3d stark::RigidBodyDynamics::get_x1(int rb, const Eigen::Vector3d& x_loc, double dt) const
{
	return integrate_loc_point(x_loc, this->t0[rb], this->q0[rb], this->v1[rb], this->w1[rb], dt);
}
Eigen::Vector3d stark::RigidBodyDynamics::get_d1(int rb, const Eigen::Vector3d& d_loc, double dt) const
{
	return integrate_loc_direction(d_loc, this->q0[rb], this->w1[rb], dt);
}

Eigen::Vector3d stark::RigidBodyDynamics::get_position_at(int rb, const Eigen::Vector3d& x_loc) const
{
	return local_to_global_point(x_loc, this->R1[rb], this->t1[rb]);
}
Eigen::Vector3d stark::RigidBodyDynamics::get_velocity_at(int rb, const Eigen::Vector3d& x_loc) const
{
	return this->v1[rb] + this->w1[rb].cross((this->get_position_at(rb, x_loc) - this->t1[rb]));
}
Eigen::Vector3d stark::RigidBodyDynamics::get_direction(int rb, const Eigen::Vector3d& d_loc) const
{
	return local_to_global_direction(d_loc, this->R1[rb]);
}

void stark::RigidBodyDynamics::_before_time_step(stark::core::Stark& stark)
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
void stark::RigidBodyDynamics::_on_time_step_accepted(stark::core::Stark& stark)
{
	const double dt = stark.dt;

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
