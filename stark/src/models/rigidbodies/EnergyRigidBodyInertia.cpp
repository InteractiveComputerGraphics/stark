#include "EnergyRigidBodyInertia.h"

#include "rigidbody_transformations.h"

using namespace symx;

stark::EnergyRigidBodyInertia::EnergyRigidBodyInertia(core::Stark& stark, spRigidBodyDynamics rb)
	: rb(rb)
{
	stark.callbacks->add_before_time_step([&]() { this->_before_time_step(stark); });

	// Linear inertia
	stark.global_potential->add_potential("EnergyRigidBodyInertia_Linear", this->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			Vector v1 = mws.make_vector(this->rb->v1, conn["rb"]);
			Vector v0 = mws.make_vector(this->rb->v0, conn["rb"]);
			Vector a = mws.make_vector(this->rb->a, conn["rb"]);
			Vector f = mws.make_vector(this->rb->force, conn["rb"]);
			Scalar m = mws.make_scalar(this->mass, conn["rb"]);
			Scalar damping = mws.make_scalar(this->linear_damping, conn["rb"]);
			Scalar dt = mws.make_scalar(stark.dt);
			Vector gravity = mws.make_vector(stark.gravity);

			Vector vhat = v0 + dt * (a + gravity + f / m);
			Vector dev = v1 - vhat;
			Scalar E = 0.5 * m * dev.dot(dev) + 0.5 * m * v1.dot(v1) * damping * dt;
			// Actual force: f = -dE/dx = -1/dt*(v1 - vhat)*m
			return E;
		}
	);

	// Angular inertia
	stark.global_potential->add_potential("EnergyRigidBodyInertia_Angular", this->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			Vector w1 = mws.make_vector(this->rb->w1, conn["rb"]);
			Vector w0 = mws.make_vector(this->rb->w0, conn["rb"]);
			Vector aa = mws.make_vector(this->rb->aa, conn["rb"]);
			Vector t = mws.make_vector(this->rb->torque, conn["rb"]);
			Matrix J0_glob = mws.make_matrix(this->J0_glob, { 3, 3 }, conn["rb"]);
			Matrix J0_inv_glob = mws.make_matrix(this->J0_inv_glob, { 3, 3 }, conn["rb"]);
			Scalar damping = mws.make_scalar(this->angular_damping, conn["rb"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector what = w0 + dt * (aa + J0_inv_glob * t);
			Vector dev = w1 - what;
			Scalar E = 0.5 * (dev.dot(J0_glob.dot(dev)) + w1.dot(J0_glob.dot(w1)) * damping * dt);
			return E;
		}
	);
}

void stark::EnergyRigidBodyInertia::add(const int rb_idx, const double mass, const Eigen::Matrix3d& inertia_loc, const double linear_damping, const double angular_damping)
{
	if (rb_idx != (int)this->mass.size()) {
		std::cout << "Stark error: EnergyRigidBodyInertia::add() found non-consequtive rigid body added." << std::endl;
		exit(-1);
	}

	this->conn.push_back({ (int)this->mass.size() });
	this->mass.push_back(mass);
	this->J_loc.push_back(inertia_loc);
	this->linear_damping.push_back(linear_damping);
	this->angular_damping.push_back(angular_damping);
}

void stark::EnergyRigidBodyInertia::_before_time_step(stark::core::Stark& stark)
{
	const int n = this->rb->get_n_bodies();
	this->J0_glob.resize(n);
	this->J0_inv_glob.resize(n);
	for (int i = 0; i < n; i++) {
		const Eigen::Matrix3d J = local_to_global_matrix(this->J_loc[i], this->rb->R0[i]);
		this->J0_glob[i] = {
			J(0, 0), J(0, 1), J(0, 2),
			J(1, 0), J(1, 1), J(1, 2),
			J(2, 0), J(2, 1), J(2, 2),
		};
		const Eigen::Matrix3d J_inv = J.inverse();
		this->J0_inv_glob[i] = {
			J_inv(0, 0), J_inv(0, 1), J_inv(0, 2),
			J_inv(1, 0), J_inv(1, 1), J_inv(1, 2),
			J_inv(2, 0), J_inv(2, 1), J_inv(2, 2),
		};
	}
}
