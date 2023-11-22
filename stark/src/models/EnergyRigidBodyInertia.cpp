#include "EnergyRigidBodyInertia.h"

#include "rigidbody_transformations.h"

stark::models::EnergyRigidBodyInertia::EnergyRigidBodyInertia(Stark& stark, const spRigidBodyDynamics dyn)
	: dyn(dyn)
{
	stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step(stark); });

	// Linear inertia
	stark.global_energy.add_energy("EnergyRigidBodyInertia_Linear", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector v1 = energy.make_dof_vector(this->dyn->dof_v, this->dyn->v1, conn["rb"]);
			symx::Vector v0 = energy.make_vector(this->dyn->v0, conn["rb"]);
			symx::Vector a = energy.make_vector(this->dyn->a, conn["rb"]);
			symx::Vector f = energy.make_vector(this->dyn->force, conn["rb"]);
			symx::Scalar m = energy.make_scalar(this->mass, conn["rb"]);
			symx::Scalar damping = energy.make_scalar(this->linear_damping, conn["rb"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
			symx::Vector gravity = energy.make_vector(stark.settings.simulation.gravity);

			symx::Vector vhat = v0 + dt * (a + gravity + f / m);
			symx::Vector dev = v1 - vhat;
			symx::Scalar E = 0.5 * m * dev.dot(dev) + 0.5 * m * v1.dot(v1) * damping * dt;
			// Actual force: f = -dE/dx = -1/dt*(v1 - vhat)*m
			energy.set(E);
		}
	);

	// Angular inertia
	stark.global_energy.add_energy("EnergyRigidBodyInertia_Angular", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector w1 = energy.make_dof_vector(this->dyn->dof_w, this->dyn->w1, conn["rb"]);
			symx::Vector w0 = energy.make_vector(this->dyn->w0, conn["rb"]);
			symx::Vector aa = energy.make_vector(this->dyn->aa, conn["rb"]);
			symx::Vector t = energy.make_vector(this->dyn->torque, conn["rb"]);
			symx::Matrix J0_glob = energy.make_matrix(this->J0_glob, { 3, 3 }, conn["rb"]);
			symx::Matrix J0_inv_glob = energy.make_matrix(this->J0_inv_glob, { 3, 3 }, conn["rb"]);
			symx::Scalar damping = energy.make_scalar(this->angular_damping, conn["rb"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			symx::Vector what = w0 + dt * (aa + J0_inv_glob * t);
			symx::Vector dev = w1 - what;
			symx::Scalar E = 0.5 * (dev.transpose() * J0_glob * dev + w1.transpose() * J0_glob * w1 * damping * dt);
			energy.set(E);
		}
	);

}

void stark::models::EnergyRigidBodyInertia::add(const double mass, const Eigen::Matrix3d& inertia_loc, const double linear_damping, const double angular_damping)
{
	this->conn.push_back({ (int)this->mass.size() });
	this->mass.push_back(mass);
	this->J_loc.push_back(inertia_loc);
	this->linear_damping.push_back(linear_damping);
	this->angular_damping.push_back(angular_damping);
}

void stark::models::EnergyRigidBodyInertia::_before_time_step(Stark& stark)
{
	const int n = this->dyn->get_n_bodies();
	this->J0_glob.resize(n);
	this->J0_inv_glob.resize(n);
	for (int i = 0; i < n; i++) {
		const Eigen::Matrix3d J = local_to_global_matrix(this->J_loc[i], this->dyn->R0[i]);
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
