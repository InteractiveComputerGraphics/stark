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
			Scalar is_quasistatic = mws.make_scalar(this->is_quasistatic, conn["rb"]);
			Scalar dt = mws.make_scalar(stark.dt);
			Vector gravity = mws.make_vector(stark.gravity);

			// E_inertia = 0.5*m*||v1 - v0||^2  +  0.5*m*||v1||^2 * d * dt
			// E_ext     = -dt * ( m*(a + g) + f ).T * x1

			// Inertial energy (zero for quasistatic)
			Vector dev = v1 - v0;
			Scalar E_inertia = 0.5*m*dev.dot(dev) + 0.5*m*v1.dot(v1)*damping*dt;

			// External force potential (always applied, including quasistatic)
			Vector f_ext = m * (a + gravity) + f;  // total external force
			Scalar E_ext = -dt * f_ext.dot(v1);

			return E_ext + branch(is_quasistatic > 0.5, 0.0, E_inertia);
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
			Scalar damping = mws.make_scalar(this->angular_damping, conn["rb"]);
			Scalar is_quasistatic = mws.make_scalar(this->is_quasistatic, conn["rb"]);
			Scalar dt = mws.make_scalar(stark.dt);

			// E_inertia = 0.5*(w1-w0)^T J (w1-w0)  +  0.5*w1^T J w1 * d * dt
			// E_ext     = -dt * ( J*aa + t ) . w1

			// Inertial energy (zero for quasistatic)
			Vector dev = w1 - w0;
			Scalar E_inertia = 0.5 * (dev.dot(J0_glob.dot(dev)) + w1.dot(J0_glob.dot(w1)) * damping * dt);

			// External torque potential (always applied, including quasistatic)
			Vector t_ext = J0_glob.dot(aa) + t;  // total external torque
			Scalar E_ext = -dt * t_ext.dot(w1);

			return E_ext + branch(is_quasistatic > 0.5, 0.0, E_inertia);
		}
	);
}

void stark::EnergyRigidBodyInertia::add(const int rb_idx, const double mass, const Eigen::Matrix3d& inertia_loc)
{
	if (rb_idx != (int)this->mass.size()) {
		std::cout << "Stark error: EnergyRigidBodyInertia::add() found non-consecutive rigid body added." << std::endl;
		exit(-1);
	}

	this->conn.push_back({ (int)this->mass.size() });
	this->mass.push_back(mass);
	this->J_loc.push_back(inertia_loc);
	this->linear_damping.push_back(0.0);
	this->angular_damping.push_back(0.0);
	this->is_quasistatic.push_back(0.0);
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
