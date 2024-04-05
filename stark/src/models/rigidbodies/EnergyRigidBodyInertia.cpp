#include "EnergyRigidBodyInertia.h"

#include "rigidbody_transformations.h"

stark::EnergyRigidBodyInertia::EnergyRigidBodyInertia(core::Stark& stark, spRigidBodyDynamics rb)
	: rb(rb)
{
	stark.callbacks.add_before_time_step([&]() { this->_before_time_step(stark); });

	// Linear inertia
	stark.global_energy.add_energy("EnergyRigidBodyInertia_Linear", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector v1 = energy.make_dof_vector(this->rb->dof_v, this->rb->v1, conn["rb"]);
			symx::Vector v0 = energy.make_vector(this->rb->v0, conn["rb"]);
			symx::Vector a = energy.make_vector(this->rb->a, conn["rb"]);
			symx::Vector f = energy.make_vector(this->rb->force, conn["rb"]);
			symx::Scalar m = energy.make_scalar(this->mass, conn["rb"]);
			symx::Scalar damping = energy.make_scalar(this->linear_damping, conn["rb"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);
			symx::Vector gravity = energy.make_vector(stark.gravity);

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
			symx::Vector w1 = energy.make_dof_vector(this->rb->dof_w, this->rb->w1, conn["rb"]);
			symx::Vector w0 = energy.make_vector(this->rb->w0, conn["rb"]);
			symx::Vector aa = energy.make_vector(this->rb->aa, conn["rb"]);
			symx::Vector t = energy.make_vector(this->rb->torque, conn["rb"]);
			symx::Matrix J0_glob = energy.make_matrix(this->J0_glob, { 3, 3 }, conn["rb"]);
			symx::Matrix J0_inv_glob = energy.make_matrix(this->J0_inv_glob, { 3, 3 }, conn["rb"]);
			symx::Scalar damping = energy.make_scalar(this->angular_damping, conn["rb"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector what = w0 + dt * (aa + J0_inv_glob * t);
			symx::Vector dev = w1 - what;
			symx::Scalar E = 0.5 * (dev.dot(J0_glob.dot(dev)) + w1.dot(J0_glob.dot(w1)) * damping * dt);
			energy.set(E);
		}
	);

	// Inverse mass and inertia tensor for acceleration residual
	stark.callbacks.add_inv_mass_application(this->rb->dof_v,
		[&](double* begin, double* end)
		{
			const int n = (int)std::distance(begin, end);
			const int n_bodies = this->rb->get_n_bodies();
			if (n != 3 * n_bodies) {
				std::cout << "Stark error: EnergyRigidBodyInertia::inv_mass_application() found `begin` and `end` with different size than the set nodes." << std::endl;
				exit(-1);
			}

			// Apply inverse mass
			for (int i = 0; i < n_bodies; i++) {
				const double mass_inv = 1.0 / this->mass[i];
				begin[3 * i + 0] *= mass_inv;
				begin[3 * i + 1] *= mass_inv;
				begin[3 * i + 2] *= mass_inv;
			}
		}
	);
	stark.callbacks.add_inv_mass_application(this->rb->dof_w,
		[&](double* begin, double* end)
		{
			const int n = (int)std::distance(begin, end);
			const int n_bodies = this->rb->get_n_bodies();
			if (n != 3 * n_bodies) {
				std::cout << "Stark error: EnergyRigidBodyInertia::inv_mass_application() found `begin` and `end` with different size than the set nodes." << std::endl;
				exit(-1);
			}

			// Apply inverse mass
			for (int i = 0; i < n_bodies; i++) {
				const std::array<double, 9>& J0_inv_arr = this->J0_inv_glob[i];
				Eigen::Matrix3d J0_inv;
				J0_inv << J0_inv_arr[0], J0_inv_arr[1], J0_inv_arr[2],
					J0_inv_arr[3], J0_inv_arr[4], J0_inv_arr[5],
					J0_inv_arr[6], J0_inv_arr[7], J0_inv_arr[8];
				const Eigen::Vector3d torque = { begin[3 * i + 0], begin[3 * i + 1], begin[3 * i + 2] };
				const Eigen::Vector3d aa = J0_inv * torque;
				begin[3 * i + 0] = aa[0];
				begin[3 * i + 1] = aa[1];
				begin[3 * i + 2] = aa[2];
			}
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
