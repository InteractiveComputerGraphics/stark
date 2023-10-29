#include "Deformables.h"

#include "../time_integration.h"


void stark::models::Deformables::init(Stark& sim)
{
	this->_init_simulation_structures(sim.settings.execution.n_threads);

	// DoFs
	this->dof = sim.global_energy.add_dof_array(this->v1, "deformables_v1");

	// Callbacks
	sim.callbacks.before_time_step.push_back([&]() { this->_before_time_step(sim); });
	sim.callbacks.after_time_step.push_back([&]() { this->_after_time_step(sim); });
	sim.callbacks.write_frame.push_back([&]() { this->_write_frame(sim); });
	sim.callbacks.before_energy_evaluation.push_back([&]() { this->_update_contacts(sim); });
	sim.callbacks.is_state_valid.push_back([&]() { return this->_is_valid_configuration(sim); });

	// Energy declarations
	this->_potentials_inertia(sim);
}

void stark::models::Deformables::_potentials_inertia(Stark& sim)
{
	// Lumped mass inertia
	sim.global_energy.add_energy("deformables_inertia", this->conn_all_nodes,
		[&](symx::Energy& energy, symx::Element& node)
		{
			//// Create symbols
			symx::Vector v1 = energy.make_dof_vector(this->dof, this->v1.data, node["node"]);
			symx::Vector x0 = energy.make_vector(this->x0.data, node["node"]);
			symx::Vector v0 = energy.make_vector(this->v0.data, node["node"]);
			symx::Vector a = energy.make_vector(this->a.data, node["node"]);
			symx::Scalar mass = energy.make_scalar(this->lumped_mass.data, node["node"]);

			symx::Scalar inertial_damping = energy.make_scalar(this->inertial_damping, node["mesh"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Vector gravity = energy.make_vector(sim.settings.simulation.gravity);

			//// Set energy expression
			symx::Vector x1 = x0 + dt*v1;
			symx::Vector xhat = x0 + dt*v0 + dt*dt*(a + gravity);
			symx::Vector dev = x1 - xhat;
			symx::Vector dev2 = x1 - x0;
			symx::Scalar E = 0.5*mass*(dev.dot(dev)/(dt.powN(2)) + dev2.dot(dev2)*inertial_damping/dt);
			energy.set(E);
		}
	);
}
void stark::models::Deformables::_potentials_boundary_conditions(Stark& sim)
{
	// Prescribed nodes
	sim.global_energy.add_energy("deformables_prescribed_positions", this->conn_prescribed_positions,
		[&](symx::Energy& energy, symx::Element& node)
		{
			//// Create symbols
			symx::Vector v1 = energy.make_dof_vector(this->dof, this->v1.data, node["node"]);
			symx::Vector x0 = energy.make_vector(this->x0.data, node["node"]);
			symx::Vector x1_prescribed = energy.make_vector(this->prescribed_positions, node["idx"]);
			symx::Scalar k = energy.make_scalar(sim.settings.simulation.boundary_conditions_stiffness);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			symx::Vector x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1 - x1_prescribed).squared_norm();
			energy.set(E);
		}
	);

	// Attachments
	sim.global_energy.add_energy("deformables_attached_nodes", this->conn_attached_nodes,
		[&](symx::Energy& energy, symx::Element& node_pair)
		{
			//// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, node_pair);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, node_pair);
			symx::Scalar k = energy.make_scalar(sim.settings.simulation.boundary_conditions_stiffness);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1[0] - x1[1]).squared_norm();
			energy.set(E);
		}
	);
}
void stark::models::Deformables::_potentials_edge_strain_limiting_and_damping(Stark& sim)
{
	// Edge strain limiting
	sim.global_energy.add_energy("deformables_edge_strain_limiting", this->conn_all_edges,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Index> edge = { conn["i"], conn["j"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, edge);
			std::vector<symx::Vector> X = energy.make_vectors(this->X.data, edge);
			symx::Scalar strain_limiting_start = energy.make_scalar(this->strain_limiting_start, conn["mesh"]);
			symx::Scalar strain_limiting_stiffness = energy.make_scalar(this->strain_limiting_stiffness, conn["mesh"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Constraint
			symx::Scalar l_rest = (X[0] - X[1]).norm();
			symx::Scalar l = (x1[0] - x1[1]).norm();
			symx::Scalar dl = l - l_rest;
			symx::Scalar dl_penalty = dl - strain_limiting_start*l_rest;  // absolute stretch that we penalize
			symx::Scalar E = strain_limiting_stiffness * dl_penalty.powN(3)/3.0;
			energy.set_with_condition(E, dl_penalty > 0.0);
		}
	);

	// Edge strain damping
	sim.global_energy.add_energy("deformables_edge_strain_damping", this->conn_all_edges,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Index> edge = { conn["i"], conn["j"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, edge);
			symx::Scalar damping = energy.make_scalar(this->edge_strain_damping, conn["mesh"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Set energy expression
			symx::Scalar l = (x1[1] - x1[0]).norm();
			symx::Scalar l0 = (x0[1] - x0[0]).norm();
			symx::Scalar Energy = 0.5*dt*damping*((l - l0)/dt).powN(2);
			energy.set(Energy);
		}
	);
}
void stark::models::Deformables::_potentials_mechanics_rods(Stark& sim)
{
	// Edge strain limiting
	sim.global_energy.add_energy("deformables_rod_segment_strain", this->conn_rod_segments,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Index> edge = { conn["i"], conn["j"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, edge);
			std::vector<symx::Vector> X = energy.make_vectors(this->X.data, edge);
			symx::Scalar strain_stiffness = energy.make_scalar(this->rods_strain_stiffness, conn["mesh"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Constraint
			symx::Scalar l_rest = (X[0] - X[1]).norm();
			symx::Scalar l = (x1[0] - x1[1]).norm();
			symx::Scalar dl = l - l_rest;
			symx::Scalar E = strain_stiffness * dl.powN(2) / 2.0;
			energy.set(E);
		}
	);
}
void stark::models::Deformables::_potentials_mechanics_surface(Stark& sim)
{
	// Triangle strain
	sim.global_energy.add_energy("deformables_surfaces_triangle_strain", this->conn_surface_triangles,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> triangle = conn.slice(1, 4);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, triangle);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, triangle);
			std::vector<symx::Vector> X = energy.make_vectors(this->X.data, triangle);
			symx::Scalar E = energy.make_scalar(this->surfaces_young_modulus, conn["mesh"]);
			symx::Scalar nu = energy.make_scalar(this->surfaces_poisson_ratio, conn["mesh"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Kinematics
			//// Jacobian at rest configuration (needs projection to triangle)
			symx::Vector u = (X[1] - X[0]).normalized();
			symx::Vector n = u.cross3(X[2] - X[0]);
			symx::Vector v = u.cross3(n).normalized();
			symx::Matrix P = symx::Matrix(symx::gather({ u, v }), { 2, 3 });
			std::vector<symx::Vector> X_ = { P*X[0], P*X[1], P*X[2] };
			symx::Matrix DX = symx::Matrix(symx::gather({ X_[1] - X_[0], X_[2] - X_[0] }), { 2, 2 }).transpose();

			//// Jacobian at current configuration
			symx::Matrix Dx_32 = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0] }), { 2, 3 }).transpose();
			
			//// Deformation gradient
			symx::Matrix F_32 = Dx_32 * DX.inv();  // 3x2
			symx::Matrix C = F_32.transpose() * F_32;

			// Stable Neo-Hookean strain energy
			symx::Scalar mu = E / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (E * nu) / ((1.0 + nu) * (1.0 - nu));  // 2D !!
			symx::Scalar rest_area = 0.5 * ((X[0] - X[2]).cross3(X[1] - X[2])).norm();
			symx::Scalar area = 0.5 * ((x1[0] - x1[2]).cross3(x1[1] - x1[2])).norm();
			symx::Scalar J = area / rest_area;
			symx::Scalar Ic = C.trace();
			symx::Scalar logJ = symx::log(J);
			symx::Scalar energy_density = 0.5 * mu * (Ic - 3.0) - mu * logJ + 0.5 * lambda * logJ.powN(2);
			symx::Scalar Energy = area * energy_density;
			energy.set(Energy);
		}
	);
}
