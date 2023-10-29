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
	sim.global_energy.add_energy("deformables_inertia", this->nodes_conn,
		[&](symx::Energy& energy, symx::Element& node)
		{
			//// Create symbols
			symx::Vector v1 = energy.make_dof_vector(this->dof, this->v1.data, node["node"]);
			symx::Vector x0 = energy.make_vector(this->x0.data, node["node"]);
			symx::Vector v0 = energy.make_vector(this->v0.data, node["node"]);
			symx::Vector a = energy.make_vector(this->a.data, node["node"]);
			symx::Scalar mass = energy.make_scalar(this->nodes_lumped_mass.data, node["node"]);

			symx::Scalar inertial_damping = energy.make_scalar(this->nodes_inertial_damping, node["mesh"]);
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
	sim.global_energy.add_energy("deformables_prescribed_positions", this->prescribed_positions_conn,
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
	sim.global_energy.add_energy("deformables_attached_nodes", this->attached_nodes_conn,
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
	sim.global_energy.add_energy("deformables_edge_strain_limiting", this->edges_conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Index> edge = { conn["i"], conn["j"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, edge);
			std::vector<symx::Vector> X = energy.make_vectors(this->X.data, edge);
			symx::Scalar strain_limiting_start = energy.make_scalar(this->edges_strain_limiting_start, conn["mesh"]);
			symx::Scalar strain_limiting_stiffness = energy.make_scalar(this->edges_strain_limiting_stiffness, conn["mesh"]);
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
	sim.global_energy.add_energy("deformables_edge_strain_damping", this->edges_conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Index> edge = { conn["i"], conn["j"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, edge);
			symx::Scalar damping = energy.make_scalar(this->edges_strain_damping, conn["mesh"]);
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
	sim.global_energy.add_energy("deformables_rod_segment_strain", this->rods_conn,
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
void stark::models::Deformables::_potentials_mechanics_shells(Stark& sim)
{
	// Triangle strain
	sim.global_energy.add_energy("deformables_shells_triangle_strain", this->shells_conn_triangles,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> triangle = conn.slice(2, 5);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, triangle);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, triangle);
			symx::Matrix DXinv = energy.make_matrix(this->shells_triangle_DXinv, { 2, 2 }, conn["tri"]);
			symx::Scalar rest_area = energy.make_scalar(this->shells_triangle_rest_area, conn["tri"]);
			symx::Scalar E = energy.make_scalar(this->shells_young_modulus, conn["mesh"]);
			symx::Scalar nu = energy.make_scalar(this->shells_poisson_ratio, conn["mesh"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Kinematics
			symx::Matrix Dx_32 = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0] }), { 2, 3 }).transpose();
			symx::Matrix F_32 = Dx_32 * DXinv;  // 3x2
			symx::Matrix C = F_32.transpose() * F_32;

			// Stable Neo-Hookean strain energy
			symx::Scalar mu = E / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (E * nu) / ((1.0 + nu) * (1.0 - nu));  // 2D !!
			symx::Scalar area = 0.5 * ((x1[0] - x1[2]).cross3(x1[1] - x1[2])).norm();
			symx::Scalar J = area / rest_area;
			symx::Scalar Ic = C.trace();
			symx::Scalar logJ = symx::log(J);
			symx::Scalar energy_density = 0.5 * mu * (Ic - 3.0) - mu * logJ + 0.5 * lambda * logJ.powN(2);
			symx::Scalar Energy = area * energy_density;
			energy.set(Energy);
		}
	);

	// Bergou06 Bending Energy
	sim.global_energy.add_energy("deformables_shells_bending", this->shells_conn_surface_internal_edges,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> internal_edge = conn.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, internal_edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, internal_edge);
			symx::Matrix Q = energy.make_matrix(this->shells_bergou_Q_matrix, { 4, 4 }, conn["ie"]);
			symx::Scalar stiffness = energy.make_scalar(this->shells_bending_stiffness, conn["mesh"]);
			symx::Scalar damping = energy.make_scalar(this->shells_bending_damping, conn["mesh"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = dt.get_zero();
			for (int i = 0; i < 3; i++) {
				symx::Vector x = symx::Vector({ x1[0][i], x1[1][i], x1[2][i], x1[3][i] });
				symx::Vector v = symx::Vector({ v1[0][i], v1[1][i], v1[2][i], v1[3][i] });
				E += stiffness * (x.transpose() * Q * x) + 0.5 * damping * dt * (v.transpose() * Q * v);
			}
			energy.set(E);
		}
	);
}
void stark::models::Deformables::_potentials_mechanics_volumetrics(Stark& sim)
{
	// Stable Neo-Hookean strain
	sim.global_energy.add_energy("deformables_volumetrics_tet_strain", this->volumetrics_conn_tets,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> tet = conn.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dof, this->v1.data, tet);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->x0.data, tet);
			symx::Matrix DXinv = energy.make_matrix(this->volumetrics_tet_DXinv, { 3, 3 }, conn["tet"]);
			symx::Scalar tet_rest_volume = energy.make_scalar(this->volumetrics_tet_rest_volume, conn["tet"]);
			symx::Scalar E = energy.make_scalar(this->volumetrics_young_modulus, conn["mesh"]);
			symx::Scalar nu = energy.make_scalar(this->volumetrics_poisson_ratio, conn["mesh"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Kinematics
			symx::Matrix Dx = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0], x1[3] - x1[0] }), { 3, 3 }).transpose();
			symx::Matrix F = Dx * DXinv;

			// [Smith et al. 2022] Stable Neo-Hookean Flesh Simulation
			// Eq. 49 from [Smith et al. 2022]
			symx::Scalar mu = E / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (E * nu) / ((1.0 + nu) * (1.0 - 2.0 * nu)); // 3D
			symx::Scalar mu_ = 4.0/3.0*mu;
			symx::Scalar lambda_ = lambda + 5.0/6.0*mu;
			symx::Scalar detF = F.det();
			symx::Scalar Ic = F.frobenius_norm_sq();
			symx::Scalar alpha = 1.0 + mu_ / lambda_ - mu_ / (4.0 * lambda_);
			symx::Scalar energy_density = 0.5*mu_*(Ic - 3.0) + 0.5*lambda_*(detF - alpha).powN(2) - 0.5*mu_*symx::log(Ic + 1.0);
			symx::Scalar Energy = energy_density * tet_rest_volume;
			energy.set(Energy);
		}
	);
}
