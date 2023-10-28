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
