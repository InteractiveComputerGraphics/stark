#include "EnergyPointLumpedMassAndDamping.h"

stark::models::EnergyPointLumpedMassAndDamping::EnergyPointLumpedMassAndDamping(spPointDynamics dyn)
	: dyn(dyn)
{
}

void stark::models::EnergyPointLumpedMassAndDamping::declare(Stark& sim)
{
	sim.global_energy.add_energy("PointLumpedMassAndDamping", this->nodes_conn,
		[&](symx::Energy& energy, symx::Element& node)
		{
			PointDynamics& dyn = *(this->dyn);

			//// Create symbols
			symx::Vector v1 = energy.make_dof_vector(dyn.dof, dyn.v1.data, node["node"]);
			symx::Vector x0 = energy.make_vector(dyn.x0.data, node["node"]);
			symx::Vector v0 = energy.make_vector(dyn.v0.data, node["node"]);
			symx::Vector a = energy.make_vector(dyn.a.data, node["node"]);
			symx::Scalar mass = energy.make_scalar(this->nodes_lumped_mass.data, node["node"]);

			symx::Scalar inertial_damping = energy.make_scalar(this->nodes_inertial_damping, node["mesh"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Vector gravity = energy.make_vector(sim.settings.simulation.gravity);

			//// Set energy expression
			symx::Vector x1 = x0 + dt * v1;
			symx::Vector xhat = x0 + dt * v0 + dt * dt * (a + gravity);
			symx::Vector dev = x1 - xhat;
			symx::Vector dev2 = x1 - x0;
			symx::Scalar E = 0.5 * mass * (dev.dot(dev) / (dt.powN(2)) + dev2.dot(dev2) * inertial_damping / dt);
			energy.set(E);
		}
	);
}
