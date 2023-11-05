#include "EnergyPointLumpedMassAndDamping.h"

stark::models::EnergyPointLumpedMassAndDamping::EnergyPointLumpedMassAndDamping(const spPointDynamics dyn)
	: dyn(dyn)
{
}

void stark::models::EnergyPointLumpedMassAndDamping::declare(Stark& stark)
{
	stark.global_energy.add_energy("PointLumpedMassAndDamping", this->conn,
		[&](symx::Energy& energy, symx::Element& node)
		{
			PointDynamics& dyn = *(this->dyn);

			//// Create symbols
			symx::Vector v1 = energy.make_dof_vector(dyn.dof, dyn.v1.data, node["glob"]);
			symx::Vector x0 = energy.make_vector(dyn.x0.data, node["glob"]);
			symx::Vector v0 = energy.make_vector(dyn.v0.data, node["glob"]);
			symx::Vector a = energy.make_vector(dyn.a.data, node["glob"]);
			symx::Scalar mass = energy.make_scalar(this->lumped_mass.data, node["loc"]);
			symx::Scalar inertial_damping = energy.make_scalar(this->inertial_damping, node["obj"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
			symx::Vector gravity = energy.make_vector(stark.settings.simulation.gravity);

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

int stark::models::EnergyPointLumpedMassAndDamping::add(const std::array<int, 2>& node_range, const std::vector<double>& lumped_mass, const double inertial_damping, const std::string label)
{
	const int set_id = this->lumped_mass.append(lumped_mass);
	this->inertial_damping.push_back(inertial_damping);
	this->labels.push_back(label);
	return set_id;
}
void stark::models::EnergyPointLumpedMassAndDamping::update(const int id, const std::array<int, 2>& node_range, const std::vector<double>& lumped_mass, const double inertial_damping)
{
	this->lumped_mass.update(id, lumped_mass);
	this->inertial_damping[id] = inertial_damping;
}
