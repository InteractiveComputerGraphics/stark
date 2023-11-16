#include "Simulation.h"

#include "deformables/Shells.h"
#include "RigidBodies.h"


stark::models::Simulation::Simulation(const Settings& settings)
	: stark(settings)
{
	/* --------------------------- Old --------------------------- */
	this->cloth.init(this->stark);
	this->deformables.init(this->stark);
	this->rigid_bodies.init(this->stark);
	this->interactions.init(this->stark, &this->cloth, &this->rigid_bodies);

	/* --------------------------- New --------------------------- */
	// Base
	spPointDynamics point_dynamics = std::make_shared<PointDynamics>(this->stark);
	spRigidBodies rb = std::make_shared<RigidBodies>(this->stark);

	// Energies
	//// Point
	auto energy_point_inertia = std::make_shared<EnergyPointInertia>(this->stark, point_dynamics);
	auto energy_point_prescribed_positions = std::make_shared<EnergyPointPrescribedPositions>(this->stark, point_dynamics);
	
	//// Edge
	auto energy_edge_strain = std::make_shared<EnergyEdgeStrain>(this->stark, point_dynamics);

	//// Triangle
	auto energy_triangle_strain = std::make_shared<EnergyTriangleStrain>(this->stark, point_dynamics);
	auto energy_triangle_bending_bergou06 = std::make_shared<EnergyTriangleBendingBergou06>(this->stark, point_dynamics);
	
	//// Other
	auto energy_frictional_contact = std::make_shared<EnergyFrictionalContact>(this->stark, point_dynamics, rb);


	// Physical Systems
	this->shells = std::make_shared<Shells>(
		this->stark,
		point_dynamics,
		energy_point_inertia,
		energy_point_prescribed_positions,
		energy_triangle_strain,
		energy_triangle_bending_bergou06,
		energy_edge_strain,
		energy_frictional_contact);
}