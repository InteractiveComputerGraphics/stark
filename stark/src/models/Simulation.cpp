#include "Simulation.h"


stark::models::Simulation::Simulation(const Settings& settings)
	: stark(settings)
{
	// Base definitions
	spPointDynamics point_dynamics = std::make_shared<PointDynamics>(this->stark);
	spRigidBodyDynamics rb_dynamics = std::make_shared<RigidBodyDynamics>(this->stark);

	// Common energies
	auto energy_point_inertia = std::make_shared<EnergyPointInertia>(this->stark, point_dynamics);
	auto energy_point_prescribed_positions = std::make_shared<EnergyPointPrescribedPositions>(this->stark, point_dynamics);
	//auto energy_frictional_contact = std::make_shared<EnergyFrictionalContact>(this->stark, point_dynamics, rb);

	// Physical Systems
	this->lines = std::make_shared<OneDimensionalDeformableSolids>(
		this->stark,
		point_dynamics,
		energy_point_inertia,
		energy_point_prescribed_positions
		//energy_frictional_contact
		);
	this->surfaces = std::make_shared<SurfaceDeformableSolids>(
		this->stark,
		point_dynamics,
		energy_point_inertia,
		energy_point_prescribed_positions
		//energy_frictional_contact
		);
	this->volumes = std::make_shared<VolumetricDeformableSolids>(
		this->stark,
		point_dynamics,
		energy_point_inertia,
		energy_point_prescribed_positions
		//energy_frictional_contact
		);
	this->rigidbodies = std::make_shared<RigidBodies>(
		this->stark,
		rb_dynamics
		);
}