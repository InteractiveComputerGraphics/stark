#include <nanobind/nanobind.h>
namespace nb = nanobind;

void pystark_Simulation(nb::module_& m);

// Deformables
void pystark_PointDynamics(nb::module_& m);
void pystark_PointSetHandler(nb::module_& m);
void pystark_Deformables(nb::module_& m);
void pystark_DeformablesMeshOutput(nb::module_& m);

void pystark_EnergyLumpedInertia(nb::module_& m);
void pystark_EnergyPrescribedPositions(nb::module_& m);
void pystark_EnergySegmentStrain(nb::module_& m);
void pystark_EnergyTriangleStrain(nb::module_& m);
void pystark_EnergyDiscreteShells(nb::module_& m);
void pystark_EnergyTetStrain(nb::module_& m);

// Rigid Bodies
void pystark_RigidBodies(nb::module_& m);
void pystark_RigidBodyDynamics(nb::module_& m);
void pystark_RigidBodyHandler(nb::module_& m);
void pystark_RigidBodiesMeshOutput(nb::module_& m);
void pystark_rigidbody_constraints_ui(nb::module_& m);

// Interactions
void pystark_Interactions(nb::module_& m);
void pystark_EnergyFrictionalContact(nb::module_& m);
void pystark_EnergyAttachments(nb::module_& m);

// Presets
void pystark_Presets(nb::module_& m);
void pystark_DeformablesPresets(nb::module_& m);
void pystark_deformables_preset_types(nb::module_& m);
void pystark_RigidBodyPresets(nb::module_& m);



// =================================================================================================
// =================================================================================================
void pystark_models(nb::module_& m)
{
	pystark_Simulation(m);

	// Deformables
	pystark_PointDynamics(m);
	pystark_PointSetHandler(m);
	pystark_Deformables(m);
	pystark_DeformablesMeshOutput(m);

	pystark_EnergyLumpedInertia(m);
	pystark_EnergyPrescribedPositions(m);
	pystark_EnergySegmentStrain(m);
	pystark_EnergyTriangleStrain(m);
	pystark_EnergyDiscreteShells(m);
	pystark_EnergyTetStrain(m);

	// Rigid Bodies
	pystark_RigidBodies(m);
	pystark_RigidBodyDynamics(m);
	pystark_RigidBodyHandler(m);
	pystark_RigidBodiesMeshOutput(m);
	pystark_rigidbody_constraints_ui(m);

	// Interactions
	pystark_Interactions(m);
	pystark_EnergyFrictionalContact(m);
	pystark_EnergyAttachments(m);

	// Presets
	pystark_Presets(m);
	pystark_DeformablesPresets(m);
	pystark_deformables_preset_types(m);
	pystark_RigidBodyPresets(m);
}
