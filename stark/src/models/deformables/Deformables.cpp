#include "Deformables.h"

#include "../../utils/include.h"

stark::Deformables::Deformables(core::Stark& stark, spPointDynamics dyn)
	: point_sets(dyn)
{
	this->output = std::make_shared<DeformablesMeshOutput>(stark, dyn);
	this->lumped_inertia = std::make_shared<EnergyLumpedInertia>(stark, dyn);
	this->prescribed_positions = std::make_shared<EnergyPrescribedPositions>(stark, dyn);
	this->segment_strain = std::make_shared<EnergySegmentStrain>(stark, dyn);
	this->triangle_strain = std::make_shared<EnergyTriangleStrain>(stark, dyn);
	this->discrete_shells = std::make_shared<EnergyDiscreteShells>(stark, dyn);
	this->tet_strain = std::make_shared<EnergyTetStrain>(stark, dyn);
}

