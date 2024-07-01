#pragma once
#include "DeformablesMeshOutput.h"
#include "deformables_energies_include.h"

namespace stark
{
	class Deformables
	{
	public:
		/* Methods */
		Deformables(core::Stark& stark, spPointDynamics dyn);

		/* Fields */
		std::shared_ptr<DeformablesMeshOutput> output;

		// Models
		spPointDynamics point_sets;
		std::shared_ptr<EnergyLumpedInertia> lumped_inertia;
		std::shared_ptr<EnergyPrescribedPositions> prescribed_positions;
		std::shared_ptr<EnergySegmentStrain> segment_strain;
		std::shared_ptr<EnergyTriangleStrain> triangle_strain;
		std::shared_ptr<EnergyDiscreteShells> discrete_shells;
		std::shared_ptr<EnergyTetStrain> tet_strain;
	};
}
