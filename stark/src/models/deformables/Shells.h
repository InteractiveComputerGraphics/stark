#pragma once

#include "../../solver/Stark.h"
#include "interval_types.h"
#include "PointDynamics.h"
#include "EnergyPointInertia.h"
#include "EnergyPointPrescribedPositions.h"
#include "EnergyTriangleStrain.h"
#include "EnergyTriangleBendingBergou06.h"
#include "EnergyEdgeStrain.h"
#include "EnergyFrictionalContact.h"


namespace stark::models
{
	/*
		This class is exposed to the user.
	*/
	class Shells
	{
	public:
		/* Fields */
		spPointDynamics dyn;
		spEnergyPointInertia inertia;
		spEnergyPointPrescribedPositions prescribed_positions;
		spEnergyTriangleStrain strain;
		spEnergyTriangleBendingBergou06 bending_bergou;
		spEnergyEdgeStrain edge_strain_limiting_and_damping;
		spEnergyFrictionalContact contact;

		/* Methods */
		Shells(
			Stark& stark, 
			spPointDynamics dyn, 
			spEnergyPointInertia inertia,
			spEnergyPointPrescribedPositions prescribed_positions,
			spEnergyTriangleStrain strain,
			spEnergyTriangleBendingBergou06 bending_bergou,
			spEnergyEdgeStrain edge_strain,
			spEnergyFrictionalContact contact
		);

		int add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles);
	};
}
