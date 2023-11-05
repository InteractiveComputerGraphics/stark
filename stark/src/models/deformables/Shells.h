#pragma once

#include "../../solver/Stark.h"
#include "PointDynamics.h"
#include "EnergyPointLumpedMassAndDamping.h"
#include "interval_types.h"


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
		spEnergyPointLumpedMassAndDamping lumped_mass_and_damping;

		/* Methods */
		Shells(Stark& stark, spPointDynamics dyn, spEnergyPointLumpedMassAndDamping lumped_mass_and_damping);

		int add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles);
	};
}
