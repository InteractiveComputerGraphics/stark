#pragma once
#include <vector>
#include <string>

#include "../../solver/Energy.h"
#include "PointDynamics.h"

namespace stark::models
{
	class EnergyPointLumpedMassAndDamping :
		public Energy
	{
	public:
		/* Fields */
		spPointDynamics dyn;
		std::vector<std::string> labels;  // Set identifiers
		symx::LabelledConnectivity<2> conn{ { "mesh", "node" } };
		IntervalVector<double> lumped_mass;  // [kg] per vertex
		std::vector<double> inertial_damping; // per mesh

		/* Methods */
		EnergyPointLumpedMassAndDamping(spPointDynamics dyn);
		void declare(Stark& sim);
		int add(const int begin_idx, const int end_idx, const std::vector<double>& lumped_mass, const double inertial_damping);
		void update(const int id, const int begin_idx, const int end_idx, const std::vector<double>& lumped_mass, const double inertial_damping);
	};
}
