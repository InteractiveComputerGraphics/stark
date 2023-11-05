#pragma once
#include <vector>
#include <string>

#include "../../solver/Energy.h"
#include "../../solver/Stark.h"
#include "PointDynamics.h"

namespace stark::models
{
	class EnergyPointLumpedMassAndDamping :
		public Energy
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<3> conn{ { "glob", "loc", "obj" } };
		IntervalVector<double> lumped_mass;  // [kg] per vertex
		std::vector<double> inertial_damping; // per obj
		std::vector<std::string> labels;  // per obj

		/* Methods */
		EnergyPointLumpedMassAndDamping(const spPointDynamics dyn);
		void declare(Stark& stark);
		int add(const std::array<int, 2>& node_range, const std::vector<double>& lumped_mass, const double inertial_damping, const std::string label = "");
		void update(const int id, const std::array<int, 2>& node_range, const std::vector<double>& lumped_mass, const double inertial_damping);
	};
	using spEnergyPointLumpedMassAndDamping = std::shared_ptr<EnergyPointLumpedMassAndDamping>;
}
