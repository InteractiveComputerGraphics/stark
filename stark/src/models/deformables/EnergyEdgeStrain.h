#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>

#include "../../solver/Energy.h"
#include "../../solver/Stark.h"
#include "PointDynamics.h"

namespace stark::models
{
	class EnergyEdgeStrain :
		public Energy
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<4> conn{ {"idx", "group", "i", "j"} };
		std::vector<double> strain_stiffness;  // per group
		std::vector<double> strain_limiting_start;  // per group
		std::vector<double> strain_limiting_stiffness;  // per group
		std::vector<double> strain_damping;  // per group
		std::vector<double> rest_length;  // per edge
		std::vector<std::string> labels;  // per group

		/* Methods */
		EnergyEdgeStrain(spPointDynamics dyn);
		void declare(Stark& stark);
		int add(const int obj_idx, const std::vector<std::array<int, 2>>& edges, const double strain_stiffness, const double strain_limiting_start, const double strain_limiting_stiffness, const double strain_damping, const std::string label = "");
		void set_parameters(const int id, const double strain_stiffness, const double strain_limiting_start, const double strain_limiting_stiffness, const double strain_damping);
	};
	using spEnergyEdgeStrain = std::shared_ptr<EnergyEdgeStrain>;
}
