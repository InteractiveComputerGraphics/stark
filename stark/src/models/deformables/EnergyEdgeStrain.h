#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>

#include "../../solver/Stark.h"
#include "Id.h"
#include "PointDynamics.h"

namespace stark::models
{
	class EnergyEdgeStrain
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<4> conn{ {"idx", "group", "i", "j"} };
		std::vector<double> young_modulus;  // per group
		std::vector<double> strain_limit;  // per group
		std::vector<double> strain_limiting_stiffness;  // per group
		std::vector<double> strain_damping;  // per group
		std::vector<double> section_area;  // group
		std::vector<double> rest_length;  // per edge
		std::vector<std::string> labels;  // per group

		/* Methods */
		EnergyEdgeStrain(Stark& stark, spPointDynamics dyn);
		void add(Id& id, const std::vector<std::array<int, 2>>& edges, const double section_radius, const double young_modulus, const double strain_limit, const double strain_limiting_stiffness, const double strain_damping, const std::string label = "");
		void set_parameters(Id& id, const double young_modulus, const double strain_limit, const double strain_limiting_stiffness, const double strain_damping);
	};
	using spEnergyEdgeStrain = std::shared_ptr<EnergyEdgeStrain>;
}
