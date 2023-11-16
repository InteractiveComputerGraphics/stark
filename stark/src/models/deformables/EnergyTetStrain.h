#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>

#include "../../solver/Stark.h"
#include "PointDynamics.h"

namespace stark::models
{
	class EnergyTetStrain
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<6> conn{ { "idx", "group", "i", "j", "k", "l" } };
		std::vector<double> young_modulus;  // per group
		std::vector<double> poisson_ratio;  // per group
		std::vector<double> tet_volume_rest;  // per tet
		std::vector<std::array<double, 9>> DXinv;  // per tet
		std::vector<std::string> labels;  // per group

		/* Methods */
		EnergyTetStrain(Stark& stark, spPointDynamics dyn);
		int add(const int obj_idx, const std::vector<std::array<int, 4>>& tets, const double young_modulus, const double poisson_ratio, const std::string label = "");
		void set_parameters(const int id, const double young_modulus, const double poisson_ratio);
	};
	using spEnergyTetStrain = std::shared_ptr<EnergyTetStrain>;
}
