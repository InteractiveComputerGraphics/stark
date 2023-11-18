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
	class EnergyTriangleStrain
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<5> conn{ { "idx", "group", "i", "j", "k" } };
		std::vector<double> thickness;  // per group
		std::vector<double> young_modulus;  // per group
		std::vector<double> poisson_ratio;  // per group
		std::vector<double> strain_limit;  // per group
		std::vector<double> strain_limiting_stiffness;  // per group
		std::vector<double> triangle_area_rest;  // per triangle
		std::vector<std::array<double, 4>> DXinv;  // per triangle
		std::vector<std::string> labels;  // per group

		/* Methods */
		EnergyTriangleStrain(Stark& stark, spPointDynamics dyn);
		void add(Id& id, const std::vector<std::array<int, 3>>& triangles, const double thickness, const double young_modulus, const double poisson_ratio, const double strain_limit, const double strain_limiting_stiffness, const std::string label = "");
		void set_parameters(Id& id, const double young_modulus, const double poisson_ratio, const double strain_limit, const double strain_limiting_stiffness);
	};
	using spEnergyTriangleStrain = std::shared_ptr<EnergyTriangleStrain>;
}
