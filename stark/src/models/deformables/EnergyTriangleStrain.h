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
	class EnergyTriangleStrain :
		public Energy
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<5> conn{ { "idx", "group", "i", "j", "k" } };
		std::vector<double> young_modulus;  // per group
		std::vector<double> poisson_ratio;  // per group
		std::vector<double> triangle_area_rest;  // per triangle
		std::vector<std::array<double, 4>> DXinv;  // per triangle
		std::vector<std::string> labels;  // per group

		/* Methods */
		EnergyTriangleStrain(spPointDynamics dyn);
		void declare(Stark& stark);
		int add(const std::vector<std::array<int, 3>>& triangles, const int conn_offset, const double young_modulus, const double poisson_ratio, const std::string label = "");
		void set_parameters(const int id, const double young_modulus, const double poisson_ratio);
	};
	using spEnergyTriangleStrain = std::shared_ptr<EnergyTriangleStrain>;
}
