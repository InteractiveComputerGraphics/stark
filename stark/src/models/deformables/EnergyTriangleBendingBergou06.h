#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>

#include "../../core/Stark.h"
#include "Id.h"
#include "PointDynamics.h"

namespace stark::models
{
	class EnergyTriangleBendingBergou06
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<6> conn{ {"idx", "group", "i", "j", "k", "l"} };
		std::vector<double> stiffness;  // per group
		std::vector<double> damping;  // per group
		std::vector<std::array<double, 16>> Q_matrix;  // per ie
		std::vector<double> cutoff_angle_deg;  // per group
		std::vector<std::string> labels;  // per group

		/* Methods */
		EnergyTriangleBendingBergou06(stark::core::Stark& stark, spPointDynamics dyn);
		void add(Id& id, const std::vector<std::array<int, 3>>& triangles, const double stiffness, const double damping, const double cutoff_angle_deg, const std::string label = "");
		void set_parameters(Id& id, const double stiffness, const double damping);
	};
	using spEnergyTriangleBendingBergou06 = std::shared_ptr<EnergyTriangleBendingBergou06>;
}
