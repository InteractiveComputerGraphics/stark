#pragma once
#include <array>
#include <vector>
#include <string>

#include "../../core/Stark.h"
#include "Id.h"
#include "PointDynamics.h"

namespace stark::models
{
	class EnergyPointInertia
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<3> conn{ { "idx", "glob", "obj" } };
		IntervalVector<double> lumped_volume;  // [kg] per vertex
		std::vector<double> density;  // [kg/m3] per vertex
		std::vector<double> inertial_damping; // per obj
		std::vector<std::string> labels;  // per obj

		/* Methods */
		EnergyPointInertia(stark::core::Stark& stark, const spPointDynamics dyn);
		void add(Id& id, const std::vector<double>& lumped_volume, const double density, const double inertial_damping, const std::string label = "");
		void add(Id& id, const std::vector<std::array<int, 2>>& edges, const double line_density, const double inertial_damping, const std::string label = "");
		void add(Id& id, const std::vector<std::array<int, 3>>& triangles, const double area_density, const double inertial_damping, const std::string label = "");
		void add(Id& id, const std::vector<std::array<int, 4>>& tets, const double volume_density, const double inertial_damping, const std::string label = "");
		void update(Id& id, const std::vector<double>& lumped_volume, const double density, const double inertial_damping);
	};
	using spEnergyPointInertia = std::shared_ptr<EnergyPointInertia>;
}
