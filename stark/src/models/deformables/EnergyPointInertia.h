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

		// Inputs
		std::vector<double> density;  // [kg/V] per obj
		std::vector<double> inertia_damping; // per obj
		
		// Computed
		IntervalVector<double> lumped_volume;  // [V] per vertex
		std::vector<Eigen::Matrix3d> inv_mass;

		/* Methods */
		EnergyPointInertia(stark::core::Stark& stark, const spPointDynamics dyn);
		void add(Id& id, const std::vector<double>& lumped_volume, const double density, const double inertial_damping);
		void add(Id& id, const std::vector<std::array<int, 2>>& edges, const double line_density, const double inertial_damping);
		void add(Id& id, const std::vector<std::array<int, 3>>& triangles, const double area_density, const double inertial_damping);
		void add(Id& id, const std::vector<std::array<int, 4>>& tets, const double volume_density, const double inertial_damping);

		void set_density(const Id& id, const double density);
		void set_inertia_damping(const Id& id, const double inertia_damping);

		double get_density(const Id& id);
		double get_inertia_damping(const Id& id);

		int get_index(const Id& id) const;
	};
	using spEnergyPointInertia = std::shared_ptr<EnergyPointInertia>;
}
