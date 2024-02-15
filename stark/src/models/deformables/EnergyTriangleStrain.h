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
	class EnergyTriangleStrain
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<5> conn{ { "idx", "group", "i", "j", "k" } };

		// Input
		std::vector<double> thickness;  // per group
		std::vector<double> young_modulus;  // per group
		std::vector<double> poisson_ratio;  // per group
		std::vector<double> strain_damping;  // per group
		std::vector<double> strain_limit;  // per group
		std::vector<double> strain_limit_stiffness;  // per group

		// Computed
		std::vector<double> triangle_area_rest;  // per triangle
		std::vector<std::array<double, 4>> DXinv;  // per triangle

		/* Methods */
		EnergyTriangleStrain(stark::core::Stark& stark, spPointDynamics dyn);
		void add(Id& id, const std::vector<std::array<int, 3>>& triangles, const double thickness, const double young_modulus, const double poisson_ratio, const double strain_damping, const double strain_limit, const double set_strain_limit_stiffness);

		void set_thickness(const Id& id, const double thickness);
		void set_young_modulus(const Id& id, const double young_modulus);
		void set_poisson_ratio(const Id& id, const double poisson_ratio);
		void set_strain_damping(const Id& id, const double strain_damping);
		void set_strain_limit(const Id& id, const double strain_limit);
		void set_strain_limit_stiffness(const Id& id, const double set_strain_limit_stiffness);

		double get_thickness(const Id& id);
		double get_young_modulus(const Id& id);
		double get_poisson_ratio(const Id& id);
		double get_strain_damping(const Id& id);
		double get_strain_limit(const Id& id);
		double get_strain_limit_stiffness(const Id& id);

		int get_index(const Id& id) const;
	};
	using spEnergyTriangleStrain = std::shared_ptr<EnergyTriangleStrain>;
}
