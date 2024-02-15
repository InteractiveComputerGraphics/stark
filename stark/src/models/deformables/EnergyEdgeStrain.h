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
	class EnergyEdgeStrain
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<4> conn{ {"idx", "group", "i", "j"} };

		// Input
		std::vector<double> section_area;  // group
		std::vector<double> young_modulus;  // per group
		std::vector<double> strain_damping;  // per group
		std::vector<double> strain_limit;  // per group
		std::vector<double> strain_limit_stiffness;  // per group

		// Computed
		std::vector<double> rest_length;  // per edge

		/* Methods */
		EnergyEdgeStrain(stark::core::Stark& stark, spPointDynamics dyn);
		void add(Id& id, const std::vector<std::array<int, 2>>& edges, const double section_radius, const double young_modulus, const double strain_limit, const double strain_limit_stiffness, const double strain_damping);
	
		void set_radius(const Id& id, const double radius);
		void set_young_modulus(const Id& id, const double young_modulus);
		void set_strain_damping(const Id& id, const double strain_damping);
		void set_strain_limit(const Id& id, const double strain_limit);
		void set_strain_limit_stiffness(const Id& id, const double set_strain_limit_stiffness);

		double get_radius(const Id& id);
		double get_young_modulus(const Id& id);
		double get_strain_damping(const Id& id);
		double get_strain_limit(const Id& id);
		double get_strain_limit_stiffness(const Id& id);

		int get_index(const Id& id) const;
	};
	using spEnergyEdgeStrain = std::shared_ptr<EnergyEdgeStrain>;
}
