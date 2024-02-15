#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>

#include "../../core/Stark.h"
#include "PointDynamics.h"
#include "Id.h"

namespace stark::models
{
	class EnergyTetStrain
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<6> conn{ { "idx", "group", "i", "j", "k", "l" } };

		// Input
		std::vector<double> young_modulus;  // per group
		std::vector<double> poisson_ratio;  // per group
		std::vector<double> damping;  // per group
		std::vector<double> strain_limit;  // per group
		std::vector<double> strain_limit_stiffness;  // per group

		// Computed
		std::vector<double> tet_volume_rest;  // per tet
		std::vector<std::array<double, 9>> DXinv;  // per tet

		/* Methods */
		EnergyTetStrain(stark::core::Stark& stark, spPointDynamics dyn);
		void add(Id& id, const std::vector<std::array<int, 4>>& tets, const double young_modulus, const double poisson_ratio, const double damping, const double strain_limit, const double strain_limi_stiffness);

		void set_young_modulus(const Id& id, const double young_modulus);
		void set_poisson_ratio(const Id& id, const double poisson_ratio);
		void set_strain_damping(const Id& id, const double strain_damping);
		void set_strain_limit(const Id& id, const double strain_limit);
		void set_strain_limit_stiffness(const Id& id, const double set_strain_limit_stiffness);

		double get_young_modulus(const Id& id);
		double get_poisson_ratio(const Id& id);
		double get_strain_damping(const Id& id);
		double get_strain_limit(const Id& id);
		double get_strain_limit_stiffness(const Id& id);

		int get_index(const Id& id) const;
	};
	using spEnergyTetStrain = std::shared_ptr<EnergyTetStrain>;
}
