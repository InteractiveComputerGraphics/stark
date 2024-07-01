#pragma once
#include "../../../core/Stark.h"
#include "../PointDynamics.h"
#include "../../types.h"

namespace stark
{
	class EnergyTetStrain
	{
	public:
		/* Types */
		struct Params
		{
			STARK_PARAM_ELASTICITY_ONLY()
			STARK_PARAM_SCALE()
			STARK_PARAM_YOUNGS_MODULUS()
			STARK_PARAM_POISSONS_RATIO()
			STARK_PARAM_DAMPING()
			STARK_PARAM_STRAIN_LIMITING()
		};
		struct Handler { STARK_COMMON_HANDLER_CONTENTS(EnergyTetStrain, Params) };

	private:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<6> conn_elasticity_only{ { "idx", "group", "i", "j", "k", "l" } };
		symx::LabelledConnectivity<6> conn_complete{ { "idx", "group", "i", "j", "k", "l" } };

		std::vector<bool> elasticity_only;  // per group
		std::vector<double> scale;  // per group
		std::vector<double> youngs_modulus;  // per group
		std::vector<double> poissons_ratio;  // per group
		std::vector<double> strain_damping;  // per group
		std::vector<double> strain_limit;  // per group
		std::vector<double> strain_limit_stiffness;  // per group

	public:
		/* Methods */
		EnergyTetStrain(core::Stark& stark, spPointDynamics dyn);
		Handler add(const PointSetHandler& set, const std::vector<std::array<int, 4>>& tets, const Params& params);
		Params get_params(const Handler& handler) const;
		void set_params(const Handler& handler, const Params& params);
	};
}
