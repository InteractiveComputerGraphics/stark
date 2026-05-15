#pragma once
#include "../../../core/Stark.h"
#include "../PointDynamics.h"
#include "../../types.h"

namespace stark
{
	class EnergyDiscreteShells
	{
	public:
		/* Types */
		struct Params 
		{ 
			STARK_PARAM_SCALE()
			STARK_PARAM_NO_VALIDATION(bool, flat_rest_angle, false)
			STARK_PARAM_NON_NEGATIVE(double, stiffness, 1e-6)
			STARK_PARAM_DAMPING()
		};
		struct Handler { STARK_COMMON_HANDLER_CONTENTS(EnergyDiscreteShells, Params) };

	private:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<6> conn_complete{ {"idx", "group", "v_edge_0", "v_edge_1", "v_opp_0", "v_opp_1"} };
		symx::LabelledConnectivity<6> conn_flat_rest{ {"idx", "group", "v_edge_0", "v_edge_1", "v_opp_0", "v_opp_1"} };

		// Input
		std::vector<bool> flat_rest_angle;  // per group 
		std::vector<double> scale;  // per group
		std::vector<double> bending_stiffness;  // group
		std::vector<double> bending_damping;  // per group

		// Computed
		std::vector<double> rest_dihedral_angle_rad; // per hinge
		std::vector<double> rest_edge_length; // per hinge
		std::vector<double> rest_height; // per hinge
		std::vector<Eigen::Vector4d> bergou_K; // per hinge
		std::vector<double> bergou_coef; // per hinge

	public:
		/* Methods */
		EnergyDiscreteShells(stark::core::Stark& stark, spPointDynamics dyn);
		Handler add(const PointSetHandler& set, const std::vector<std::array<int, 3>>& triangles, const Params& params);
		Params get_params(const Handler& handler) const;
		void set_params(const Handler& handler, const Params& params);
	};
}
