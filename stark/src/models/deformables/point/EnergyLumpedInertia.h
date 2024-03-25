#pragma once
#include "../../../core/Stark.h"
#include "../PointDynamics.h"
#include "../../types.h"

namespace stark
{
	/**
	*   Energy definition and data structure for the lumped mass inertia model.
	* 
	*   Note: Units of "volume" and "density" are not specified (e.g. [m^3] and [kg/m^3]) because it works
	*         with any unit system as long as the multiplication of the two gives the mass in kg.
	*         For example, it can be used with volumetric [m^3] and [kg/m^3], surface [m^2] and [kg/m^2], 
	*         linear [m] and density [kg/m], etc.
	*/
	class EnergyLumpedInertia
	{
	public:
		/* Types */
		struct Params 
		{ 
			STARK_PARAM_NON_NEGATIVE(double, density, 1.0)
			STARK_PARAM_DAMPING() 
		};
		struct Handler
		{
			STARK_COMMON_HANDLER_CONTENTS(EnergyLumpedInertia, Params)
			inline double get_mass() const { return this->model->get_mass(*this); }
		};

	private:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<3> conn{ { "idx", "glob", "group" } };

		std::vector<double> density;  // [kg/V] per group
		std::vector<double> damping; // per group
		std::vector<double> lumped_volume;  // [V] per idx

	public:
		/* Methods */
		EnergyLumpedInertia(stark::core::Stark& stark, const spPointDynamics dyn);
		Handler add(const PointSetHandler& set, const std::vector<int>& points, const std::vector<double>& lumped_volume, const Params& params);
		Handler add(const PointSetHandler& set, const std::vector<double>& lumped_volume, const Params& params);
		Handler add(const PointSetHandler& set, const std::vector<std::array<int, 2>>& edges, const Params& params);
		Handler add(const PointSetHandler& set, const std::vector<std::array<int, 3>>& triangles, const Params& params);
		Handler add(const PointSetHandler& set, const std::vector<std::array<int, 4>>& tets, const Params& params);
		Params get_params(const Handler& idx) const;
		void set_params(const Handler& idx, const Params& params);
		double get_mass(const Handler& idx) const;
	};	
}
