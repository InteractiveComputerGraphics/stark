#pragma once
#include <vector>
#include <string>
#include <functional>
#include <memory>

#include <Eigen/Dense>

#include "../../solver/Energy.h"
#include "../../solver/Stark.h"
#include "PointDynamics.h"
#include "PrescribedPointGroup.h"
#include "PrescribedPointGroupWithTransformation.h"

namespace stark::models
{
	class EnergyPointPrescribedPositions :
		public Energy
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<3> conn{ { "idx", "point", "group" }};
		std::vector<Eigen::Vector3d> target_positions; // per point
		std::vector<double> stiffness; // per group
		std::vector<std::string> labels;  // per group

		// Source data (visible to user) to generate the target positions each time step
		std::vector<std::shared_ptr<PrescribedPointGroup>> bc_source;
		std::vector<std::shared_ptr<PrescribedPointGroupWithTransformation>> bc_transform_source;

		/* Methods */
		EnergyPointPrescribedPositions(spPointDynamics dyn);
		void declare(Stark& stark);
		std::shared_ptr<PrescribedPointGroup> add_group(const int obj_idx, const std::string label = "");
		std::shared_ptr<PrescribedPointGroupWithTransformation> add_group_with_transformation(const int obj_idx, const std::string label = "");

	private:
		void _before_time_step(Stark& stark);
	};
	using spEnergyPointPrescribedPositions = std::shared_ptr<EnergyPointPrescribedPositions>;
}
