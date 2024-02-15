#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>

#include "../../core/Stark.h"
#include "PointDynamics.h"
#include "Id.h"
#include "PrescribedPointGroup.h"
#include "PrescribedPointGroupWithTransformation.h"


namespace stark::models
{
	class EnergyPointPrescribedPositions
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
		EnergyPointPrescribedPositions(stark::core::Stark& stark, spPointDynamics dyn);
		std::shared_ptr<PrescribedPointGroup> create_group(Id& id, const std::string label = "");
		std::shared_ptr<PrescribedPointGroupWithTransformation> create_group_with_transformation(Id& id, const std::string label = "");

	private:
		void _before_time_step(stark::core::Stark& stark);
	};
	using spEnergyPointPrescribedPositions = std::shared_ptr<EnergyPointPrescribedPositions>;
}
