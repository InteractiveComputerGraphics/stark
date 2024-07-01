#pragma once
#include <unordered_map>

#include "../../../core/Stark.h"
#include "../PointDynamics.h"
#include "../../types.h"


namespace stark
{
	/**
	*   Energy definition and data structure for the penalty based point prescribed positions used as boundary conditions.
	*   It uses an additional "tolerance" parameter to define the maximum distance between the prescribed and the actual position.
	*   If the violation is larger than the tolerance, the penalty bending_stiffness is increased.
	*/
	class EnergyPrescribedPositions
	{
	public:
		/* Types */
		struct Params 
		{ 
			STARK_PARAM_STIFFNESS() 
			STARK_PARAM_TOLERANCE() 
		};
		struct Handler
		{
			STARK_COMMON_HANDLER_CONTENTS(EnergyPrescribedPositions, Params)
			inline void set_transformation(const Eigen::Vector3d& t, const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity())
			{
				this->get_model()->set_transformation(*this, t, R);
			}
			inline void set_transformation(const Eigen::Vector3d& t, const double angle_deg, const Eigen::Vector3d& axis)
			{
				this->get_model()->set_transformation(*this, t, angle_deg, axis);
			}
			inline void set_target_position(int prescribed_idx, const Eigen::Vector3d& t)
			{
				this->get_model()->set_target_position(*this, prescribed_idx, t);
			}
		};

	private:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<3> conn{ { "idx", "point", "group" }};

		std::vector<Eigen::Vector3d> target_positions; // per point
		std::vector<double> stiffness; // per group
		std::vector<double> tolerance; // per group

		std::vector<std::array<int, 2>> group_begin_end; // per group  (for tolerance checking purposes)
		std::vector<Eigen::Vector3d> rest_positions; // per point  (for transformation purposes)

	public:
		/* Methods */
		EnergyPrescribedPositions(core::Stark& stark, spPointDynamics dyn);
		Handler add(const PointSetHandler& set, const std::vector<int>& points, const Params& params);
		Handler add_inside_aabb(const PointSetHandler& set, const Eigen::Vector3d& aabb_center, const Eigen::Vector3d& aabb_dim, const Params& params);
		Handler add_outside_aabb(const PointSetHandler& set, const Eigen::Vector3d& aabb_center, const Eigen::Vector3d& aabb_dim, const Params& params);
		Params get_params(const Handler& handler) const;
		void set_params(const Handler& handler, const Params& params);
		void set_transformation(const Handler& handler, const Eigen::Vector3d& t, const Eigen::Matrix3d& R);
		void set_transformation(const Handler& handler, const Eigen::Vector3d& t, const double angle_deg, const Eigen::Vector3d& axis);
		void set_target_position(const Handler& handler, int prescribed_idx, const Eigen::Vector3d& t);

	private:
		bool _is_converged_state_valid(core::Stark& stark);
	};
}

