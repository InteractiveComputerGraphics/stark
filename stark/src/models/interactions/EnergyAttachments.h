#pragma once
#include "../deformables/PointDynamics.h"
#include "../types.h"
#include "../rigidbodies/RigidBodyDynamics.h"
#include "../rigidbodies/RigidBodyHandler.h"

namespace stark
{
	class EnergyAttachments
	{
	public:
		/* Types */
		struct Params 
		{
			STARK_PARAM_STIFFNESS()
			STARK_PARAM_TOLERANCE()
		};
		struct Handler { STARK_COMMON_HANDLER_CONTENTS(EnergyAttachments, Params) };
		using MultiHandler = MultiEnergyHandler<Handler, Params>;

	public:
		/* Methods */
		EnergyAttachments(core::Stark& stark, const spPointDynamics dyn, const spRigidBodyDynamics rb);
		Handler add(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<int>& points_0, const std::vector<int>& points_1, const Params& params);
		Handler add(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<int>& points, const std::vector<std::array<int, 2>>& edges, const std::vector<std::array<double, 2>>& bary, const Params& params);
		Handler add(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<int>& points, const std::vector<std::array<int, 3>>& triangles, const std::vector<std::array<double, 3>>& bary, const Params& params);
		Handler add(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<std::array<int, 2>>& edges_0, const std::vector<std::array<int, 2>>& edges_1, const std::vector<std::array<double, 2>>& bary_0, const std::vector<std::array<double, 2>>& bary_1, const Params& params);
		MultiHandler add_by_distance(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<int>& points, const std::vector<std::array<int, 3>>& triangles, const double distance, const Params& params);
		
		Handler add(const RigidBodyHandler& rb, const PointSetHandler& set, const std::vector<Eigen::Vector3d>& rb_points_loc, const std::vector<int>& set_points, const Params& params);
		Handler add(const RigidBodyHandler& rb, const PointSetHandler& set, const std::vector<int>& points, const Params& params);
		Handler add_by_distance(const RigidBodyHandler& rb, const PointSetHandler& set, const std::vector<Eigen::Vector3d>& rb_points_loc, const std::vector<std::array<int, 3>>& rb_triangles, const std::vector<int>& set_points, const double distance, const Params& params);
		Params get_params(const Handler& handler) const;
		void set_params(const Handler& handler, const Params& params);

	private:
		enum class AttachmentType
		{
			Deformable_Deformable_Point_Point,
			Deformable_Deformable_Point_Edge,
			Deformable_Deformable_Point_Triangle,
			Deformable_Deformable_Edge_Edge,
			Rigid_Deformable
		};

		/* Fields */
		const spPointDynamics dyn;
		const spRigidBodyDynamics rb;

		// Deformable - Deformable
		//// Point - Point
		symx::LabelledConnectivity<3> conn_d_d_p_p{ {"group", "a", "b"} };
		std::vector<double> stiffness_d_d_p_p;  // per group
		std::vector<double> tolerance_d_d_p_p;  // per group

		//// Point - Edge
		symx::LabelledConnectivity<5> conn_d_d_p_e{ {"idx", "group", "p", "e0", "e1"} };
		std::vector<std::array<double, 2>> bary_p_e;  // per index
		std::vector<double> stiffness_d_d_p_e;  // per group
		std::vector<double> tolerance_d_d_p_e;  // per group

		//// Point - Triangle
		symx::LabelledConnectivity<6> conn_d_d_p_t{ {"idx", "group", "p", "t0", "t1", "t2"} };
		std::vector<std::array<double, 3>> bary_p_t;  // per index
		std::vector<double> stiffness_d_d_p_t;  // per group
		std::vector<double> tolerance_d_d_p_t;  // per group

		//// Edge - Edge
		symx::LabelledConnectivity<6> conn_d_d_e_e{ {"idx", "group", "ea0", "ea1", "eb0", "eb1"} };
		std::vector<std::array<double, 2>> bary_e_e_0;  // per index
		std::vector<std::array<double, 2>> bary_e_e_1;  // per index
		std::vector<double> stiffness_d_d_e_e;  // per group
		std::vector<double> tolerance_d_d_e_e;  // per group

		// RigidBody - Deformable
		symx::LabelledConnectivity<4> conn_rb_d{ {"idx", "group", "rb", "p"} };
		std::vector<Eigen::Vector3d> rb_points_loc;  // per index
		std::vector<double> stiffness_rb_d;  // per group
		std::vector<double> tolerance_rb_d;  // per group

		// Handlers map
		std::vector<std::pair<AttachmentType, int>> handlers_map;

		/* Methods */
		bool _is_converged_state_valid(stark::core::Stark& stark);
	};
}
