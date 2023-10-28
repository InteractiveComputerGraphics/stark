#pragma once

#include <symx>

#include "../../solver/Stark.h"
#include "../../utils/unordered_array_set_and_map.h"

#include "interval_types.h"

namespace stark::models
{
	class Deformables
	{
	public:
		/* Fields */
		// Dynamics
		IntervalVector<Eigen::Vector3d> X;   // Node positions at rest
		IntervalVector<Eigen::Vector3d> x0;  // Node positions at time n
		IntervalVector<Eigen::Vector3d> x1;  // Node positions at time n+1
		IntervalVector<Eigen::Vector3d> v0;  // Node velocities at time n
		IntervalVector<Eigen::Vector3d> v1;  // Node velocities at time n+1
		IntervalVector<Eigen::Vector3d> a;   // Node accelerations

		// Input connectivities
		IntervalConnectivity<2> input_edges;
		IntervalConnectivity<3> input_triangles;
		IntervalConnectivity<4> input_tets;

		// Misc
		bool needs_initialization = true;

		// Potentials
		symx::DoF dof;
		
		//// Inertia
		symx::LabelledConnectivity<2> conn_all_nodes{ { "mesh", "node" } };
		IntervalVector<double> lumped_mass;  // [kg] per vertex
		std::vector<double> inertial_damping; // per mesh

		//// Prescribed positions
		symx::LabelledConnectivity<2> conn_prescribed_positions{ {"idx", "node"}};
		std::unordered_map<int, Eigen::Vector3d> prescribed_nodes_map;
		std::vector<Eigen::Vector3d> prescribed_positions;

		//// Attachments
		symx::LabelledConnectivity<2> conn_attached_nodes{ {"node_a", "node_b"} };
		utils::unordered_array_set<int, 2> attached_nodes_set;


		/* Methods */
		void init(Stark& sim);

	private:
		void _potentials_inertia(Stark& sim);
		void _potentials_boundary_conditions(Stark& sim);
		void _potentials_edge_strain_limiting_and_damping(Stark& sim);
	};
}