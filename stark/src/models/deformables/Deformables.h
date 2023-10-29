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
		/* ==============================  Fields  =============================== */
		/* ------------------------------  General  ------------------------------ */
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

		// Collision meshes
		IntervalConnectivity<2> collision_edges;
		IntervalConnectivity<3> collision_triangles;

		// Misc
		symx::DoF dof;
		bool needs_initialization = true;


		/* ------------------------------  Potentials  ------------------------------ */
		// Nodal Potentials
		//// Inertia
		symx::LabelledConnectivity<2> nodes_conn{ { "mesh", "node" } };
		IntervalVector<double> nodes_lumped_mass;  // [kg] per vertex
		std::vector<double> nodes_inertial_damping; // per mesh

		//// Prescribed positions
		symx::LabelledConnectivity<2> prescribed_positions_conn{ {"idx", "node"}};
		std::unordered_map<int, Eigen::Vector3d> prescribed_nodes_map;
		std::vector<Eigen::Vector3d> prescribed_positions;

		//// Attachments
		symx::LabelledConnectivity<2> attached_nodes_conn{ {"i", "j"} };
		utils::unordered_array_set<int, 2> attached_nodes_set;


		// Edge Potentials
		//// Strain (rods)
		symx::LabelledConnectivity<3> rods_conn{ {"mesh", "i", "j"} };
		std::vector<double> rods_strain_stiffness;  // per mesh

		//// Strain limiting and damping (non-exclusive to rods)
		symx::LabelledConnectivity<3> edges_conn{ {"mesh", "i", "j"}};
		std::vector<double> edges_strain_limiting_start;  // per mesh
		std::vector<double> edges_strain_limiting_stiffness;  // per mesh
		
		//// Strain damping (non-exclusive to rods) [uses conn_all_edges]
		std::vector<double> edges_strain_damping;  // per mesh


		// Triangle Potentials
		//// Neo-Hookean strain (cloth and shells)
		symx::LabelledConnectivity<5> shells_conn_triangles{ {"mesh", "tri", "i", "j", "k"} };
		std::vector<double> shells_young_modulus;  // per mesh
		std::vector<double> shells_poisson_ratio;  // per mesh
		std::vector<std::array<double, 4>> shells_triangle_DXinv;  // per triangle
		std::vector<double> shells_triangle_rest_area;  // per triangle

		//// Bending (cloth and shells)
		symx::LabelledConnectivity<6> shells_conn_surface_internal_edges{ {"mesh", "ie", "i", "j", "k", "l"}};
		std::vector<double> shells_bending_stiffness;  // per mesh
		std::vector<double> shells_bending_damping;  // per mesh
		std::vector<std::array<double, 16>> shells_bergou_Q_matrix;  // per internal angle
		std::vector<double> shells_cutoff_bending_angle_deg;  // per mesh


		// Tet potentials
		//// Neo-Hookean strain (volumetrics)
		symx::LabelledConnectivity<6> volumetrics_conn_tets{ {"mesh", "tet", "i", "j", "k", "l"}};
		std::vector<double> volumetrics_young_modulus;  // per mesh
		std::vector<double> volumetrics_poisson_ratio;  // per mesh
		std::vector<std::array<double, 9>> volumetrics_tet_DXinv;  // per tet
		std::vector<double> volumetrics_tet_rest_volume;  // per tet


		/* ==============================  Methods  ============================== */
		void init(Stark& sim);

	private:
		void _potentials_inertia(Stark& sim);
		void _potentials_boundary_conditions(Stark& sim);
		void _potentials_edge_strain_limiting_and_damping(Stark& sim);
		void _potentials_mechanics_rods(Stark& sim);
		void _potentials_mechanics_shells(Stark& sim);
		void _potentials_mechanics_volumetrics(Stark& sim);
	};
}
