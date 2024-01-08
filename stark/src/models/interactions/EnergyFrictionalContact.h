#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>
#include <symx>
#include <TriangleMeshCollisionDetection>

#include "../../core/Stark.h"
#include "../../utils/unordered_array_set_and_map.h"
#include "../IntervalConnectivity.h"
#include "../deformables/PointDynamics.h"
#include "../rigidbodies/RigidBodyDynamics.h"
#include "contact_and_friction_data.h"



/*
	HOW ALL THIS WORKS
	==================
		- There is no tracking of outter types in the interface. The user declares meshes with their physical system and a index is returned. 
			Handling of these indices must be done in a higher layer of abstraction.
		- There is no order in the meshes.
		- Contact pair deactivation and pair friction must be done using the returned indices.
		- Obviously, this class needs to know where each mesh comes from and has hardcoded routines to the Deformable and RigidBody classes.

*/

namespace stark::models
{
	enum class IPCBarrierType { Log, Cubic };
	enum class IPCFrictionType { C0, C1 };

	class EnergyFrictionalContact
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		const spRigidBodyDynamics rb;
		utils::unordered_array_map<int, 2, double> pair_coulombs_mu;
		utils::unordered_array_set<int, 2> disabled_collision_pairs;
		IPCBarrierType ipc_barrier_type = IPCBarrierType::Cubic;
		IPCFrictionType ipc_friction_type = IPCFrictionType::C0;

		// Meshes
		std::vector<Mesh> meshes;
		IntervalVector<Eigen::Vector3d> rigidbody_local_vertices;
		std::vector<std::vector<int>> surface_node_maps;

		// Mappings
		std::vector<int> rigidbody_global_idx;
		std::vector<int> deformable_global_idx;

		// Collision detection
		tmcd::IntersectionDetection id;
		tmcd::ProximityDetection pd;

		// Data structures for SymX
		Contacts_Deformables contacts_deformables;
		Contacts_RB contacts_rb;
		Contacts_RB_Deformables contacts_rb_deformables;
		Friction_Deformables friction_deformables;
		Friction_RB friction_rb;
		Friction_RB_Deformables friction_rb_deformables;


		/* Methods */
		EnergyFrictionalContact(core::Stark& stark, const spPointDynamics dyn, const spRigidBodyDynamics rb);
		void set_barrier_type(const IPCBarrierType type);
		void set_friction_type(const IPCFrictionType type);
		int add_rigid_body(const int idx, const std::vector<std::array<int, 3>>& triangles, const std::vector<Eigen::Vector3d>& vertices);
		int add_rigid_body(const int idx, const std::vector<std::array<int, 2>>& edges, const std::vector<Eigen::Vector3d>& vertices);
		int add_deformable(const int idx, const std::vector<std::array<int, 3>>& triangles, const std::vector<int>& surface_node_map);
		int add_deformable(const int idx, const std::vector<std::array<int, 3>>& triangles, const int n_points);
		int add_deformable(const int idx, const std::vector<std::array<int, 2>>& edges, const int n_points);
		void set_coulomb_friction_pair(const int idx0, const int idx1, const double mu);
		void disable_collision(const int idx0, const int idx1);
		// void enable_collision(const int idx1, const int idx2); // Not available in collision detection !

	private:
		// Helpers
		int _add_edges_and_points(const PhysicalSystem& ps, const int idx, const std::vector<std::array<int, 2>>& edges, const int n_points);
		int _add_triangles_edges_and_points(const PhysicalSystem& ps, const int idx, const std::vector<std::array<int, 3>>& triangles, const int n_points);
		void _update_vertices(core::Stark& stark, const double dt);
		const tmcd::ProximityResults& _run_proximity_detection(core::Stark& stark, const double dt);
		const tmcd::IntersectionResults& _run_intersection_detection(core::Stark& stark, const double dt);

		// Collision helpers
		template<std::size_t N>
		std::array<int, N> _local_to_ps_global_indices(const PhysicalSystem& ps, const int set_idx, const std::array<int, N>& local);
		ProximityHelper<1> _get_proximity_helper_point(const tmcd::Point& point);
		ProximityHelper<2> _get_proximity_helper_edge(const tmcd::TriangleEdge::Edge& edge);
		ProximityHelper<3> _get_proximity_helper_triangle(const tmcd::Triangle& triangle);
		ProximityHelper<1> _get_proximity_helper_edge_point(const tmcd::EdgePoint& edge_point);
		ProximityHelper<2> _get_proximity_helper_edge(const tmcd::Edge& edge);
		double _get_friction(const int idx0, const int idx1);

		// SymX callbacks
		void _before_time_step__update_friction_contacts(core::Stark& stark);
		void _before_energy_evaluation__update_contacts(core::Stark& stark);
		bool _is_valid_configuration(core::Stark& stark);

		// SymX definitions
		void _energies_contact_deformables(core::Stark& stark);
		void _energies_contact_rb(core::Stark& stark);
		void _energies_contact_rb_deformables(core::Stark& stark);

		void _energies_friction_deformables(core::Stark& stark);
		void _energies_friction_rb(core::Stark& stark);
		void _energies_friction_rb_deformables(core::Stark& stark);

		// IPC
		symx::Scalar _barrier_potential(const symx::Scalar& d, const symx::Scalar& dhat, const symx::Scalar& k);
		double _barrier_force(const double d, const double dhat, const double k);
		symx::Scalar _edge_edge_mollifier(const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest);
		symx::Scalar _friction_potential(const symx::Vector& v, const symx::Scalar& fn, const symx::Scalar& mu, const symx::Matrix& T, const symx::Scalar& epsv, const symx::Scalar& dt);

		// SymX setters
		void _set_barrier_potential(symx::Energy& energy, const core::Stark& stark, const symx::Scalar& d);
		void _set_edge_edge_mollified_barrier_potential(symx::Energy& energy, const core::Stark& stark, const symx::Scalar& d, const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest);
		void _set_friction_potential(symx::Energy& energy, const core::Stark& stark, const symx::Vector& v, const symx::Index& contact_idx, const FrictionContact& contact);
		void _set_friction_point_edge(symx::Energy& energy, const core::Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& ve, const symx::Index& contact_idx, FrictionPointEdge& data);
		void _set_friction_point_triangle(symx::Energy& energy, const core::Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& vt, const symx::Index& contact_idx, FrictionPointTriangle& data);
		void _set_friction_edge_edge(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Vector>& vea, const std::vector<symx::Vector>& veb, const symx::Index& contact_idx, FrictionEdgeEdge& data);

		// SymX data-symbol getters
		//// Rigid bodies
		std::vector<symx::Vector> _get_rb_v1(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn);
		std::vector<symx::Vector> _get_rb_x1(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn);
		std::vector<symx::Vector> _get_rb_X(symx::Energy& energy, const std::vector<symx::Index>& conn);
		std::array<std::vector<symx::Vector>, 2> _get_rb_edge(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn);
		std::array<std::vector<symx::Vector>, 3> _get_rb_edge_point(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn);

		//// Deformables
		std::vector<symx::Vector> _get_d_v1(symx::Energy& energy, const std::vector<symx::Index>& conn);
		std::vector<symx::Vector> _get_d_x1(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Index>& conn);
		std::vector<symx::Vector> _get_d_X(symx::Energy& energy, const std::vector<symx::Index>& conn);
		std::array<std::vector<symx::Vector>, 2> _get_d_edge(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Index>& conn);
		std::array<std::vector<symx::Vector>, 3> _get_d_edge_point(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Index>& conn);

		// Misc
		std::string _get_contact_label(const std::string physical_system, const std::string pair) const;
		std::string _get_friction_label(const std::string physical_system, const std::string pair) const;
	};
	using spEnergyFrictionalContact = std::shared_ptr<EnergyFrictionalContact>;
}
