#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>
#include <symx>
#include <TriangleMeshCollisionDetection>

#include "../../solver/Stark.h"
#include "../../utils/unordered_array_set_and_map.h"
#include "Id.h"
#include "PointDynamics.h"
#include "../RigidBodies.h"
#include "contact_and_friction_data.h"

namespace stark::models
{
	enum class IPCBarrierType { Log, Cubic };
	enum class IPCFrictionType { C0, C1 };

	class EnergyFrictionalContact
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		const spRigidBodies rb;
		utils::unordered_array_map<int, 2, double> pair_coulombs_mu; // uses local_idx
		std::vector<std::array<int, 2>> disabled_collision_pairs; // uses local_idx
		std::vector<std::string> labels;  // per group
		IPCBarrierType ipc_barrier_type = IPCBarrierType::Cubic;
		IPCFrictionType ipc_friction_type = IPCFrictionType::C0;

		// Connectivities
		// Note: All have the same number of sets, even if empty (edge sets not having triangles)
		IntervalConnectivity<1> points;
		IntervalConnectivity<2> edges;
		IntervalConnectivity<3> triangles;

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
		EnergyFrictionalContact(Stark& stark, const spPointDynamics dyn, const spRigidBodies rb);
		void add_points(Id& id, const int n_points, const int offset);
		void add_edges_and_points(Id& id, const std::vector<std::array<int, 2>>& edges, const int n_points, const int offset);
		void add_triangles_edges_and_points(Id& id, const std::vector<std::array<int, 3>>& triangles, const int n_points, const int offset);
		

	private:
		// SymX callbacks
		void _before_time_step__update_friction_contacts(Stark& stark);
		void _before_energy_evaluation__update_contacts(Stark& stark);
		bool _is_valid_configuration(Stark& stark);

		// SymX definitions
		void _energies_contact_deformables(Stark& stark);
		void _energies_contact_rb(Stark& stark);
		void _energies_contact_rb_deformables(Stark& stark);

		void _energies_friction_deformables(Stark& stark);
		void _energies_friction_rb(Stark& stark);
		void _energies_friction_rb_deformables(Stark& stark);

		// IPC
		symx::Scalar _barrier_potential(const symx::Scalar& d, const symx::Scalar& dhat, const symx::Scalar& k);
		symx::Scalar _edge_edge_mollifier(const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest);
		symx::Scalar _friction_potential(const symx::Vector& v, const symx::Scalar& fn, const symx::Scalar& mu, const symx::Matrix& T, const symx::Scalar& epsv, const symx::Scalar& dt);

		// SymX setters
		void _set_barrier_potential(symx::Energy& energy, const Stark& stark, const symx::Scalar& d);
		void _set_edge_edge_mollified_barrier_potential(symx::Energy& energy, const Stark& stark, const symx::Scalar& d, const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest);
		void _set_friction_potential(symx::Energy& energy, const Stark& stark, const symx::Vector& v, const symx::Index& contact_idx, const FrictionContact& contact);
		void _set_friction_point_edge(symx::Energy& energy, const Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& ve, const symx::Index& contact_idx, FrictionPointEdge& data);
		void _set_friction_point_triangle(symx::Energy& energy, const Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& vt, const symx::Index& contact_idx, FrictionPointTriangle& data);
		void _set_friction_edge_edge(symx::Energy& energy, const Stark& stark, const std::vector<symx::Vector>& vea, const std::vector<symx::Vector>& veb, const symx::Index& contact_idx, FrictionEdgeEdge& data);

		// SymX data-symbol getters
		//// Rigid bodies
		std::vector<symx::Vector> _get_rb_v1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn);
		std::vector<symx::Vector> _get_rb_x1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn);
		std::vector<symx::Vector> _get_rb_X(symx::Energy& energy, const std::vector<symx::Index>& conn);
		std::array<std::vector<symx::Vector>, 2> _get_rb_edge(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn);
		std::array<std::vector<symx::Vector>, 3> _get_rb_edge_point(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn);

		//// Deformables
		std::vector<symx::Vector> _get_d_v1(symx::Energy& energy, const std::vector<symx::Index>& conn);
		std::vector<symx::Vector> _get_d_x1(symx::Energy& energy, const Stark& stark, const std::vector<symx::Index>& conn);
		std::vector<symx::Vector> _get_d_X(symx::Energy& energy, const std::vector<symx::Index>& conn);
		std::array<std::vector<symx::Vector>, 2> _get_d_edge(symx::Energy& energy, const Stark& stark, const std::vector<symx::Index>& conn);
		std::array<std::vector<symx::Vector>, 3> _get_d_edge_point(symx::Energy& energy, const Stark& stark, const std::vector<symx::Index>& conn);

		// Misc
		std::string _get_contact_label(const std::string physical_system, const std::string pair) const;
		std::string _get_friction_label(const std::string physical_system, const std::string pair) const;
	};
	using spEnergyFrictionalContact = std::shared_ptr<EnergyFrictionalContact>;
}
