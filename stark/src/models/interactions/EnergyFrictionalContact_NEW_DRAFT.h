#pragma once
#include <TriangleMeshCollisionDetection>

#include "../../utils/unordered_array_set_and_map.h"
#include "../deformables/PointDynamics.h"
#include "../rigidbodies/RigidBodyHandler.h"
#include "contact_and_friction_data.h"


namespace stark
{
	enum class IPCBarrierType { Log, Cubic };
	enum class IPCFrictionType { C0, C1 };

	class EnergyFrictionalContact
	{
	public:
		/* Types */
		struct Params
		{
			/* Fields */
			// Mandatory
			double dhat = -1.0;  // Can't have a good universal default value. If left at -1.0, the simulation will give an error

			// Recommended (Default values very soft. Make sure you )
			double stiffness = 1e3;
			double friction_stick_slide_threshold = 0.1;

			// Enable/Disable
			bool collisions_enabled = true;
			bool friction_enabled = true;
			bool triangle_point_enabled = true;
			bool edge_edge_enabled = true;
			bool intersection_test_enabled = true;

			/* Methods */
			Params& set_contact_distace(const double dhat) { this->dhat = dhat; return *this; }
			Params& set_stiffness(const double stiffness) { this->stiffness = stiffness; return *this; }
			Params& set_friction_threshold(const double friction_stick_slide_threshold) { this->friction_stick_slide_threshold = friction_stick_slide_threshold; return *this; }
			Params& enable_collisions(bool enable) { this->collisions_enabled = enable; return *this; }
			Params& enable_friction(bool enable) { this->friction_enabled = enable; return *this; }
			Params& enable_triangle_point(bool enable) { this->triangle_point_enabled = enable; return *this; }
			Params& enable_edge_edge(bool enable) { this->edge_edge_enabled = enable; return *this; }
			Params& enable_intersection_test(bool enable) { this->intersection_test_enabled = enable; return *this; }
		};
		struct Handler
		{
		private:
			int idx;
			Handler(int idx) : idx(idx) {}
			int get_idx() const { return idx; }
		};

	private:
		/* Fields */
		const spPointDynamics dyn;
		const spRigidBodyDynamics rb;

		// Parameters
		Params params;
		double base_stiffness = -1.0;
		IPCBarrierType ipc_barrier_type = IPCBarrierType::Cubic;
		IPCFrictionType ipc_friction_type = IPCFrictionType::C0;
		const double edge_edge_cross_norm_sq_cutoff = 1e-30;
		const double friction_displacement_perturbation = 1e-9;

		// Data structures for SymX
		Contacts_Deformables contacts_deformables;
		Contacts_RB contacts_rb;
		Contacts_RB_Deformables contacts_rb_deformables;
		Contacts_Static contacts_static;
		Friction_Deformables friction_deformables;
		Friction_RB friction_rb;
		Friction_RB_Deformables friction_rb_deformables;
		Friction_Static friction_static;

		//// Mappings
		//std::unordered_map<int, int> rigidbody_idx_collision_idx_map;
		//std::unordered_map<int, int> deformable_idx_collision_idx_map;

		// Meshes
		std::vector<Mesh> meshes;
		IntervalVector<Eigen::Vector3d> rigidbody_local_vertices;
		//std::vector<std::vector<int>> surface_node_maps;

		// Friction pairs
		utils::unordered_array_map<int, 2, double> pair_coulombs_mu;
		utils::unordered_array_set<int, 2> disabled_collision_pairs;

		// Static objects
		StaticPlanes static_planes;
		utils::unordered_array_map<int, 2, double> static_pair_coulombs_mu;
		utils::unordered_array_set<int, 2> static_disabled_collision_pairs;

		// Collision detection structures
		tmcd::IntersectionDetection id;
		tmcd::ProximityDetection pd;


	public:
		/* Methods */
		EnergyFrictionalContact(core::Stark& stark, const spPointDynamics dyn, const spRigidBodyDynamics rb);

		Handler add(const PointSetHandler& point_set, const std::vector<std::array<int, 3>>& triangles);
		Handler add(const PointSetHandler& point_set, const std::vector<std::array<int, 3>>& triangles, const std::vector<int>& local_to_global_map);
		Handler add(const PointSetHandler& point_set, const std::vector<std::array<int, 2>>& edges);
		Handler add(const PointSetHandler& point_set, const std::vector<std::array<int, 2>>& edges, const std::vector<int>& local_to_global_map = std::vector<int>());
		Handler add(const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& points, const std::vector<std::array<int, 3>>& triangles);
		Handler add(const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& points, const std::vector<std::array<int, 2>>& edges);
		Handler add_static_mesh(const std::vector<Eigen::Vector3d>& points, const std::vector<std::array<int, 3>>& triangles);

		//int add_rigid_body(const int idx, const std::vector<std::array<int, 3>>& triangles, const std::vector<Eigen::Vector3d>& vertices);
		//int add_rigid_body(const int idx, const std::vector<std::array<int, 2>>& edges, const std::vector<Eigen::Vector3d>& vertices);
		//int add_deformable(const int idx, const std::vector<std::array<int, 3>>& triangles, const std::vector<int>& surface_node_map);
		//int add_deformable(const int idx, const std::vector<std::array<int, 3>>& triangles, const int n_points);
		//int add_deformable(const int idx, const std::vector<std::array<int, 2>>& edges, const int n_points);
		//int add_static_plane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal);

		void set_friction(const Handler& obj0, const Handler& obj1, const double coulombs_coefficient);
		//void set_coulomb_friction_pair(const PhysicalSystem& ps0, const int idx0_in_ps, const PhysicalSystem& ps1, const int idx1_in_ps, const double mu);
		//void set_coulomb_friction_pair(const int idx0, const int idx1, const double mu);
		//void set_coulomb_friction_pair_static(const int static_idx, const PhysicalSystem& ps, const int idx_in_ps, const double mu);
		
		void disable_collision(const Handler& obj0, const Handler& obj1);
		//void disable_collision(const int idx0, const int idx1);
		//void disable_collision(const PhysicalSystem& ps0, const int idx0_in_ps, const PhysicalSystem& ps1, const int idx1_in_ps);
		//void disable_collision_static(int static_idx, const PhysicalSystem& ps, const int idx_in_ps);

		void set_params(const Params& params);
		Params get_params(const Params& params);

	private:
		// Helpers
		//int _get_collision_idx(const PhysicalSystem& ps, const int idx_in_ps);
		int _add_edges_and_points(const PhysicalSystem& ps, const int idx, const std::vector<std::array<int, 2>>& edges, const int n_points);
		int _add_triangles_edges_and_points(const PhysicalSystem& ps, const int idx, const std::vector<std::array<int, 3>>& triangles, const int n_points);
		void _update_vertices(core::Stark& stark, const double dt);
		void _update_contacts_static(core::Stark& stark);
		bool _intersection_static(core::Stark& stark);
		void _update_friction_static(core::Stark& stark);
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
		double _get_friction_static(const int static_idx, const int object_idx);
		bool _is_disabled_collision_pair_static(const int static_idx, const int object_idx);

		// SymX callbacks
		void _before_time_step__update_friction_contacts(core::Stark& stark);
		void _before_energy_evaluation__update_contacts(core::Stark& stark);
		bool _is_valid_configuration(core::Stark& stark);

		// SymX definitions
		void _energies_contact_deformables(core::Stark& stark);
		void _energies_contact_rb(core::Stark& stark);
		void _energies_contact_rb_deformables(core::Stark& stark);
		void _energies_contact_static(core::Stark& stark);

		void _energies_friction_deformables(core::Stark& stark);
		void _energies_friction_rb(core::Stark& stark);
		void _energies_friction_rb_deformables(core::Stark& stark);
		void _energies_friction_static(core::Stark& stark);

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
