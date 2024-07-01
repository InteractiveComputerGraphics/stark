#pragma once
#include <TriangleMeshCollisionDetection>

#include "../../utils/unordered_array_set_and_map.h"
#include "../deformables/PointDynamics.h"
#include "../rigidbodies/RigidBodyHandler.h"
#include "../types.h"
#include "contact_and_friction_data.h"


namespace stark
{
	enum class IPCBarrierType { Log, Cubic };
	enum class IPCFrictionType { C0, C1 };

	class EnergyFrictionalContact
	{
	public:
		/* Types */
		struct GlobalParams
		{
			STARK_PARAM_POSITIVE(double, default_contact_thickness, -1.0)  // Default parameter -1.0 means no default is set

			// Recommended (Default values are very soft. Probably a good idea to set them manually.)
			STARK_PARAM_NON_NEGATIVE(double, min_contact_stiffness, 1e3)
			STARK_PARAM_NON_NEGATIVE(double, friction_stick_slide_threshold, 0.1)

			// Enable/Disable
			STARK_PARAM_NO_VALIDATION(bool, collisions_enabled, true)
			STARK_PARAM_NO_VALIDATION(bool, friction_enabled, true)
			STARK_PARAM_NO_VALIDATION(bool, triangle_point_enabled, true)
			STARK_PARAM_NO_VALIDATION(bool, edge_edge_enabled, true)
			STARK_PARAM_NO_VALIDATION(bool, intersection_test_enabled, true)
		};
		struct Params
		{
			// Default 0.0 will use the global default_contact_thickness if defined, otherwise will throw.
			//// Contact thickness cannot be zero.
			STARK_PARAM_POSITIVE(double, contact_thickness, 0.0)  
		};
		struct Handler 
		{ 
		private:
			EnergyFrictionalContact* model = nullptr;
			int idx = -1;

		public:
			inline Handler() {}
			inline Handler(EnergyFrictionalContact* model, int idx) : model(model), idx(idx) {}
			inline int get_idx() const { return this->idx; } 
			inline EnergyFrictionalContact* get_model() { return this->model; }
			inline const EnergyFrictionalContact* get_model() const { return this->model; }
			inline void set_contact_thickness(double d) { this->model->set_contact_thickness(*this, d); }
			inline void set_friction(const Handler& other, double coulombs_mu) { this->model->set_friction(*this, other, coulombs_mu); }
			inline void disable_collision(const Handler& other) { this->model->disable_collision(*this, other); }
			inline bool is_valid() const { return this->model != nullptr; }
			inline void exit_if_not_valid(const std::string& where_) const { if (!this->is_valid()) { std::cout << "stark error: Invalid handler found in " << where_ << std::endl; exit(-1); } }
		};

	private:
		/* Fields */
		const spPointDynamics dyn;
		const spRigidBodyDynamics rb;
		bool is_initialized = false;

		// Parameters
		GlobalParams global_params;
		double contact_stiffness = 1e3;
		IPCBarrierType ipc_barrier_type = IPCBarrierType::Cubic;
		IPCFrictionType ipc_friction_type = IPCFrictionType::C0;
		const double edge_edge_cross_norm_sq_cutoff = 1e-30;
		const double friction_displacement_perturbation = 1e-9;

		// Data structures for SymX
		std::vector<double> contact_thicknesses;
		IntervalVector<Eigen::Vector3d> rigidbody_local_vertices;
		Contacts_Deformables contacts_deformables;
		Contacts_RB contacts_rb;
		Contacts_RB_Deformables contacts_rb_deformables;
		Friction_Deformables friction_deformables;
		Friction_RB friction_rb;
		Friction_RB_Deformables friction_rb_deformables;

		//// Mappings
		std::unordered_map<int, DeformableBookkeeping> group_to_deformable_bookkeeping;
		std::unordered_map<int, RigidBodyBookkeeping> group_to_rigidbody_bookkeeping;

		// Meshes
		std::vector<ContactMesh> meshes;

		// Pairwise data
		unordered_array_map<int, 2, double> pair_coulombs_mu;
		unordered_array_set<int, 2> disabled_collision_pairs;

		// Collision detection structures
		tmcd::IntersectionDetection id;
		tmcd::ProximityDetection pd;


	public:
		/* Methods */
		EnergyFrictionalContact(core::Stark& stark, const spPointDynamics dyn, const spRigidBodyDynamics rb);

		// Deformables
		Handler add_triangles(const PointSetHandler& point_set, const std::vector<std::array<int, 3>>& triangles, const Params& params);
		Handler add_triangles(const PointSetHandler& point_set, const std::vector<std::array<int, 3>>& triangles, const std::vector<int>& point_set_map, const Params& params);
		Handler add_edges(const PointSetHandler& point_set, const std::vector<std::array<int, 2>>& edges, const Params& params);
		Handler add_edges(const PointSetHandler& point_set, const std::vector<std::array<int, 2>>& edges, const std::vector<int>& point_set_map, const Params& params);

		// Rigid bodies
		Handler add_triangles(const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Params& params);
		Handler add_edges(const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 2>>& edges, const Params& params);

		// Setters and getters
		GlobalParams get_global_params() const;
		void set_global_params(const GlobalParams& params);
		void set_contact_thickness(const Handler& obj, const double contact_thickness);
		double get_contact_stiffness() const;
		void set_friction(const Handler& obj0, const Handler& obj1, const double coulombs_coefficient);
		void disable_collision(const Handler& obj0, const Handler& obj1);
		bool is_empty() const;

	private:
		// Helpers
		std::array<int, 2> _get_pair_key(const Handler& obj0, const Handler& obj1);
		double _init_contact_thickness(double contact_thickness);
		Handler _add_edges_and_points(const PhysicalSystem& ps, int idx, const Params& params, int n_vertices, const std::vector<std::array<int, 2>>& edges);
		Handler _add_triangles_edges_and_points(const PhysicalSystem& ps, int idx, const Params& params, int n_vertices, const std::vector<std::array<int, 3>>& triangles);
		void _add_rigid_body(const RigidBodyHandler& rb, const Handler& handler, const std::vector<Eigen::Vector3d>& vertices);
		void _add_deformable(const PointSetHandler& point_set, const Handler& handler, const std::vector<int>& point_set_map);

		void _update_vertices(core::Stark& stark, const double dt);
		const tmcd::ProximityResults& _run_proximity_detection(core::Stark& stark, const double dt);
		const tmcd::IntersectionResults& _run_intersection_detection(core::Stark& stark, const double dt);

		// Collision helpers
		template<std::size_t N>
		std::array<int, N> _local_to_ps_global_indices(int group, const std::array<int, N>& local);
		ProximityHelper<1> _get_proximity_helper_point(const tmcd::Point& point);
		ProximityHelper<2> _get_proximity_helper_edge(const tmcd::TriangleEdge::Edge& edge);
		ProximityHelper<3> _get_proximity_helper_triangle(const tmcd::Triangle& triangle);
		ProximityHelper<1> _get_proximity_helper_edge_point(const tmcd::EdgePoint& edge_point);
		ProximityHelper<2> _get_proximity_helper_edge(const tmcd::Edge& edge);
		double _get_friction(const int idx0, const int idx1);

		// SymX callbacks
		void _before_time_step__update_friction_contacts(core::Stark& stark);
		void _before_energy_evaluation__update_contacts(core::Stark& stark);
		bool _is_intermidiate_state_valid(core::Stark& stark, bool is_initial_check);
		void _on_intermidiate_state_invalid(core::Stark& stark);
		void _on_time_step_accepted(core::Stark& stark);

		// SymX definitions
		void _energies_contact_deformables(core::Stark& stark);
		void _energies_contact_rb(core::Stark& stark);
		void _energies_contact_rb_deformables(core::Stark& stark);

		void _energies_friction_deformables(core::Stark& stark);
		void _energies_friction_rb(core::Stark& stark);
		void _energies_friction_rb_deformables(core::Stark& stark);

		// IPC
		symx::Scalar _get_contact_distance(symx::Energy& energy, const symx::Index& group_a, const symx::Index& group_b);
		double _get_contact_distance(int group_a, int group_b);
		symx::Scalar _barrier_potential(const symx::Scalar& d, const symx::Scalar& dhat, const symx::Scalar& k);
		double _barrier_force(const double d, const double dhat, const double k);
		symx::Scalar _edge_edge_mollifier(const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest);
		symx::Scalar _friction_potential(const symx::Vector& v, const symx::Scalar& fn, const symx::Scalar& mu, const symx::Matrix& T, const symx::Scalar& epsv, const symx::Scalar& dt);

		// SymX setters
		void _set_barrier_potential(symx::Energy& energy, const core::Stark& stark, const symx::Scalar& d, const symx::Index& group_a, const symx::Index& group_b);
		void _set_edge_edge_mollified_barrier_potential(symx::Energy& energy, const core::Stark& stark, const symx::Scalar& d, const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest, const symx::Index& group_a, const symx::Index& group_b);
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
	// Name shortening since it is very often used
	using ContactParams = EnergyFrictionalContact::Params;  
	using ContactGlobalParams = EnergyFrictionalContact::GlobalParams;
	using ContactHandler = EnergyFrictionalContact::Handler;
}
