#pragma once
#include <type_traits>

#include "EnergyFrictionalContact.h"
#include "EnergyAttachments.h"
#include "../rigidbodies/RigidBodyHandler.h"
#include "../deformables/DeformableLineHandler.h"
#include "../deformables/DeformableSurfaceHandler.h"
#include "../deformables/DeformableVolumeHandler.h"

namespace stark::models
{
	struct StaticPlaneHandler
	{
		int idx;
	};

	class Interactions
	{
	public:
		Interactions(core::Stark& stark, spEnergyFrictionalContact);

		// Attachments

		// Note: All attachments disable collision between the objects

		// Note: Distance to the rigid body collision mesh
		template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
		void attach_by_distance(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1, const double distance = 1e-6, const double stiffness = 1e6);

		template<typename DEFORMABLE_HANDLER_0, typename DEFORMABLE_HANDLER_1>
		void attach(const DEFORMABLE_HANDLER_0& deformable0, const DEFORMABLE_HANDLER_1& deformable1, const std::vector<std::array<int, 2>>& pairs, const double stiffness = 1e6);

		template<typename DEFORMABLE_HANDLER>
		void attach(const RigidBodyHandler& rigidbody, const DEFORMABLE_HANDLER& deformable, const std::vector<int>& deformable_vertices, const double stiffness = 1e6);

		// Contact and Friction
		template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
		void set_friction(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1, double coulombs_mu);

		template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
		void disable_collision(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1);

		// Static objects
		StaticPlaneHandler add_static_plane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal);

	private:
		/* Fields */
		spEnergyFrictionalContact contact = nullptr;
		std::shared_ptr<EnergyAttachments> attachments = nullptr;
		
		/* Methods */
		template<typename OBJECT_HANDLER>
		std::pair<PhysicalSystem, int> _get_physical_system_and_index(const OBJECT_HANDLER& object);
		template<typename DEFORMABLE_HANDLER>
		void _check_deformable_type(const DEFORMABLE_HANDLER& deformable);
		void _attach_rb_deformable_by_distance(const RigidBodyHandler& rigidbody, const int deformable_idx, const double distance, const double stiffness);
		void _attach_deformable_deformable_by_distance(const int deformable_idx0, const int deformable_idx1, const double distance, const double stiffness);
	};

	// ===========================================  DEFINITIONS  ===================================================
	template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
	inline void Interactions::attach_by_distance(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1, const double distance, const double stiffness)
	{
		this->disable_collision(object0, object1);
		if constexpr (std::is_base_of<DeformableHandler, OBJECT_HANDLER_0>::value && std::is_base_of<DeformableHandler, OBJECT_HANDLER_1>::value) {
			this->_attach_deformable_deformable_by_distance(object0.get_deformable_idx(), object1.get_deformable_idx(), distance, stiffness);
		}
		else if constexpr (std::is_same<RigidBodyHandler, OBJECT_HANDLER_0>::value && std::is_base_of<DeformableHandler, OBJECT_HANDLER_1>::value) {
			this->_attach_rb_deformable_by_distance(object0, object1.get_deformable_idx(), distance, stiffness);
		}
		else if constexpr (std::is_base_of<DeformableHandler, OBJECT_HANDLER_0>::value && std::is_same<RigidBodyHandler, OBJECT_HANDLER_1>::value) {
			this->_attach_rb_deformable_by_distance(object1, object0.get_deformable_idx(), distance, stiffness);
		}
		else if constexpr (std::is_same<RigidBodyHandler, OBJECT_HANDLER_0>::value && std::is_same<RigidBodyHandler, OBJECT_HANDLER_1>::value) {
			std::cout << "stark error: Interactions can't handle two rigid bodies for attachments. Please, use rigid body constraints instead." << std::endl;
			exit(-1);
		}
		else {
			std::cout << "stark error: Interactions received an unsupported object Handler type." << std::endl;
			exit(-1);
		}
	}

	template<typename DEFORMABLE_HANDLER_0, typename DEFORMABLE_HANDLER_1>
	inline void Interactions::attach(const DEFORMABLE_HANDLER_0& deformable0, const DEFORMABLE_HANDLER_1& deformable1, const std::vector<std::array<int, 2>>& pairs, const double stiffness)
	{
		this->disable_collision(deformable0, deformable1);
		this->_check_deformable_type(deformable0);
		this->_check_deformable_type(deformable1);

		std::vector<std::array<int, 2>> global_pairs;
		global_pairs.reserve(pairs.size());
		for (const std::array<int, 2>& pair : pairs) {
			global_pairs.push_back({ deformable0.get_global_vertex_idx(pair[0]), deformable1.get_global_vertex_idx(pair[1]) });
		}

		this->attachments->add_deformable_deformable(global_pairs, stiffness);
	}
	template<typename DEFORMABLE_HANDLER>
	inline void Interactions::attach(const RigidBodyHandler& rigidbody, const DEFORMABLE_HANDLER& deformable, const std::vector<int>& deformable_vertices, const double stiffness)
	{
		this->disable_collision(rigidbody, deformable);
		this->_check_deformable_type(deformable);

		std::vector<int> global_vertices;
		global_vertices.reserve(deformable_vertices.size());
		for (const int& vertex_idx : deformable_vertices) {
			global_vertices.push_back(deformable.get_global_vertex_idx(vertex_idx));
		}

		this->attachments->add_rigidbody_deformable(rigidbody.index(), global_vertices, stiffness);
	}
	template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
	inline void Interactions::set_friction(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1, double coulombs_mu)
	{
		if constexpr (std::is_same_v<OBJECT_HANDLER_0, StaticPlaneHandler>) {
			auto [ps1, idx_in_ps1] = this->_get_physical_system_and_index(object1);
			this->contact->set_coulomb_friction_pair_static(object0.idx, ps1, idx_in_ps1, coulombs_mu);
		}
		else if constexpr (std::is_same_v<OBJECT_HANDLER_1, StaticPlaneHandler>) {
			auto [ps0, idx_in_ps0] = this->_get_physical_system_and_index(object0);
			this->contact->set_coulomb_friction_pair_static(object1.idx, ps0, idx_in_ps0, coulombs_mu);
		}
		else {
			auto [ps0, idx_in_ps0] = this->_get_physical_system_and_index(object0);
			auto [ps1, idx_in_ps1] = this->_get_physical_system_and_index(object1);
			this->contact->set_coulomb_friction_pair(ps0, idx_in_ps0, ps1, idx_in_ps1, coulombs_mu);
		}
	}
	template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
	inline void Interactions::disable_collision(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1)
	{
		if constexpr (std::is_same_v<OBJECT_HANDLER_0, StaticPlaneHandler>) {
			auto [ps1, idx_in_ps1] = this->_get_physical_system_and_index(object1);
			this->contact->disable_collision_static(object0.idx, ps1, idx_in_ps1);
		}
		else if constexpr (std::is_same_v<OBJECT_HANDLER_1, StaticPlaneHandler>) {
			auto [ps0, idx_in_ps0] = this->_get_physical_system_and_index(object0);
			this->contact->disable_collision_static(object1.idx, ps0, idx_in_ps0);
		}
		else {
			auto [ps0, idx_in_ps0] = this->_get_physical_system_and_index(object0);
			auto [ps1, idx_in_ps1] = this->_get_physical_system_and_index(object1);
			this->contact->disable_collision(ps0, idx_in_ps0, ps1, idx_in_ps1);
		}
	}
	template<typename OBJECT_HANDLER>
	inline std::pair<PhysicalSystem, int> Interactions::_get_physical_system_and_index(const OBJECT_HANDLER& object)
	{
		if constexpr (std::is_same_v<OBJECT_HANDLER, RigidBodyHandler>) {
			return std::make_pair(PhysicalSystem::Rigidbody, object.index());
		}
		else if constexpr (std::is_same_v<OBJECT_HANDLER, DeformableLineHandler>) {
			return std::make_pair(PhysicalSystem::Deformable, object.get_deformable_idx());
		}
		else if constexpr (std::is_same_v<OBJECT_HANDLER, DeformableSurfaceHandler>) {
			return std::make_pair(PhysicalSystem::Deformable, object.get_deformable_idx());
		}
		else if constexpr (std::is_same_v<OBJECT_HANDLER, DeformableVolumeHandler>) {
			return std::make_pair(PhysicalSystem::Deformable, object.get_deformable_idx());
		}
		else {
			std::cout << "stark error: Interactions received an unsupported object Handler type." << std::endl;
			exit(-1);
		}
	}
	template<typename DEFORMABLE_HANDLER>
	inline void Interactions::_check_deformable_type(const DEFORMABLE_HANDLER& deformable)
	{
		if constexpr (!std::is_base_of<DeformableHandler, DEFORMABLE_HANDLER>::value) {
			std::cout << "stark error: Interactions received an unsupported deformable Handler type." << std::endl;
			exit(-1);
		}
	}
}
