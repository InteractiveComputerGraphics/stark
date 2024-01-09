#pragma once
#include <type_traits>

#include "EnergyFrictionalContact.h"
#include "../rigidbodies/RigidBodyHandler.h"
#include "../deformables/DeformableLineHandler.h"
#include "../deformables/DeformableSurfaceHandler.h"
#include "../deformables/DeformableVolumeHandler.h"

namespace stark::models
{
	class Interactions
	{
	public:
		Interactions(core::Stark& stark, spEnergyFrictionalContact);

		template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
		void set_friction(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1, double coulombs_mu);
		void set_friction(const int global_idx0, const int global_idx1, double coulombs_mu);

		template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
		void disable_collision(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1);
		void disable_collision(const int global_idx0, const int global_idx1);

	private:
		spEnergyFrictionalContact contact = nullptr;
		template<typename OBJECT_HANDLER>
		std::pair<PhysicalSystem, int> _get_physical_system_and_index(const OBJECT_HANDLER& object);
	};

	// ===========================================  DEFINITIONS  ===================================================
	template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
	inline void Interactions::set_friction(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1, double coulombs_mu)
	{
		auto [ps0, idx_in_ps0] = this->_get_physical_system_and_index(object0);
		auto [ps1, idx_in_ps1] = this->_get_physical_system_and_index(object1);
		this->contact->set_coulomb_friction_pair(ps0, idx_in_ps0, ps1, idx_in_ps1, coulombs_mu);
	}
	template<typename OBJECT_HANDLER_0, typename OBJECT_HANDLER_1>
	inline void Interactions::disable_collision(const OBJECT_HANDLER_0& object0, const OBJECT_HANDLER_1& object1)
	{
		auto [ps0, idx_in_ps0] = this->_get_physical_system_and_index(object0);
		auto [ps1, idx_in_ps1] = this->_get_physical_system_and_index(object1);
		this->contact->disable_collision(ps0, idx_in_ps0, ps1, idx_in_ps1);
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
	}
}
