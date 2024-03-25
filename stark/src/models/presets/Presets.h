#pragma once
#include "DeformablesPresets.h"
#include "RigidBodyPresets.h"

namespace stark
{
	class Presets
	{
	public:
		/* Methods */
		Presets(core::Stark& stark, std::shared_ptr<Deformables> deformables, std::shared_ptr<RigidBodies> rigidbodies, std::shared_ptr<Interactions> interactions);

		/* Fields */
		std::shared_ptr<DeformablesPresets> deformables;
		std::shared_ptr<RigidBodyPresets> rigidbodies;
	};
}
