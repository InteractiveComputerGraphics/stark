#include "Presets.h"

stark::Presets::Presets(core::Stark& stark, std::shared_ptr<Deformables> deformables, std::shared_ptr<RigidBodies> rigidbodies, std::shared_ptr<Interactions> interactions)
{
	this->deformables = std::make_shared<DeformablesPresets>(stark, deformables, interactions);
	this->rigidbodies = std::make_shared<RigidBodyPresets>(stark, rigidbodies, interactions);
}
