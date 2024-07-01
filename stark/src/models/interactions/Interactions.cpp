#include "Interactions.h"

using namespace stark;

stark::Interactions::Interactions(core::Stark& stark, spPointDynamics dyn, spRigidBodyDynamics rb)
	: dyn(dyn), rb(rb)
{
	this->attachments = std::make_shared<EnergyAttachments>(stark, dyn, rb);
	this->contact = std::make_shared<EnergyFrictionalContact>(stark, dyn, rb);
}
