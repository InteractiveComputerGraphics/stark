#pragma once
#include "EnergyFrictionalContact.h"
#include "EnergyAttachments.h"
#include "../rigidbodies/RigidBodyHandler.h"
#include "../deformables/PointSetHandler.h"

namespace stark
{
	class Interactions
	{
	public:
		/* Methods */
		Interactions(core::Stark& stark, spPointDynamics dyn, spRigidBodyDynamics rb);

		/* Fields */
		std::shared_ptr<EnergyAttachments> attachments;
		std::shared_ptr<EnergyFrictionalContact> contact;

	private:
		/* Fields */
		spPointDynamics dyn;
		spRigidBodyDynamics rb;
	};
}
