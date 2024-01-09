#pragma once
#include "../deformables/PointDynamics.h"
#include "../rigidbodies/RigidBodyDynamics.h"


namespace stark::models
{
	class EnergyAttachments
	{
	public:
		/* Fields */
		const spPointDynamics dyn;
		const spRigidBodyDynamics rb;

		// Deformable - Deformable
		symx::LabelledConnectivity<3> conn_d_d{ {"group", "a", "b"} };
		std::vector<double> stiffness_d_d;  // per group

		// RigidBody - Deformable
		symx::LabelledConnectivity<4> conn_rb_d{ {"idx", "group", "rb", "p"} };
		std::vector<Eigen::Vector3d> rb_points_loc;  // per index
		std::vector<double> stiffness_rb_d;  // per group

		/* Methods */
		EnergyAttachments(core::Stark& stark, const spPointDynamics dyn, const spRigidBodyDynamics rb);
	};
}
