#pragma once
#include <vector>
#include <array>
#include <unordered_map>
#include <memory>

#include <Eigen/Dense>
#include <symx>

#include "../solver/Stark.h"
#include "../utils/Logger.h"
#include "RigidBodyDynamics.h"
#include "deformables/MeshOutputGroups.h"
#include "deformables/Id.h"
#include "../utils/MultiMesh/MultiMesh_.h"
//#include "../utils/MultiMeshEdges.h"
#include "../utils/mesh_generators.h"
#include "../utils/inertia_tensors.h"
//#include "../utils/unordered_array_set_and_map.h"


namespace stark::models
{
	class FourWheelsVehicleRigidBodySystem
	{
		struct Template
		{
			double wheel_radius = 0.0;
			double body_wheight = 0.0;
			double max_torque = 0.0;
			//...

			static Template car();
			static Template truck();
			static Template monster_truck();
		};

		/* Fields */

		/* Methods */
		FourWheelsVehicleRigidBodySystem(Stark& stark, const Template& vehicle_template);
	};
}