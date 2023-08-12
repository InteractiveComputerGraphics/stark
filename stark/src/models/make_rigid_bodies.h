#pragma once

#include <Eigen/Dense>

#include "RigidBodies.h"
#include "../utils/inertia_tensors.h"
#include "../utils/mesh_generators.h"

namespace stark::models
{ 
	RigidBodies::InputRigidBody make_sphere(const double mass, const double radius, const int subdivisions = 2,
		const Eigen::Vector3d& displacement = { 0, 0, 0 }, const double rotate_deg = 0.0, const Eigen::Vector3d& rotation_axis = {0, 0, 1});
}