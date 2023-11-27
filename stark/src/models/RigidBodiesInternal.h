#pragma once
#include <vector>
#include <memory>

#include "../solver/Stark.h"
#include "../utils/MultiMesh/MultiMesh_.h"
#include "deformables/MeshOutputGroups.h"
#include "../utils/mesh_generators.h"
#include "../utils/inertia_tensors.h"

#include "RigidBodyDynamics.h"
#include "EnergyRigidBodyInertia.h"
#include "EnergyRigidBodyConstraints.h"
#include "rigidbody_constraints_ui.h"


namespace stark::models
{
	class RigidBodiesInternal
	{
	public:
		spRigidBodyDynamics dyn;
		spEnergyRigidBodyInertia inertia;
		spEnergyRigidBodyConstraints constraints;

		// Meshes
		std::vector<utils::Mesh<3>> collision_meshes;  // Always stays at rest positions
		std::vector<utils::Mesh<3>> render_meshes;  // Always stays at rest positions

		// Output
		MeshOutputGroups output_groups;  // local_indices
		bool write_render_mesh = true;
		bool write_collision_mesh = false;
		bool write_transformation_sequences = false;

		RigidBodiesInternal(Stark& stark, spRigidBodyDynamics dyn);

	private:
		// Stark callbacks
		void _write_frame(Stark& stark);
	};
	using spRigidBodiesInternal = std::shared_ptr<RigidBodiesInternal>;
}
