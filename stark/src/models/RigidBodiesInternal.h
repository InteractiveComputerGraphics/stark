#pragma once
#include <vector>
#include <string>
#include <memory>

#include "../solver/Stark.h"
#include "deformables/MeshOutputGroups.h"
#include "../utils/mesh_generators.h"
#include "../utils/inertia_tensors.h"

#include "RigidBodyDynamics.h"
#include "EnergyRigidBodyInertia.h"
#include "EnergyRigidBodyConstraints.h"


namespace stark::models
{
	class RigidBodiesInternal
	{
	public:
		struct SequenceWriter
		{
			int body_idx;
			std::string label;
			utils::Logger logger;
		};

		/* Fields */
		spRigidBodyDynamics dyn;
		spEnergyRigidBodyInertia inertia;
		spEnergyRigidBodyConstraints constraints;

		// Meshes
		std::vector<utils::Mesh<3>> collision_meshes;  // Always stays at rest positions
		std::vector<utils::Mesh<3>> render_meshes;  // Always stays at rest positions

		// Output
		std::vector<SequenceWriter> transformation_sequences;
		MeshOutputGroups output_groups;  // local_indices
		bool write_render_mesh = true;
		bool write_collision_mesh = false;

		/* Methods */
		RigidBodiesInternal(Stark& stark, spRigidBodyDynamics dyn);

	private:
		// Stark callbacks
		void _write_frame(Stark& stark);
	};
	using spRigidBodiesInternal = std::shared_ptr<RigidBodiesInternal>;
}
