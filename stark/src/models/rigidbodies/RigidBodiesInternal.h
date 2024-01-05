#pragma once
#include <vector>
#include <string>
#include <memory>

#include "../../core/Stark.h"
#include "../../utils/mesh_generators.h"
#include "../MeshOutputGroups.h"

#include "inertia_tensors.h"
#include "RigidBodyDynamics.h"
#include "EnergyRigidBodyInertia.h"
#include "EnergyRigidBodyConstraints.h"
#include "../interactions/EnergyFrictionalContact.h"

namespace stark::models
{
	class RigidBodiesInternal
	{
	public:
		struct SequenceWriter
		{
			int body_idx;
			std::string label;
			core::Logger logger;
		};

		/* Fields */
		spRigidBodyDynamics dyn;
		spEnergyRigidBodyInertia inertia;
		spEnergyRigidBodyConstraints constraints;
		spEnergyFrictionalContact contact;

		// Meshes
		std::vector<utils::Mesh<3>> collision_meshes;  // Always stays at rest positions
		std::vector<utils::Mesh<3>> render_meshes;  // Always stays at rest positions

		// Output
		std::vector<SequenceWriter> transformation_sequences;
		MeshOutputGroups output_groups;  // local_indices
		bool write_render_mesh = true;
		bool write_collision_mesh = false;

		/* Methods */
		RigidBodiesInternal(stark::core::Stark& stark, spRigidBodyDynamics dyn, spEnergyFrictionalContact contact);

	private:
		// Stark callbacks
		void _write_frame(stark::core::Stark& stark);
		void _before_simulation__init_collision_meshes(core::Stark& stark);
	};
	using spRigidBodiesInternal = std::shared_ptr<RigidBodiesInternal>;
}
