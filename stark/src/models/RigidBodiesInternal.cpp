#include "RigidBodiesInternal.h"

#include "rigidbody_transformations.h"
#include "../utils/mesh_utils.h"

stark::models::RigidBodiesInternal::RigidBodiesInternal(Stark& stark, spRigidBodyDynamics dyn)
	: dyn(dyn)
{
	this->inertia = std::make_shared<EnergyRigidBodyInertia>(stark, dyn);
	this->constraints = std::make_shared<EnergyRigidBodyConstraints>(stark, dyn);

	stark.callbacks.write_frame.push_back([&]() { this->_write_frame(stark); });
}

void stark::models::RigidBodiesInternal::_write_frame(Stark& stark)
{
	if (this->dyn->get_n_bodies() == 0) { return; }

	auto concatenate_meshes = [&](const std::vector<int>& bodies, const bool collision_mesh)
	{
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 3>> triangles;
		for (const int body_idx : bodies) {
			const int offset = (int)vertices.size();
			const utils::Mesh<3>& mesh = (collision_mesh) ? this->collision_meshes[body_idx] : this->render_meshes[body_idx];

			const Eigen::Matrix3d& R = this->dyn->R1[body_idx];
			const Eigen::Vector3d& t = this->dyn->t1[body_idx];
			for (const Eigen::Vector3d& p : mesh.vertices) {
				vertices.push_back(local_to_global_point(p, R, t));
			}
			for (const std::array<int, 3>& triangle : mesh.conn) {
				triangles.push_back({ triangle[0] + offset, triangle[1] + offset, triangle[2] + offset });
			}
		}
		return std::pair{ vertices, triangles };
	};

	// Export groups
	if (this->output_groups.size() > 0) {
		for (auto it : this->output_groups.groups) {
			const std::string label = it.first;
			const std::unordered_set<int> group = it.second;

			if (this->write_render_mesh) {
				auto [vertices, triangles] = concatenate_meshes(std::vector<int>(group.begin(), group.end()), false);
				utils::write_VTK(stark.get_vtk_path("rb_"), vertices, triangles, false);
			}
			if (this->write_collision_mesh) {
				auto [vertices, triangles] = concatenate_meshes(std::vector<int>(group.begin(), group.end()), true);
				utils::write_VTK(stark.get_vtk_path("rb_col_"), vertices, triangles, false);
			}
		}
	}

	// Default: everything goes into the same .vtk
	else {
		std::vector<int> all_local_indices(this->dyn->get_n_bodies());
		std::iota(all_local_indices.begin(), all_local_indices.end(), 0);

		if (this->write_render_mesh) {
			auto [vertices, triangles] = concatenate_meshes(all_local_indices, false);
			utils::write_VTK(stark.get_vtk_path("rb_"), vertices, triangles, false);
		}
		if (this->write_collision_mesh) {
			auto [vertices, triangles] = concatenate_meshes(all_local_indices, true);
			utils::write_VTK(stark.get_vtk_path("rb_col_"), vertices, triangles, false);
		}
	}

	// Transformation sequences
	for (auto& seq : this->transformation_sequences) {
		const int idx = seq.body_idx;
		
		for (int i = 0; i < 3; i++) {
			seq.logger.append_to_series("translation", this->dyn->t1[idx][i]);
		}

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				seq.logger.append_to_series("rotation", this->dyn->R1[idx](i, j));
			}
		}
	}
}
