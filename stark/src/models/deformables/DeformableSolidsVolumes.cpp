#include "DeformableSolidsVolumes.h"

#include "../../utils/mesh_utils.h"

stark::models::DeformableSolidsVolumes::DeformableSolidsVolumes(
	stark::core::Stark& stark, 
	spPointDynamics dyn, 
	spEnergyPointInertia inertia, 
	spEnergyPointPrescribedPositions prescribed_positions,
	spEnergyFrictionalContact contact
)
	: dyn(dyn), inertia(inertia), prescribed_positions(prescribed_positions), contact(contact)
{
	stark.callbacks.write_frame.push_back([&]() { this->_write_frame(stark); });
	this->strain = std::make_shared<EnergyTetStrain>(stark, dyn);
}

stark::models::Id stark::models::DeformableSolidsVolumes::add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 4>>& tets, const MaterialVolume& material)
{
	Id id = this->dyn->add(vertices);
	const int shell_id = (int)this->global_indices.size();
	this->global_indices.push_back(id.get_global_idx());
	const int size = this->dyn->size(id);
	const int offset = this->dyn->get_begin(id);
	
	// Output
	std::vector<Eigen::Vector3d> surface_vertices;
	std::vector<std::array<int, 3>> surface_triangles;
	std::vector<int> triangle_to_tet_node_map;
	utils::extract_surface(surface_vertices, surface_triangles, triangle_to_tet_node_map, vertices, tets);
	this->input_triangles.push_back(surface_triangles);
	this->triangle_to_tet_node_maps.push_back(triangle_to_tet_node_map);

	// Add to energies
	this->inertia->add(id, tets, 
		material.density, 
		material.inertia_damping);
	this->strain->add(id, tets, 
		material.strain_young_modulus, 
		material.strain_poisson_ratio,
		material.strain_damping,
		material.strain_limit,
		material.strain_limit_stiffness);
	this->contact->add_deformable(id.get_global_idx(), surface_triangles, triangle_to_tet_node_map);

	id.set_local_idx("DeformableSolidsVolumes", shell_id);
	return id;
}
int stark::models::DeformableSolidsVolumes::get_index(const Id& id) const
{
	return id.get_local_idx("DeformableSolidsVolumes");
}
int stark::models::DeformableSolidsVolumes::get_n_volumes() const
{
	return (int)this->global_indices.size();
}
void stark::models::DeformableSolidsVolumes::_write_frame(stark::core::Stark& stark)
{
	if (this->get_n_volumes() == 0) { return; }

	auto concatenate_meshes = [&](const std::vector<int>& local_indices)
	{
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 3>> triangles;
		for (const int local_idx : local_indices) {
			const int global_idx = this->global_indices[local_idx];
			const int offset = (int)vertices.size();
			const std::vector<int>& triangle_to_tet_node_map = this->triangle_to_tet_node_maps[local_idx];

			for (int surface_vertex_idx = 0; surface_vertex_idx < (int)triangle_to_tet_node_map.size(); surface_vertex_idx++) {
				const int tet_vertex_idx = triangle_to_tet_node_map[surface_vertex_idx];
				vertices.push_back(this->dyn->x1.get(global_idx, tet_vertex_idx));
			}

			for (const std::array<int, 3>&tri : this->input_triangles[local_idx]) {
				triangles.push_back({ tri[0] + offset, tri[1] + offset, tri[2] + offset });
			}
		}
		return std::pair{ vertices, triangles };
	};

	// Export groups
	if (this->output_groups.size() > 0) {
		for (auto it : this->output_groups.groups) {
			const std::string label = it.first;
			const std::unordered_set<int> group = it.second;
			auto [vertices, triangles] = concatenate_meshes(std::vector<int>(group.begin(), group.end()));
			utils::write_VTK(stark.get_frame_path("volume_" + label) + ".vtk", vertices, triangles, stark.settings.output.calculate_smooth_normals);
		}
	}

	// Default: everything goes into the same .vtk
	else {
		std::vector<int> all_local_indices(this->get_n_volumes());
		std::iota(all_local_indices.begin(), all_local_indices.end(), 0);
		auto [vertices, triangles] = concatenate_meshes(all_local_indices);
		utils::write_VTK(stark.get_frame_path("volume_") + ".vtk", vertices, triangles, stark.settings.output.calculate_smooth_normals);
	}
}
stark::models::MaterialVolume stark::models::MaterialVolume::soft_rubber()
{
	MaterialVolume material;
	material.density = 1000.0;
	material.inertia_damping = 0.1;
	material.strain_young_modulus = 1e5;
	material.strain_poisson_ratio = 0.3;
	material.strain_limit = 0.1;
	material.strain_limit_stiffness = 1e5;
	material.strain_damping = 0.2;
	return material;
}
