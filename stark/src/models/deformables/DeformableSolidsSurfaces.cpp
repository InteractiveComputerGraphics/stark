#include "DeformableSolidsSurfaces.h"

#include "../../utils/mesh_utils.h"

stark::models::DeformableSolidsSurfaces::DeformableSolidsSurfaces(
	stark::core::Stark& stark, 
	spPointDynamics dyn, 
	spEnergyPointInertia inertia, 
	spEnergyPointPrescribedPositions prescribed_positions
	//spEnergyFrictionalContact contact
)
	: dyn(dyn), inertia(inertia), prescribed_positions(prescribed_positions) //, contact(contact)
{
	stark.callbacks.write_frame.push_back([&]() { this->_write_frame(stark); });

	this->strain = std::make_shared<EnergyTriangleStrain>(stark, dyn);
	this->bending_grispun_03 = std::make_shared<EnergyTriangleBendingGrinspun03>(stark, dyn);
}

stark::models::Id stark::models::DeformableSolidsSurfaces::add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const MaterialSurface& material)
{
	Id id = this->dyn->add(vertices);
	const int shell_id = (int)this->global_indices.size();
	this->global_indices.push_back(id.get_global_idx());
	const int size = this->dyn->size(id);
	const int offset = this->dyn->get_begin(id);
	this->input_triangles.push_back(triangles);

	this->inertia->add(id, triangles, 
		material.area_density, 
		material.inertia_damping);
	this->strain->add(id, triangles, 
		material.thickness, 
		material.strain_young_modulus, 
		material.strain_poisson_ratio, 
		material.strain_damping,
		material.strain_limit, 
		material.strain_limit_stiffness);
	this->bending_grispun_03->add(id, triangles, 
		material.bending_stiffness,
		material.bending_damping);
	//this->contact->add_triangles_edges_and_points(id, triangles, size, offset);

	id.set_local_idx("DeformableSolidsSurfaces", shell_id);
	return id;
}

int stark::models::DeformableSolidsSurfaces::get_index(const Id& id) const
{
	return id.get_local_idx("DeformableSolidsSurfaces");
}

int stark::models::DeformableSolidsSurfaces::get_n_surfaces() const
{
	return (int)this->global_indices.size();
}

void stark::models::DeformableSolidsSurfaces::_write_frame(stark::core::Stark& stark)
{
	if (this->get_n_surfaces() == 0) { return; }

	auto concatenate_meshes = [&](const std::vector<int>& local_indices)
	{
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 3>> triangles;
		for (const int local_idx : local_indices) {
			const int global_idx = this->global_indices[local_idx];
			const int offset = (int)vertices.size();
			vertices.insert(vertices.end(), this->dyn->x1.get_begin_ptr(global_idx), this->dyn->x1.get_end_ptr(global_idx));
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
			utils::write_VTK(stark.get_frame_path("surface_" + label) + ".vtk", vertices, triangles, stark.settings.output.calculate_smooth_normals);
		}
	}

	// Default: everything goes into the same .vtk
	else {
		std::vector<int> all_local_indices(this->get_n_surfaces());
		std::iota(all_local_indices.begin(), all_local_indices.end(), 0);
		auto [vertices, triangles] = concatenate_meshes(all_local_indices);
		utils::write_VTK(stark.get_frame_path("surface_") + ".vtk", vertices, triangles, stark.settings.output.calculate_smooth_normals);
	}
}

stark::models::MaterialSurface stark::models::MaterialSurface::towel()
{
	MaterialSurface material;
	material.area_density = 0.2;
	material.thickness = 3.2e-3;
	material.inertia_damping = 2.0;
	material.strain_young_modulus = 1e3; // 1.56e4;
	material.strain_poisson_ratio = 0.3;
	material.strain_limit = 0.1;
	material.strain_limit_stiffness = 1e5;
	material.strain_damping = 0.2;
	material.bending_stiffness = 5e-6;
	material.bending_damping = 0.2;
	return material;
}
