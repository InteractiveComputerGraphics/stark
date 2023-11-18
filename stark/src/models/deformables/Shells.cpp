#include "Shells.h"

#include "../../utils/mesh_utils.h"

stark::models::Shells::Shells(
	Stark& stark, 
	spPointDynamics dyn, 
	spEnergyPointInertia inertia, 
	spEnergyPointPrescribedPositions prescribed_positions, 
	spEnergyTriangleStrain strain, 
	spEnergyTriangleBendingBergou06 bending_bergou
	//spEnergyFrictionalContact contact
)
	: dyn(dyn), inertia(inertia), prescribed_positions(prescribed_positions), strain(strain), bending_bergou(bending_bergou) //, contact(contact)
{
	stark.callbacks.write_frame.push_back([&]() { this->_write_frame(stark); });
}

stark::models::Id stark::models::Shells::add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const Material material)
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
		material.strain_limit, 
		material.strain_limit_stiffness);
	this->bending_bergou->add(id, triangles, 
		material.bending_stiffness, 
		material.bending_damping, 
		material.bending_cutoff_angle_deg);
	//this->contact->add_triangles_edges_and_points(id, triangles, size, offset);

	id.set_local_idx("Shells", shell_id);
	return id;
}

std::shared_ptr<stark::models::PrescribedPointGroup> stark::models::Shells::create_prescribed_positions_group(Id& id, const std::string label)
{
	return this->prescribed_positions->create_group(id, label);
}

std::shared_ptr<stark::models::PrescribedPointGroupWithTransformation> stark::models::Shells::create_prescribed_positions_group_with_transformation(Id& id, const std::string label)
{
	return this->prescribed_positions->create_group_with_transformation(id, label);
}

void stark::models::Shells::add_to_output_label(const std::string label, Id& id)
{
	this->output_groups.add_to_group(label, id.get_local_idx("Shells"));
}

bool stark::models::Shells::is_empty() const
{
	return this->get_n_objects() == 0;
}

int stark::models::Shells::get_n_objects() const
{
	return (int)this->global_indices.size();
}

void stark::models::Shells::_write_frame(Stark& stark)
{
	if (this->is_empty()) { return; }

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
			utils::write_VTK(stark.get_vtk_path("shells_" + label), vertices, triangles, stark.settings.output.calculate_smooth_normals);
		}
	}

	// Default: everything goes into the same .vtk
	else {
		std::vector<int> all_local_indices(this->get_n_objects());
		std::iota(all_local_indices.begin(), all_local_indices.end(), 0);
		auto [vertices, triangles] = concatenate_meshes(all_local_indices);
		utils::write_VTK(stark.get_vtk_path("shells"), vertices, triangles, stark.settings.output.calculate_smooth_normals);
	}
}

stark::models::Shells::Material stark::models::Shells::Material::towel()
{
	Material material;
	material.area_density = 0.2;
	material.thickness = 3.2e-3;
	material.inertia_damping = 2.0;
	material.strain_young_modulus = 1.56e4;
	material.strain_poisson_ratio = 0.3;
	material.strain_limit = 0.1;
	material.strain_limit_stiffness = 1e3;
	//material.strain_damping = 0.2;
	material.bending_stiffness = 5e-6;
	//material.bending_damping = 0.2;
	return material;
}
