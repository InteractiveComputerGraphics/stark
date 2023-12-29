#include "DeformableSolidsLines.h"

#include "../../utils/mesh_utils.h"

stark::models::DeformableSolidsLines::DeformableSolidsLines(
	stark::core::Stark& stark, 
	spPointDynamics dyn, 
	spEnergyPointInertia inertia, 
	spEnergyPointPrescribedPositions prescribed_positions
	//spEnergyFrictionalContact contact
)
	: dyn(dyn), inertia(inertia), prescribed_positions(prescribed_positions) //, contact(contact)
{
	stark.callbacks.write_frame.push_back([&]() { this->_write_frame(stark); });

	// Init specific energies
	this->strain = std::make_shared<EnergyEdgeStrain>(stark, dyn);
}

stark::models::Id stark::models::DeformableSolidsLines::add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 2>>& edges, const MaterialLine& material)
{
	Id id = this->dyn->add(vertices);
	const int shell_id = (int)this->global_indices.size();
	this->global_indices.push_back(id.get_global_idx());
	const int size = this->dyn->size(id);
	const int offset = this->dyn->get_begin(id);
	
	// Output
	this->input_edges.push_back(edges);

	// Add to energies
	this->inertia->add(id, edges,
		material.line_density,
		material.inertia_damping);
	this->strain->add(id, edges,
		material.section_radius, 
		material.strain_young_modulus, 
		material.strain_limit,
		material.strain_limit_stiffness,
		material.strain_damping);
	//this->contact->add_triangles_edges_and_points(id, triangles, size, offset);

	id.set_local_idx("DeformableSolidsLines", shell_id);
	return id;
}
int stark::models::DeformableSolidsLines::get_index(const Id& id) const
{
	return id.get_local_idx("DeformableSolidsLines");
}
int stark::models::DeformableSolidsLines::get_n_volumes() const
{
	return (int)this->global_indices.size();
}
void stark::models::DeformableSolidsLines::_write_frame(stark::core::Stark& stark)
{
	if (this->get_n_volumes() == 0) { return; }

	auto concatenate_meshes = [&](const std::vector<int>& local_indices)
	{
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 2>> edges;
		for (const int local_idx : local_indices) {
			const int global_idx = this->global_indices[local_idx];
			const int offset = (int)vertices.size();
			vertices.insert(vertices.end(), this->dyn->x1.get_begin_ptr(global_idx), this->dyn->x1.get_end_ptr(global_idx));
			for (const std::array<int, 2>& edge : this->input_edges[local_idx]) {
				edges.push_back({ edge[0] + offset, edge[1] + offset });
			}
		}
		return std::pair{ vertices, edges };
	};

	// Export groups
	if (this->output_groups.size() > 0) {
		for (auto it : this->output_groups.groups) {
			const std::string label = it.first;
			const std::unordered_set<int> group = it.second;
			auto [vertices, edges] = concatenate_meshes(std::vector<int>(group.begin(), group.end()));
			//utils::write_VTK(stark.get_frame_path("line_" + label) + ".vtk", vertices, edges);
			utils::write_VTK(stark.get_frame_path("line_" + label) + ".vtk", vertices);
		}
	}

	// Default: everything goes into the same .vtk
	else {
		std::vector<int> all_local_indices(this->get_n_volumes());
		std::iota(all_local_indices.begin(), all_local_indices.end(), 0);
		auto [vertices, edges] = concatenate_meshes(all_local_indices);
		//utils::write_VTK(stark.get_frame_path("line_") + ".vtk", vertices);
		utils::write_VTK(stark.get_frame_path("line_") + ".vtk", vertices);
	}
}

stark::models::MaterialLine stark::models::MaterialLine::sticky_goo()
{
	MaterialLine material;
	material.line_density = 0.05;
	material.section_radius = 0.002;
	material.inertia_damping = 0.1;
	material.strain_young_modulus = 5e4;
	material.strain_limit = std::numeric_limits<double>::max();
	material.strain_limit_stiffness = 1e6;
	material.strain_damping = 0.2;
	return material;
}
