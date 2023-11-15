#include "Shells.h"

#include "../../utils/mesh_utils.h"

stark::models::Shells::Shells(
	Stark& stark, 
	spPointDynamics dyn, 
	spEnergyPointInertia inertia, 
	spEnergyPointPrescribedPositions prescribed_positions, 
	spEnergyTriangleStrain strain, 
	spEnergyTriangleBendingBergou06 bending_bergou, 
	spEnergyEdgeStrain edge_strain_limiting_and_damping,
	spEnergyFrictionalContact contact)
	: dyn(dyn), inertia(inertia), prescribed_positions(prescribed_positions), strain(strain), bending_bergou(bending_bergou), edge_strain_limiting_and_damping(edge_strain_limiting_and_damping), contact(contact)
{

}

stark::models::Id stark::models::Shells::add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const MaterialPresets material)
{
	Id id = this->dyn->add(vertices);
	const int size = this->dyn->size(id);
	const int offset = this->dyn->get_begin(id);
	this->contact->add_triangles_edges_and_points(id, triangles, size, offset);
	
	// MeshWriter
}

void stark::models::Shells::_write_frame(Stark& stark)
{
	if (this->is_empty()) { return; }

	if (this->labeled_groups.size() == 0) {

	}
	else {
		// NAIVE VERSION
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 3>> triangles;
		for (int local_obj_idx = 0; local_obj_idx < (int)this->global_indices.size(); local_obj_idx++) {
			const int glob_obj_idx = this->global_indices[local_obj_idx];
			const int base_idx = (int)vertices.size();
			vertices.insert(vertices.end(), this->dyn->x1.get_begin_ptr(glob_obj_idx), this->dyn->x1.get_end_ptr(glob_obj_idx));

			for (const std::array<int, 3>&tri : this->input_triangles[local_obj_idx]) {
				triangles.push_back({ tri[0] + base_idx, tri[1] + base_idx, tri[2] + base_idx });
			}
		}
		utils::write_VTK(stark.get_vtk_path("cloth"), vertices, triangles, stark.settings.output.calculate_smooth_normals);

		// SIMPLE MANAGER
		// -> This would go when adding a cloth
		this->mesh_writer.add_triangles(
			[&](const int local_idx) { return { this->dyn->x1.get_begin_ptr(this->global_indices[local_idx]), this->dyn->x1.get_end_ptr(this->global_indices[local_idx]) }; },
			[&](const int local_idx) { return { this->input_triangles[local_idx].begin(), this->input_triangles[local_idx].end() }; });
		// Yes, it is only what it needs. But the execution order becomes really convoluted. This is called from somewhere and the pointers are get from somewhere else...
		

		// Instead of lambdas, we can be explicit here (old style)
		this->mesh_writer.clear();
		
		for (int local_obj_idx = 0; local_obj_idx < (int)this->global_indices.size(); local_obj_idx++) {
			const int glob_obj_idx = this->global_indices[local_obj_idx];

			this->mesh_writer.add_triangles(
				this->dyn->x1.get_begin_ptr(glob_obj_idx),
				this->dyn->x1.get_end_ptr(glob_obj_idx),
				this->input_triangles[local_obj_idx].begin(),
				this->input_triangles[local_obj_idx].end()
			);
		}
		
		this->mesh_writer.write();
	}
}
