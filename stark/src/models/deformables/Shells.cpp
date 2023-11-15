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

	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::array<int, 3>> triangles;
	if (this->labeled_groups.size() == 0) {

	}
	else {
		std::vector<Eigen::Vector3d> vertices = this->dyn->x1.extract(this->global_indices).data;
		for (const int glob_idx : this->global_indices){
			const int offset = 
		}
	}




	this->mesh_writer.write(
		[&](const int local_idx) { return this->dyn->x1.get_begin_ptr(id); }
		[&](const int local_idx) { return this->dyn->x1.get_end_ptr(local_idx); }
		[&](const int local_idx) { return this->dyn->x1.get_end_ptr(local_idx); }
	);

	utils::write_VTK(stark.get_vtk_path("cloth"), this->model.x1, this->model.mesh.connectivity, stark.settings.output.calculate_smooth_normals);
}
