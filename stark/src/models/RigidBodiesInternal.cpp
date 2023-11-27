#include "RigidBodiesInternal.h"

stark::models::RigidBodiesInternal::RigidBodiesInternal(Stark& stark, spRigidBodyDynamics dyn)
	: dyn(dyn)
{
	this->inertia = std::make_shared<EnergyRigidBodyInertia>(stark, dyn);
	this->constraints = std::make_shared<EnergyRigidBodyConstraints>(stark, dyn);

	stark.callbacks.write_frame.push_back([&]() { this->_write_frame(stark); });
}

//void stark::models::RigidBodies::_write_frame(Stark& sim)
//{
//	if (this->is_empty()) { return; }
//	this->constraint_logger.save_to_disk();
//	if (!this->write_VTK) { return; }
//
//	if (this->output_labeled_groups.size() > 0) {
//		for (const auto& pair : this->output_labeled_groups) {
//			const std::string& label = pair.first;
//			const std::vector<int>& rb_indices = pair.second;
//
//			std::vector<Eigen::Vector3d> glob_vertices;
//			std::vector<std::array<int, 3>> triangles;
//			for (const int rb_i : rb_indices) {
//				const Eigen::Matrix3d& R = this->R1[rb_i];
//				const Eigen::Vector3d& t = this->t1[rb_i];
//
//				const std::array<int, 2> v_range = this->collision_mesh.get_vertices_range(rb_i);
//				const int idx_offset = (int)glob_vertices.size() - v_range[0];
//				for (int vertex_i = v_range[0]; vertex_i < v_range[1]; vertex_i++) {
//					const Eigen::Vector3d p = local_to_global_point(this->collision_mesh.vertices[vertex_i], R, t);
//					glob_vertices.push_back(p);
//				}
//
//				const std::array<int, 2> e_range = this->collision_mesh.get_elements_range(rb_i);
//				for (int tri_i = e_range[0]; tri_i < e_range[1]; tri_i++) {
//					const std::array<int, 3>& tri = this->collision_mesh.connectivity[tri_i];
//					triangles.push_back({ tri[0] + idx_offset, tri[1] + idx_offset, tri[2] + idx_offset });
//				}
//			}
//			if (label.size() == 0) {
//				utils::write_VTK(sim.get_vtk_path("rb"), glob_vertices, triangles, false);
//			}
//			else {
//				utils::write_VTK(sim.get_vtk_path("rb_" + label), glob_vertices, triangles, false);
//			}
//		}
//	}
//	else {
//		std::vector<Eigen::Vector3d> glob_vertices(this->collision_mesh.get_n_vertices());
//		for (int rb_i = 0; rb_i < this->collision_mesh.get_n_meshes(); rb_i++) {
//			const Eigen::Matrix3d& R = this->R1[rb_i];
//			const Eigen::Vector3d& t = this->t1[rb_i];
//
//			const std::array<int, 2> range = this->collision_mesh.get_vertices_range(rb_i);
//			for (int vertex_i = range[0]; vertex_i < range[1]; vertex_i++) {
//				const Eigen::Vector3d p = local_to_global_point(this->collision_mesh.vertices[vertex_i], R, t);
//				glob_vertices[vertex_i] = p;
//			}
//		}
//		utils::write_VTK(sim.get_vtk_path("rb"), glob_vertices, this->collision_mesh.connectivity, false);
//	}
//}
