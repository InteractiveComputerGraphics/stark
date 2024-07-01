#include "DeformablesMeshOutput.h"

#include "../../utils/include.h"


stark::DeformablesMeshOutput::DeformablesMeshOutput(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	stark.callbacks.add_write_frame([&]() { this->_write_frame(stark); });
}
void stark::DeformablesMeshOutput::add_point_set(const std::string& label, const PointSetHandler& set)
{
	set.exit_if_not_valid("DeformablesMeshOutput::add_point_set");
	std::vector<int> conn(set.size());
	std::iota(conn.begin(), conn.end(), 0);
	this->add_point_set(label, set, conn);
}
void stark::DeformablesMeshOutput::add_point_set(const std::string& label, const PointSetHandler& set, const std::vector<int>& conn)
{
	set.exit_if_not_valid("DeformablesMeshOutput::add_point_set");
	std::vector<std::array<int, 1>> conn_1;
	for (int i = 0; i < conn.size(); i++) {
		conn_1.push_back({ conn[i] });
	}

	this->points.push_back({ set, conn_1, conn });
	this->point_groups.add_to_group(label, (int)this->points.size() - 1);
}
void stark::DeformablesMeshOutput::add_segment_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 2>>& conn)
{
	set.exit_if_not_valid("DeformablesMeshOutput::add_segment_mesh");
	std::vector<int> point_set_map(set.size());
	std::iota(point_set_map.begin(), point_set_map.end(), 0);
	this->add_segment_mesh(label, set, conn, point_set_map);
}
void stark::DeformablesMeshOutput::add_segment_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 2>>& conn, const std::vector<int>& point_set_map)
{
	set.exit_if_not_valid("DeformablesMeshOutput::add_segment_mesh");
	this->segments.push_back({ set, conn, point_set_map });
	this->segment_groups.add_to_group(label, (int)this->segments.size() - 1);
}
void stark::DeformablesMeshOutput::add_triangle_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 3>>& conn)
{
	set.exit_if_not_valid("DeformablesMeshOutput::add_triangle_mesh");
	std::vector<int> point_set_map(set.size());
	std::iota(point_set_map.begin(), point_set_map.end(), 0);
	this->add_triangle_mesh(label, set, conn, point_set_map);	
}
void stark::DeformablesMeshOutput::add_triangle_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 3>>& conn, const std::vector<int>& point_set_map)
{
	set.exit_if_not_valid("DeformablesMeshOutput::add_triangle_mesh");
	this->triangles.push_back({ set, conn, point_set_map });
	this->triangle_groups.add_to_group(label, (int)this->triangles.size() - 1);
}
void stark::DeformablesMeshOutput::add_tet_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 4>>& conn)
{
	set.exit_if_not_valid("DeformablesMeshOutput::add_tet_mesh");
	std::vector<int> point_set_map(set.size());
	std::iota(point_set_map.begin(), point_set_map.end(), 0);
	this->add_tet_mesh(label, set, conn, point_set_map);
}
void stark::DeformablesMeshOutput::add_tet_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 4>>& conn, const std::vector<int>& point_set_map)
{
	set.exit_if_not_valid("DeformablesMeshOutput::add_tet_mesh");
	this->tets.push_back({ set, conn, point_set_map });
	this->tet_groups.add_to_group(label, (int)this->tets.size() - 1);
}

void stark::DeformablesMeshOutput::_write_frame(stark::core::Stark& stark)
{
	this->_write(stark, this->point_groups, this->points, conn_points);
	this->_write(stark, this->segment_groups, this->segments, conn_segments);
	this->_write(stark, this->triangle_groups, this->triangles, conn_triangles);
	this->_write(stark, this->tet_groups, this->tets, conn_tets);
}

template<std::size_t N>
inline void stark::DeformablesMeshOutput::_write(stark::core::Stark& stark, const stark::MeshOutputGroups& groups, const std::vector<stark::DeformablesMeshOutput::Mesh<N>>& meshes, std::vector<std::array<int, N>>& conn_buffer)
{
	for (const auto& it : groups.groups) {
		const std::string& label = it.first;
		const std::unordered_set<int>& group = it.second;

		this->vertices.clear();
		conn_buffer.clear();
		for (const int local_idx : group) {
			const auto& mesh = meshes[local_idx];

			const int out_vertex_offset = (int)this->vertices.size();
			const int dyn_vertices_begin = mesh.set.get_begin();
			for (int loc = 0; loc < mesh.point_set_map.size(); loc++) {
				const int global = mesh.point_set_map[loc];
				this->vertices.push_back(dyn->x1[dyn_vertices_begin + global]);
			}

			for (const auto& elem : mesh.conn) {
				conn_buffer.push_back(elem);
				for (int i = 0; i < N; i++) {
					conn_buffer.back()[i] += out_vertex_offset;
				}
			}
		}
		stark::write_VTK(stark.get_frame_path(label) + ".vtk", this->vertices, conn_buffer);
	}
}
// Forward declaration
template void stark::DeformablesMeshOutput::_write<1>(stark::core::Stark& stark, const stark::MeshOutputGroups& groups, const std::vector<stark::DeformablesMeshOutput::Mesh<1>>& meshes, std::vector<std::array<int, 1>>& conn_buffer);
template void stark::DeformablesMeshOutput::_write<2>(stark::core::Stark& stark, const stark::MeshOutputGroups& groups, const std::vector<stark::DeformablesMeshOutput::Mesh<2>>& meshes, std::vector<std::array<int, 2>>& conn_buffer);
template void stark::DeformablesMeshOutput::_write<3>(stark::core::Stark& stark, const stark::MeshOutputGroups& groups, const std::vector<stark::DeformablesMeshOutput::Mesh<3>>& meshes, std::vector<std::array<int, 3>>& conn_buffer);
template void stark::DeformablesMeshOutput::_write<4>(stark::core::Stark& stark, const stark::MeshOutputGroups& groups, const std::vector<stark::DeformablesMeshOutput::Mesh<4>>& meshes, std::vector<std::array<int, 4>>& conn_buffer);
