#include "MultiMeshEdges.h"

#include "mesh_utils.h"

stark::utils::MultiMeshEdges::MultiMeshEdges(const MultiMesh<3>& triangle_mesh)
{
	this->connectivity.clear();
	utils::find_edges(this->connectivity, triangle_mesh.connectivity, triangle_mesh.get_n_vertices());

	// Find groups
	this->connectivity_offsets = {0};
	for (int mesh_i = 0; mesh_i < triangle_mesh.get_n_meshes(); mesh_i++) {
		const int end = triangle_mesh.get_vertices_range(mesh_i)[1];
		const int idx = (int)std::distance(this->connectivity.begin(), std::find_if(this->connectivity.begin(), this->connectivity.end(), [&](const std::array<int, 2>& edge) { return edge[0] >= end; }));
		this->connectivity_offsets.push_back(idx);
	}
}

int stark::utils::MultiMeshEdges::get_n_meshes() const
{
	return (int)(this->connectivity_offsets.size() - 1);
}

int stark::utils::MultiMeshEdges::get_n_edges(const int mesh_i) const
{
	return this->connectivity_offsets[mesh_i + 1] - this->connectivity_offsets[mesh_i];
}

int stark::utils::MultiMeshEdges::get_n_edges() const
{
	return this->connectivity_offsets.back();
}

std::array<int, 2> stark::utils::MultiMeshEdges::get_edges_range(const int mesh_i) const
{
	return { this->connectivity_offsets[mesh_i], this->connectivity_offsets[mesh_i + 1] };
}
