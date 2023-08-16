#pragma once
#include <vector>
#include <array>
#include <algorithm>
#include <cassert>

#include <Eigen/Dense>

#include "MultiMesh.h"

namespace stark::utils
{
	/* =========================  DEFINITIONS  ========================= */
	class MultiMeshEdges
	{
	public:
		/* Fields */
		std::vector<std::array<int, 2>> connectivity;
		std::vector<int> connectivity_offsets;

		/* Methods */
		MultiMeshEdges() = default;
		MultiMeshEdges(const MultiMesh<3>& triangle_mesh);

		int get_n_meshes() const;
		int get_n_edges(const int mesh_i) const;
		int get_n_edges() const;
		std::array<int, 2> get_edges_range(const int mesh_i) const;
		//int get_mesh_containing_edge(const int element_i) const;
		//int get_local_edge_idx(const int mesh_i, const int element_i) const;
		//int get_global_edge_idx(const int mesh_i, const int local_element_i) const;
	};
}
