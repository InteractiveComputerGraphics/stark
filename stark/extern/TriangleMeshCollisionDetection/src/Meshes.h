#pragma once
#include <vector>
#include <array>

#include "Vec3.h"
#include "info_structs.h"

namespace tmcd
{
	namespace internals
	{
		class Meshes
		{
		public:

			/* Fields */
			std::vector<const int32_t*> set_triangles;
			std::vector<int32_t> n_triangles_in_set;
			std::vector<int32_t> triangles_offsets;
			std::vector<const int32_t*> set_edges;
			std::vector<int32_t> n_edges_in_set;
			std::vector<int32_t> edges_offsets;
			std::vector<const double*> set_x0;
			std::vector<const double*> set_x1;
			std::vector<const double*> set_xm;
			std::vector<int32_t> n_points_in_set;
			std::vector<int32_t> points_offsets;

			/* Methods */
			const std::array<int32_t, 2> get_edge_connectivity(const int32_t set, const int32_t idx) const;
			const std::array<int32_t, 3> get_triangle_connectivity(const int32_t set, const int32_t idx) const;
			Vec3d get_coords(const int32_t set, const int32_t idx) const;
			int32_t get_total_n_points() const;
			int32_t get_total_n_edges() const;
			int32_t get_total_n_triangles() const;
			int32_t get_n_meshes() const;
			std::array<int32_t, 2> get_vertices_range(const int32_t set) const;
			std::array<int32_t, 2> get_triangles_range(const int32_t set) const;
			std::array<int32_t, 2> get_edges_range(const int32_t set) const;
			info::Meshes get_info() const;
			int32_t add_mesh(const double* x0, const double* x1, const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges);
			void clear();
		};
	}
}