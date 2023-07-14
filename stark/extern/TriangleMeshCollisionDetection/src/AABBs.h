#pragma once
#include <vector>
#include <array>

#include "Meshes.h"
#include "types.h"  // TODO: bring AABB here

namespace tmcd
{
	namespace internals
	{
		class AABBs
		{
		public:
			/* Fields */
			std::vector<AABB> aabbs;
			int32_t first_point_idx = -1;
			int32_t first_triangle_idx = -1;
			int32_t first_edge_idx = -1;
			std::array<float, 3> world_bottom;
			std::array<float, 3> world_top;
			double runtime;

			/* Methods */
			void compute(const Meshes& meshes, const double enlargement, const bool ccd, const bool add_points, const bool add_triangles, const bool add_edges, const int n_threads);
			const AABB& get_point_aabb(const int32_t i) const;
			const AABB& get_triangle_aabb(const int32_t i) const;
			const AABB& get_edge_aabb(const int32_t i) const;
		};
	}
}