#pragma once
#include <vector>
#include <array>

#include "BroadPhaseBase.h"
#include "info_structs.h"


namespace tmcd
{
	namespace internals
	{
		class BroadPhasePTEEBase
			: public internals::BroadPhaseBase
		{
		public:
			/* Methods */
			BroadPhasePTEEBase() = default;
			~BroadPhasePTEEBase() = default;

			void clear();
			void add_blacklist_range_point_triangle(const int32_t point_mesh_id, const std::array<int32_t, 2>& point_interval, const int32_t triangle_mesh_id, const std::array<int32_t, 2>& triangle_interval);
			void add_blacklist_range_edge_edge(const int32_t mesh_id_0, const std::array<int32_t, 2>& interval_0, const int32_t mesh_id_1, const std::array<int32_t, 2>& interval_1);

			const BroadPhasePTEEResults& get_results() const;
			info::BroadPhasePTEE get_info() const;

		protected:

			/* Methods */
			const BroadPhasePTEEResults& _run(const bool is_ccd, const double enlargement, const BroadPhaseStrategy strat = BroadPhaseStrategy::OctreeSIMD);
			void _run_broad_phase_bruteforce();
			void _run_octree_leaf_scalar(const OctreeNode& leaf);
			void _run_octree_leaf_simd(const OctreeNode& leaf);

			/* Fields */
			// Blacklists
			std::vector<internals::BlacklistInterval> blacklist_triangle_point;
			std::vector<uint8_t> has_blacklist_triangle_point;
			std::vector<internals::BlacklistInterval> blacklist_edge_edge;
			std::vector<uint8_t> has_blacklist_edge_edge;

			// Results
			BroadPhasePTEEResults results;
			double runtime_solve;
			double runtime_merge;
			double runtime_total;
		};
	}
}