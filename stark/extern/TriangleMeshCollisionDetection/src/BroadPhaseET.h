#pragma once
#include <vector>
#include <array>

#include "BroadPhaseBase.h"
#include "info_structs.h"


namespace tmcd
{
	class BroadPhaseET 
		: public internals::BroadPhaseBase
	{
	public:

		/* Methods */
		BroadPhaseET() = default;
		~BroadPhaseET() = default;

		void clear();
		int32_t add_mesh(const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges);
		void add_blacklist_range_edge_triangle(const int32_t edge_mesh_id, const std::array<int32_t, 2>& edge_interval, const int32_t triangle_mesh_id, const std::array<int32_t, 2>& triangle_interval);
		
		const BroadPhaseETResults& run(const BroadPhaseStrategy strat = BroadPhaseStrategy::OctreeSIMD);
		const BroadPhaseETResults& get_results() const;
		info::BroadPhaseET get_info() const;

	private:

		/* Methods */
		void _run_broad_phase_bruteforce();
		void _run_octree_leaf_scalar(const OctreeNode& leaf);
#ifdef TMCD_ENABLE_AVX
		void _run_octree_leaf_simd(const OctreeNode& leaf);
#endif

		/* Fields */
		// Blacklists
		std::vector<internals::BlacklistInterval> blacklist_triangle_edge;
		std::vector<uint8_t> has_blacklist_triangle_edge;

		// Results
		BroadPhaseETResults results;
		double runtime_solve;
		double runtime_merge;
		double runtime_total;
	};
}
