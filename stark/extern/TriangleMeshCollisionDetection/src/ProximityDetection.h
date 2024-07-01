#pragma once
#include <vector>
#include <array>

#include "BroadPhasePTEE.h"
#include "ProximityResults.h"

namespace tmcd
{
	class ProximityDetection 
	{
	public:
		/* Methods */
		ProximityDetection() = default;
		~ProximityDetection() = default;

		void clear();
		void set_edge_edge_parallel_cutoff(const double cross_norm_sq);
		void set_n_threads(const int32_t n_threads);
		int32_t get_n_threads() const;
		void set_max_recursion(const int32_t cap = 20);
		void set_recursion_cap(const int32_t cap = 1500);
		int32_t get_n_meshes() const;
		int32_t add_mesh(const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges);
		void add_blacklist_range_point_triangle(const int32_t point_mesh_id, const std::array<int32_t, 2>& loc_point_interval, const int32_t triangle_mesh_id, const std::array<int32_t, 2>& loc_triangle_interval);
		void add_blacklist_range_edge_edge(const int32_t mesh_id_0, const std::array<int32_t, 2>& loc_interval_0, const int32_t mesh_id_1, const std::array<int32_t, 2>& loc_interval_1);
		void add_blacklist(const int32_t mesh_id_0, const int32_t mesh_id_1);
		void activate_point_triangle(const bool activate);
		void activate_edge_edge(const bool activate);

		const ProximityResults& run(const double enlargement, const BroadPhaseStrategy strat = BroadPhaseStrategy::OctreeSIMD);
		const BroadPhasePTEEResults& get_broad_phase_results() const;
		const ProximityResults& get_narrow_phase_results() const;
		const info::ProximityDetection get_info() const;

	private:

		/* Fields */
		BroadPhasePTEE bp;
		internals::Meshes meshes;
		double edge_edge_parallel_cross_norm_sq_cutoff = 1e-30;
		bool is_point_triangle_enabled = true;
		bool is_edge_edge_enabled = true;

		// Results
		ProximityResults results;
		std::vector<ProximityResults> thread_results;
		double runtime_solve_pt;
		double runtime_solve_ee;
		double runtime_merge;
		double runtime_total;
	};
}
