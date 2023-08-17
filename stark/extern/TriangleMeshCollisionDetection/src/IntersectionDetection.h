#pragma once
#include <vector>
#include <array>

#include "BroadPhaseET.h"
#include "IntersectionResults.h"

namespace tmcd
{
	class IntersectionDetection 
	{
	public:
		/* Methods */
		IntersectionDetection() = default;
		~IntersectionDetection() = default;

		void clear();
		void set_n_threads(const int32_t n_threads);
		int32_t get_n_threads() const;
		void set_max_recursion(const int32_t cap = 20);
		void set_recursion_cap(const int32_t cap = 1500);
		int32_t get_n_meshes() const;
		int32_t add_mesh(const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges);
		void add_blacklist_range_edge_triangle(const int32_t edge_mesh_id, const std::array<int32_t, 2>& loc_edge_interval, const int32_t triangle_mesh_id, const std::array<int32_t, 2>& loc_triangle_interval);
		void add_blacklist(const int32_t mesh_id_0, const int32_t mesh_id_1);

		const IntersectionResults& run(const BroadPhaseStrategy strat = BroadPhaseStrategy::OctreeSIMD);
		const BroadPhaseETResults& get_broad_phase_results() const;
		const IntersectionResults& get_narrow_phase_results() const;
		info::IntersectionDetection get_info() const;

	private:

		/* Fields */
		BroadPhaseET bp;
		internals::Meshes meshes;

		// Results
		IntersectionResults results;
		std::vector<IntersectionResults> thread_results;
		double runtime_solve;
		double runtime_merge;
		double runtime_total;
	};
}
