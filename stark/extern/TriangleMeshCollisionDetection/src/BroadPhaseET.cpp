#include "BroadPhaseET.h"

#include "helpers.h"
#include "shuffle_lut.h"

using namespace tmcd;
using namespace internals;

void BroadPhaseET::clear()
{
	// Input
	this->meshes.clear();
	this->blacklist_triangle_edge.clear();

	// Output
	this->results.clear();
}
int BroadPhaseET::add_mesh(const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges)
{
	return this->meshes.add_mesh(nullptr, nullptr, xm, n_vertices, triangles, n_triangles, edges, n_edges);
}
void tmcd::BroadPhaseET::add_blacklist_range_edge_triangle(const int32_t edge_mesh_id, const std::array<int32_t, 2>& edge_interval, const int32_t triangle_mesh_id, const std::array<int32_t, 2>& triangle_interval)
{
	BlacklistInterval bl;
	bl.from.begin = triangle_interval[0] + this->meshes.triangles_offsets[triangle_mesh_id];
	bl.from.end = triangle_interval[1] + this->meshes.triangles_offsets[triangle_mesh_id];
	bl.to.begin = edge_interval[0] + this->meshes.edges_offsets[edge_mesh_id];
	bl.to.end = edge_interval[1] + this->meshes.edges_offsets[edge_mesh_id];
	this->blacklist_triangle_edge.push_back(bl);
}

const BroadPhaseETResults& tmcd::BroadPhaseET::run(const BroadPhaseStrategy strat)
{
	double total_t0 = omp_get_wtime();
	this->_init_threads();

	// Compute AABBs
	const double enlargement = 0.0;
	const bool ccd = false;
	const bool add_points = false;
	const bool add_triangles = true;
	const bool add_edges = true;
	const int n_threads = this->n_threads;
	this->aabbs.compute(this->meshes, enlargement, ccd, add_points, add_triangles, add_edges, n_threads);

	// Prepare blacklists
	this->has_blacklist_triangle_edge.resize(this->meshes.get_total_n_triangles());
	std::fill(this->has_blacklist_triangle_edge.begin(), this->has_blacklist_triangle_edge.end(), static_cast<uint8_t>(false));
	for (const BlacklistInterval& bl : this->blacklist_triangle_edge) {
		for (int32_t tri_i = bl.from.begin; tri_i < bl.from.end; tri_i++) {
			this->has_blacklist_triangle_edge[tri_i] = static_cast<uint8_t>(true);
		}
	}

	// Clear thread buffers
	for (ThreadBuffer& thread_buffer : this->thread_buffers) {
		thread_buffer.broad_phase_results_et.clear();
	}

	// Run
	double t0 = omp_get_wtime();
	if (strat == BroadPhaseStrategy::Bruteforce) {
		
		// Bruteforce all
		this->_run_broad_phase_bruteforce();
	}
	else if (strat == BroadPhaseStrategy::Octree || strat == BroadPhaseStrategy::OctreeSIMD) {
		
		// Octree to gather AABBs closeby
		const std::vector<OctreeNode*> leaves = this->octree.run(this->aabbs.aabbs, this->aabbs.world_bottom, this->aabbs.world_top);

		// Find actual overlapping AABBs
#ifdef TMCD_ENABLE_AVX
		if (strat == BroadPhaseStrategy::Octree) {
#else
		if (strat == BroadPhaseStrategy::Octree || strat == BroadPhaseStrategy::OctreeSIMD) {
#endif
			#pragma omp parallel for schedule(static) num_threads(this->n_threads)
			for (int leaf_i = 0; leaf_i < (int)leaves.size(); leaf_i++) {
				this->_run_octree_leaf_scalar(*leaves[leaf_i]);
			}
		}
#ifdef TMCD_ENABLE_AVX
		else if (strat == BroadPhaseStrategy::OctreeSIMD) {
			#pragma omp parallel for schedule(static) num_threads(this->n_threads)
			for (int leaf_i = 0; leaf_i < (int)leaves.size(); leaf_i++) {
				this->_run_octree_leaf_simd(*leaves[leaf_i]);
			}
		}
#endif
	}
	double t1 = omp_get_wtime();
	this->runtime_solve = t1 - t0;

	// Merge
	t0 = omp_get_wtime();
	std::vector<std::vector<std::pair<SetIndex, SetIndex>>*> thread_edge_triangle(this->n_threads);
	for (int thread_id = 0; thread_id < this->n_threads; thread_id++) {
		thread_edge_triangle[thread_id] = &this->thread_buffers[thread_id].broad_phase_results_et.edge_triangle;
	}
	parallel_concat(this->results.edge_triangle, thread_edge_triangle);
	t1 = omp_get_wtime();
	this->runtime_merge = t1 - t0;

	double total_t1 = omp_get_wtime();
	this->runtime_total = total_t1 - total_t0;

	return this->results;
}
const BroadPhaseETResults& tmcd::BroadPhaseET::get_results() const
{
	return this->results;
}
tmcd::info::BroadPhaseET tmcd::BroadPhaseET::get_info() const
{
	info::BroadPhaseET info;
	info.octree = this->octree.get_info();
	info.meshes = this->meshes.get_info();
	info.n_blacklist_range_edge_triangle = (int32_t)this->blacklist_triangle_edge.size();
	info.n_overlaps_found_edge_triangle = (int32_t)this->results.edge_triangle.size();
	info.runtime_compute_aabbs = this->aabbs.runtime;
	info.runtime_octree = info.octree.runtime;
	info.runtime_solve = this->runtime_solve;
	info.runtime_merge = this->runtime_merge;
	info.runtime_total = this->runtime_total;
	info.n_threads = this->get_n_threads();
	return info;
}

// =================================================================================================================================================

void tmcd::BroadPhaseET::_run_broad_phase_bruteforce()
{
	const int32_t n_triangles = (int32_t)this->meshes.triangles_offsets.back();
	const int32_t n_edges = (int32_t)this->meshes.edges_offsets.back();

	#pragma omp parallel for schedule(static) num_threads(this->n_threads)
	for (int32_t tri_i = 0; tri_i < n_triangles; tri_i++) {
			
		ThreadBuffer& thread_buffer = this->thread_buffers[omp_get_thread_num()];
			
		const AABB& triangle = this->aabbs.get_triangle_aabb(tri_i);
		const std::array<int32_t, 3> t = this->meshes.get_triangle_connectivity(triangle.set, triangle.idx);
			
		const bool has_blacklist = (bool)this->has_blacklist_triangle_edge[tri_i];
		if (has_blacklist) {
			thread_buffer.blacklist_buffer.clear();
			for (const BlacklistInterval& bl : this->blacklist_triangle_edge) {
				if (bl.from.begin <= tri_i && tri_i < bl.from.end) {
					thread_buffer.blacklist_buffer.push_back(bl.to);
				}
			}
		}

		for (int32_t edge_i = 0; edge_i < n_edges; edge_i++) {
			const AABB& edge = this->aabbs.get_edge_aabb(edge_i);
			const std::array<int32_t, 2> e = this->meshes.get_edge_connectivity(edge.set, edge.idx);

			if (overlap(triangle, edge)) {

				// Orphan set
				bool orphan_set_discard = (triangle.set == edge.set) && 
					((e[0] == t[0]) || (e[0] == t[1]) || (e[0] == t[2]) ||
					 (e[1] == t[0]) || (e[1] == t[1]) || (e[1] == t[2]));
				if (!orphan_set_discard) {

					// Blacklist ranges
					bool blacklist_discard = false;
					if (has_blacklist) {
						for (const Interval& interval : thread_buffer.blacklist_buffer) {
							if (interval.begin <= edge_i && edge_i < interval.end) {
								blacklist_discard = true;
								break;
							}
						}
					}
					if (!blacklist_discard) {
						thread_buffer.broad_phase_results_et.edge_triangle.push_back({ {edge.set, edge.idx}, {triangle.set, triangle.idx} });
					}
				}
			}
		}
	}
}
void tmcd::BroadPhaseET::_run_octree_leaf_scalar(const OctreeNode& leaf)
{
	// Preparation
	const int thread_id = omp_get_thread_num();
	ThreadBuffer& thread_buffer = this->thread_buffers[thread_id];

	//// Find n entities
	const int32_t n_aabbs = (int32_t)leaf.aabb_indices.size();
	const int32_t first_triangle_idx_loc = 0;
	const int32_t first_edge_idx_loc = (int32_t)std::distance(leaf.aabb_indices.begin(), std::lower_bound(leaf.aabb_indices.begin(), leaf.aabb_indices.end(), this->aabbs.first_edge_idx));
	const int32_t n_triangles = first_edge_idx_loc;
	const int32_t n_edges = n_aabbs - first_edge_idx_loc;

	//// Copy AABBs to a contiguous local container
	thread_buffer.aabbs.resize(n_aabbs);
	for (int i = 0; i < n_aabbs; i++) {
		thread_buffer.aabbs[i] = this->aabbs.aabbs[leaf.aabb_indices[i]];
	}

	// Bruteforce
	for (int32_t loc_tri_i = 0; loc_tri_i < n_triangles; loc_tri_i++) {
		const int32_t loc_aabb_i = loc_tri_i + first_triangle_idx_loc;
		const AABB& triangle = thread_buffer.aabbs[loc_aabb_i];
		const std::array<int32_t, 3> t = this->meshes.get_triangle_connectivity(triangle.set, triangle.idx);
			
		const int32_t tri_i = this->meshes.triangles_offsets[triangle.set] + triangle.idx;
		const bool has_blacklist = (bool)this->has_blacklist_triangle_edge[tri_i];
		if (has_blacklist) {
			thread_buffer.blacklist_buffer.clear();
			for (const BlacklistInterval& bl : this->blacklist_triangle_edge) {
				if (bl.from.begin <= tri_i && tri_i < bl.from.end) {
					thread_buffer.blacklist_buffer.push_back(bl.to);
				}
			}
		}

		for (int32_t loc_edge_i = 0; loc_edge_i < n_edges; loc_edge_i++) {
			const int32_t loc_aabb_i = loc_edge_i + first_edge_idx_loc;
			const AABB& edge = thread_buffer.aabbs[loc_aabb_i];
			const std::array<int32_t, 2> e = this->meshes.get_edge_connectivity(edge.set, edge.idx);

			// Geometrical intersection of AABBs
			if (overlap(triangle, edge)) {

				// Orphan set
				bool orphan_set_discard = (triangle.set == edge.set) &&
					((e[0] == t[0]) || (e[0] == t[1]) || (e[0] == t[2]) ||
					 (e[1] == t[0]) || (e[1] == t[1]) || (e[1] == t[2]));
				if (!orphan_set_discard) {

					// Octree duplicity avoidance
					//// Intersection bottom
					const std::array<float, 3> ib = { 
						std::max(triangle.bottom[0], edge.bottom[0]),  
						std::max(triangle.bottom[1], edge.bottom[1]),  
						std::max(triangle.bottom[2], edge.bottom[2]),  
					};

					//// Is ib contained in the current leaf?
					const bool correct_leaf =
						leaf.bottom[0] <= ib[0] && ib[0] < leaf.top[0] &&
						leaf.bottom[1] <= ib[1] && ib[1] < leaf.top[1] &&
						leaf.bottom[2] <= ib[2] && ib[2] < leaf.top[2];

					// Blacklist ranges
					if (correct_leaf) {
						bool blacklist_discard = false;
						if (has_blacklist) {
							const int32_t edge_i = this->meshes.edges_offsets[edge.set] + edge.idx;
							for (const Interval& interval : thread_buffer.blacklist_buffer) {
								if (interval.begin <= edge_i && edge_i < interval.end) {
									blacklist_discard = true;
									break;
								}
							}
						}
						if (!blacklist_discard) {
							thread_buffer.broad_phase_results_et.edge_triangle.push_back({ {edge.set, edge.idx}, {triangle.set, triangle.idx} });
						}
					}
				}
			}
		}
	}
}
#ifdef TMCD_ENABLE_AVX
void tmcd::BroadPhaseET::_run_octree_leaf_simd(const OctreeNode& leaf)
{
	// Preparation
	const int thread_id = omp_get_thread_num();
	ThreadBuffer& thread_buffer = this->thread_buffers[thread_id];

	//// Find n entities
	const int32_t n_aabbs = (int32_t)leaf.aabb_indices.size();
	const int32_t first_triangle_idx_loc = 0;
	const int32_t first_edge_idx_loc = (int32_t)std::distance(leaf.aabb_indices.begin(), std::lower_bound(leaf.aabb_indices.begin(), leaf.aabb_indices.end(), this->aabbs.first_edge_idx));
	const int32_t n_triangles = first_edge_idx_loc;
	const int32_t n_edges = n_aabbs - first_edge_idx_loc;

	// Precomputations
	const std::array<__m256, 3> leaf_bottom = { _mm256_set1_ps(leaf.bottom[0]), _mm256_set1_ps(leaf.bottom[1]), _mm256_set1_ps(leaf.bottom[2]) };
	const std::array<__m256, 3> leaf_top = { _mm256_set1_ps(leaf.top[0]), _mm256_set1_ps(leaf.top[1]), _mm256_set1_ps(leaf.top[2]) };
	const __m256i true_simd = _mm256_cmpeq_epi32(_mm256_set1_epi32(1), _mm256_set1_epi32(1));
	thread_buffer.simd.local_collision_sets.resize(n_aabbs + 8);
	thread_buffer.simd.local_collision_idxs.resize(n_aabbs + 8);

	// Triangle - Edge
	//// SIMD shuffle
	const int32_t n_edge_simd_groups = (n_edges % 8 == 0) ? n_edges / 8 : n_edges / 8 + 1;
	thread_buffer.simd.edges.bottom.resize(n_edge_simd_groups);
	thread_buffer.simd.edges.top.resize(n_edge_simd_groups);
	thread_buffer.simd.edges.set.resize(n_edge_simd_groups);
	thread_buffer.simd.edges.idx.resize(n_edge_simd_groups);
	thread_buffer.simd.edge_vertex0.resize(n_edge_simd_groups);
	thread_buffer.simd.edge_vertex1.resize(n_edge_simd_groups);
	{
		std::array<std::array<float, 8>, 3>* bottom_edge_scalar = reinterpret_cast<std::array<std::array<float, 8>, 3>*>(&thread_buffer.simd.edges.bottom[0][0]);
		std::array<std::array<float, 8>, 3>* top_edge_scalar = reinterpret_cast<std::array<std::array<float, 8>, 3>*>(&thread_buffer.simd.edges.top[0][0]);
		std::array<int32_t, 8>* set_edge_scalar = reinterpret_cast<std::array<int32_t, 8>*>(&thread_buffer.simd.edges.set[0]);
		std::array<int32_t, 8>* idx_edge_scalar = reinterpret_cast<std::array<int32_t, 8>*>(&thread_buffer.simd.edges.idx[0]);
		std::array<int32_t, 8>* edge_vertex0_scalar = reinterpret_cast<std::array<int32_t, 8>*>(&thread_buffer.simd.edge_vertex0[0]);
		std::array<int32_t, 8>* edge_vertex1_scalar = reinterpret_cast<std::array<int32_t, 8>*>(&thread_buffer.simd.edge_vertex1[0]);

		int32_t cursor = 0;
		for (int32_t loc_edge_i = 0; loc_edge_i < n_edges; loc_edge_i++) {
			const int32_t loc_aabb_i = first_edge_idx_loc + loc_edge_i;
			const AABB& edge = this->aabbs.aabbs[leaf.aabb_indices[loc_aabb_i]];
			const std::array<int32_t, 2> edge_conn = this->meshes.get_edge_connectivity(edge.set, edge.idx);
			const int32_t group = cursor / 8;
			const int32_t in_group = cursor % 8;
			bottom_edge_scalar[group][0][in_group] = edge.bottom[0];
			bottom_edge_scalar[group][1][in_group] = edge.bottom[1];
			bottom_edge_scalar[group][2][in_group] = edge.bottom[2];
			top_edge_scalar[group][0][in_group] = edge.top[0];
			top_edge_scalar[group][1][in_group] = edge.top[1];
			top_edge_scalar[group][2][in_group] = edge.top[2];
			set_edge_scalar[group][in_group] = edge.set;
			idx_edge_scalar[group][in_group] = edge.idx;
			edge_vertex0_scalar[group][in_group] = edge_conn[0];
			edge_vertex1_scalar[group][in_group] = edge_conn[1];
			cursor++;
		}
		while (cursor % 8 != 0) {
			const int32_t group = cursor / 8;
			const int32_t in_group = cursor % 8;
			bottom_edge_scalar[group][0][in_group] = std::numeric_limits<float>::max();
			bottom_edge_scalar[group][1][in_group] = std::numeric_limits<float>::max();
			bottom_edge_scalar[group][2][in_group] = std::numeric_limits<float>::max();
			top_edge_scalar[group][0][in_group] = std::numeric_limits<float>::max();
			top_edge_scalar[group][1][in_group] = std::numeric_limits<float>::max();
			top_edge_scalar[group][2][in_group] = std::numeric_limits<float>::max();
			set_edge_scalar[group][in_group] = 0;
			idx_edge_scalar[group][in_group] = 0;
			edge_vertex0_scalar[group][in_group] = -1;
			edge_vertex1_scalar[group][in_group] = -1;
			cursor++;
		}
	}

	//// Triangle - Edge Bruteforce
	for (int32_t loc_tri_i = 0; loc_tri_i < n_triangles; loc_tri_i++) {
		const int32_t loc_aabb_i = loc_tri_i + first_triangle_idx_loc;
		const AABB& triangle = this->aabbs.aabbs[leaf.aabb_indices[loc_aabb_i]];
		const std::array<int32_t, 3> t = this->meshes.get_triangle_connectivity(triangle.set, triangle.idx);

		const std::array<__m256, 3> triangle_bottom = { _mm256_set1_ps(triangle.bottom[0]), _mm256_set1_ps(triangle.bottom[1]), _mm256_set1_ps(triangle.bottom[2]) };
		const std::array<__m256, 3> triangle_top = { _mm256_set1_ps(triangle.top[0]), _mm256_set1_ps(triangle.top[1]), _mm256_set1_ps(triangle.top[2]) };
		const __m256i triangle_set = _mm256_set1_epi32(triangle.set);
		const __m256i triangle_idx = _mm256_set1_epi32(triangle.idx);
		const __m256i triangle_0 = _mm256_set1_epi32(t[0]);
		const __m256i triangle_1 = _mm256_set1_epi32(t[1]);
		const __m256i triangle_2 = _mm256_set1_epi32(t[2]);
		int32_t* local_collisions_sets_cursor = thread_buffer.simd.local_collision_sets.data();
		int32_t* local_collisions_idxs_cursor = thread_buffer.simd.local_collision_idxs.data();

		const int32_t tri_i = this->meshes.triangles_offsets[triangle.set] + triangle.idx;
		const bool has_blacklist = (bool)this->has_blacklist_triangle_edge[tri_i];
		if (has_blacklist) {
			thread_buffer.blacklist_buffer.clear();
			for (const BlacklistInterval& bl : this->blacklist_triangle_edge) {
				if (bl.from.begin <= tri_i && tri_i < bl.from.end) {
					thread_buffer.blacklist_buffer.push_back(bl.to);
				}
			}
		}

		for (int32_t edge_group_i = 0; edge_group_i < n_edge_simd_groups; edge_group_i++) {

			const std::array<__m256, 3>& edge_bottom = thread_buffer.simd.edges.bottom[edge_group_i];
			const std::array<__m256, 3>& edge_top = thread_buffer.simd.edges.top[edge_group_i];
			const __m256i& edge_set = thread_buffer.simd.edges.set[edge_group_i];
			const __m256i& edge_idx = thread_buffer.simd.edges.idx[edge_group_i];
			const __m256i edge_0 = thread_buffer.simd.edge_vertex0[edge_group_i];
			const __m256i edge_1 = thread_buffer.simd.edge_vertex1[edge_group_i];

			// Geometrical intersection of AABBs
			//// Bottom comparisons
			__m256 overlap = _mm256_cmp_ps(triangle_bottom[0], edge_top[0], _CMP_LE_OS);	  // x
			overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(triangle_bottom[1], edge_top[1], _CMP_LE_OS));  // y
			overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(triangle_bottom[2], edge_top[2], _CMP_LE_OS));  // z

			//// Top comparisons
			overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(edge_bottom[0], triangle_top[0], _CMP_LE_OS));  // x
			overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(edge_bottom[1], triangle_top[1], _CMP_LE_OS));  // y
			overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(edge_bottom[2], triangle_top[2], _CMP_LE_OS));  // z
			const __m256i overlapi = _mm256_castps_si256(overlap);

			// Early exit
			if (_mm256_movemask_ps(overlap) == 0) { continue; }

			// Orphan set
			/*
				bool orphan_set_discard = (triangle.set == edge.set) &&
					((e[0] == tri[0]) || (e[0] == tri[1]) || (e[0] == tri[2]) ||
					 (e[1] == tri[0]) || (e[1] == tri[1]) || (e[1] == tri[2]));
			*/
			__m256i same_set = _mm256_cmpeq_epi32(triangle_set, edge_set);
			__m256i same_idx = _mm256_cmpeq_epi32(edge_0, triangle_0);
			same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(edge_0, triangle_1));
			same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(edge_0, triangle_2));
			same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(edge_1, triangle_0));
			same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(edge_1, triangle_1));
			same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(edge_1, triangle_2));
			__m256i orphan_set_discard = _mm256_and_si256(same_set, same_idx);
			__m256i orphan_set_approved = _mm256_andnot_si256(orphan_set_discard, true_simd);

			// Octree duplicity avoidance: Is the bottom corner of intersection contained in the current leaf?
			//// Intersection
			const std::array<__m256, 3> ib = {
				_mm256_max_ps(triangle_bottom[0], edge_bottom[0]),
				_mm256_max_ps(triangle_bottom[1], edge_bottom[1]),
				_mm256_max_ps(triangle_bottom[2], edge_bottom[2]),
			};

			//// Bottom comparisons
			__m256 contained = _mm256_cmp_ps(leaf_bottom[0], ib[0], _CMP_LE_OS);	  // x
			contained = _mm256_and_ps(contained, _mm256_cmp_ps(leaf_bottom[1], ib[1], _CMP_LE_OS));  // y
			contained = _mm256_and_ps(contained, _mm256_cmp_ps(leaf_bottom[2], ib[2], _CMP_LE_OS));  // z

			//// Top comparisons
			contained = _mm256_and_ps(contained, _mm256_cmp_ps(ib[0], leaf_top[0], _CMP_LT_OS));  // x
			contained = _mm256_and_ps(contained, _mm256_cmp_ps(ib[1], leaf_top[1], _CMP_LT_OS));  // y
			contained = _mm256_and_ps(contained, _mm256_cmp_ps(ib[2], leaf_top[2], _CMP_LT_OS));  // z
			const __m256i containedi = _mm256_castps_si256(contained);

			// Combined conditions
			const __m256i collision = _mm256_and_si256(overlapi, _mm256_and_si256(orphan_set_approved, containedi));

			// Push back
			const int mask = _mm256_movemask_ps(_mm256_castsi256_ps(collision));
			const int n_positives = _mm_popcnt_u32(mask);
			const __m256i permutation = *((__m256i*) & shift_lut_32[mask]);
			const __m256i shuffled_sets = _mm256_permutevar8x32_epi32(edge_set, permutation);
			const __m256i shuffled_idxs = _mm256_permutevar8x32_epi32(edge_idx, permutation);
			_mm256_storeu_si256((__m256i*)local_collisions_sets_cursor, shuffled_sets);
			_mm256_storeu_si256((__m256i*)local_collisions_idxs_cursor, shuffled_idxs);
			local_collisions_sets_cursor += n_positives;
			local_collisions_idxs_cursor += n_positives;
		}

		const int n_positives = (int)std::distance(thread_buffer.simd.local_collision_sets.data(), local_collisions_sets_cursor);
		for (int i = 0; i < n_positives; i++) {
			const int32_t edge_set = thread_buffer.simd.local_collision_sets[i];
			const int32_t edge_idx = thread_buffer.simd.local_collision_idxs[i];

			bool blacklist_discard = false;
			if (has_blacklist) {
				const int32_t edge_i = this->meshes.edges_offsets[edge_set] + edge_idx;
				for (const Interval& interval : thread_buffer.blacklist_buffer) {
					if (interval.begin <= edge_i && edge_i < interval.end) {
						blacklist_discard = true;
						break;
					}
				}
			}
			if (!blacklist_discard) {
				thread_buffer.broad_phase_results_et.edge_triangle.push_back({ {edge_set, edge_idx}, {triangle.set, triangle.idx} });
			}
		}
	}
}
#endif
