#include "BroadPhasePTEEBase.h"

#include "helpers.h"
#include "shuffle_lut.h"

using namespace tmcd;
using namespace internals;

void internals::BroadPhasePTEEBase::clear()
{
	// Input
	this->meshes.clear();
	this->blacklist_triangle_point.clear();
	this->blacklist_edge_edge.clear();

	// Output
	this->results.clear();
}
void tmcd::internals::BroadPhasePTEEBase::add_blacklist_range_point_triangle(const int32_t point_mesh_id, const std::array<int32_t, 2>& point_interval, const int32_t triangle_mesh_id, const std::array<int32_t, 2>& triangle_interval)
{
	BlacklistInterval bl;
	bl.from.begin = triangle_interval[0] + this->meshes.triangles_offsets[triangle_mesh_id];
	bl.from.end = triangle_interval[1] + this->meshes.triangles_offsets[triangle_mesh_id];
	bl.to.begin = point_interval[0] + this->meshes.points_offsets[point_mesh_id];
	bl.to.end = point_interval[1] + this->meshes.points_offsets[point_mesh_id];
	this->blacklist_triangle_point.push_back(bl);
}
void tmcd::internals::BroadPhasePTEEBase::add_blacklist_range_edge_edge(const int32_t mesh_id_0, const std::array<int32_t, 2>& interval_0, const int32_t mesh_id_1, const std::array<int32_t, 2>& interval_1)
{
	BlacklistInterval bl;
	bl.from.begin = interval_0[0] + this->meshes.edges_offsets[mesh_id_0];
	bl.from.end = interval_0[1] + this->meshes.edges_offsets[mesh_id_0];
	bl.to.begin = interval_1[0] + this->meshes.edges_offsets[mesh_id_1];
	bl.to.end = interval_1[1] + this->meshes.edges_offsets[mesh_id_1];
	this->blacklist_edge_edge.push_back(bl);

	if (bl.from.begin > bl.to.begin) {
		std::cout << "TriangleMeshCollisionDetection.add_blacklist_range_edge_edge() error: first edge interval should be lower index than the second one." << std::endl;
		exit(-1);
	}
}
void tmcd::internals::BroadPhasePTEEBase::activate_point_triangle(const bool activate)
{
	this->is_point_triangle_active = activate;
}
void tmcd::internals::BroadPhasePTEEBase::activate_edge_edge(const bool activate)
{
	this->is_edge_edge_active = activate;
}

const BroadPhasePTEEResults& tmcd::internals::BroadPhasePTEEBase::_run(const bool is_ccd, const double enlargement, const BroadPhaseStrategy strat)
{
	double total_t0 = omp_get_wtime();
	this->_init_threads();

	// Compute AABBs
	const bool add_points = true;
	const bool add_triangles = true;
	const bool add_edges = true;
	const int n_threads = this->n_threads;
	this->aabbs.compute(this->meshes, enlargement, is_ccd, add_points, add_triangles, add_edges, n_threads);

	// Prepare blacklists
	this->has_blacklist_triangle_point.resize(this->meshes.get_total_n_triangles());
	std::fill(this->has_blacklist_triangle_point.begin(), this->has_blacklist_triangle_point.end(), static_cast<uint8_t>(false));
	for (const BlacklistInterval& bl : this->blacklist_triangle_point) {
		for (int32_t tri_i = bl.from.begin; tri_i < bl.from.end; tri_i++) {
			this->has_blacklist_triangle_point[tri_i] = static_cast<uint8_t>(true);
		}
	}

	this->has_blacklist_edge_edge.resize(this->meshes.get_total_n_edges());
	std::fill(this->has_blacklist_edge_edge.begin(), this->has_blacklist_edge_edge.end(), static_cast<uint8_t>(false));
	for (const BlacklistInterval& bl : this->blacklist_edge_edge) {
		for (int32_t edge_i = bl.from.begin; edge_i < bl.from.end; edge_i++) {
			this->has_blacklist_edge_edge[edge_i] = static_cast<uint8_t>(true);
		}
	}

	// Clear thread buffers
	for (ThreadBuffer& thread_buffer : this->thread_buffers) {
		thread_buffer.broad_phase_results_pt_ee.clear();
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
	std::vector<std::vector<std::pair<SetIndex, SetIndex>>*> thread_point_triangle(this->n_threads);
	std::vector<std::vector<std::pair<SetIndex, SetIndex>>*> thread_edge_edge(this->n_threads);
	for (int thread_id = 0; thread_id < this->n_threads; thread_id++) {
		thread_point_triangle[thread_id] = &this->thread_buffers[thread_id].broad_phase_results_pt_ee.point_triangle;
		thread_edge_edge[thread_id] = &this->thread_buffers[thread_id].broad_phase_results_pt_ee.edge_edge;
	}
	parallel_concat(this->results.point_triangle, thread_point_triangle);
	parallel_concat(this->results.edge_edge, thread_edge_edge);
	t1 = omp_get_wtime();
	this->runtime_merge = t1 - t0;

	double total_t1 = omp_get_wtime();
	this->runtime_total = total_t1 - total_t0;

	return this->results;
}
const BroadPhasePTEEResults& tmcd::internals::BroadPhasePTEEBase::get_results() const
{
	return this->results;
}
info::BroadPhasePTEE tmcd::internals::BroadPhasePTEEBase::get_info() const
{
	info::BroadPhasePTEE info;
	info.octree = this->octree.get_info();
	info.meshes = this->meshes.get_info();
	info.n_blacklist_range_point_triangle = (int32_t)this->blacklist_triangle_point.size();
	info.n_blacklist_range_edge_edge = (int32_t)this->blacklist_edge_edge.size();
	info.n_overlaps_found_point_triangle = (int32_t)this->results.point_triangle.size();
	info.n_overlaps_found_edge_edge = (int32_t)this->results.edge_edge.size();
	info.runtime_compute_aabbs = this->aabbs.runtime;
	info.runtime_octree = info.octree.runtime;
	info.runtime_solve = this->runtime_solve;
	info.runtime_merge = this->runtime_merge;
	info.runtime_total = this->runtime_total;
	info.n_threads = this->get_n_threads();
	return info;
}

// =================================================================================================================================================

void tmcd::internals::BroadPhasePTEEBase::_run_broad_phase_bruteforce()
{
	// Triangle - Point
	{
		const int32_t n_triangles = this->meshes.get_total_n_triangles();
		const int32_t n_points = this->meshes.get_total_n_points();

		if (this->is_point_triangle_active && n_triangles > 0 && n_points > 0) {
			#pragma omp parallel for schedule(static) num_threads(this->n_threads)
			for (int32_t tri_i = 0; tri_i < n_triangles; tri_i++) {
			
				ThreadBuffer& thread_buffer = this->thread_buffers[omp_get_thread_num()];
			
				const AABB& triangle = aabbs.get_triangle_aabb(tri_i);
				const std::array<int32_t, 3> t = this->meshes.get_triangle_connectivity(triangle.set, triangle.idx);
			
				const bool has_blacklist = (bool)this->has_blacklist_triangle_point[tri_i];
				if (has_blacklist) {
					thread_buffer.blacklist_buffer.clear();
					for (const BlacklistInterval& bl : this->blacklist_triangle_point) {
						if (bl.from.begin <= tri_i && tri_i < bl.from.end) {
							thread_buffer.blacklist_buffer.push_back(bl.to);
						}
					}
				}

				for (int32_t point_i = 0; point_i < n_points; point_i++) {
					const AABB& point = aabbs.get_point_aabb(point_i);

					if (overlap(triangle, point)) {

						// Orphan set
						bool orphan_set_discard = (triangle.set == point.set) && ((point.idx == t[0]) || (point.idx == t[1]) || (point.idx == t[2]));
						if (!orphan_set_discard) {

							// Blacklist ranges
							bool blacklist_discard = false;
							if (has_blacklist) {
								for (const Interval& interval : thread_buffer.blacklist_buffer) {
									if (interval.begin <= point_i && point_i < interval.end) {
										blacklist_discard = true;
										break;
									}
								}
							}
							if (!blacklist_discard) {
								thread_buffer.broad_phase_results_pt_ee.point_triangle.push_back({ {point.set, point.idx}, {triangle.set, triangle.idx} });
							}
						}
					}
				}
			}
		}
	}

	// Edge - Edge
	{
		const int32_t n_edges = this->meshes.get_total_n_edges();
		if (this->is_edge_edge_active && n_edges > 1) {
			#pragma omp parallel for schedule(static) num_threads(this->n_threads)
			for (int32_t edge_i = 0; edge_i < n_edges; edge_i++) {
			
				ThreadBuffer& thread_buffer = this->thread_buffers[omp_get_thread_num()];

				const AABB& edge_a = aabbs.get_edge_aabb(edge_i);
				const std::array<int32_t, 2> ea = this->meshes.get_edge_connectivity(edge_a.set, edge_a.idx);
			
				const bool has_blacklist = (bool)this->has_blacklist_edge_edge[edge_i];
				if (has_blacklist) {
					thread_buffer.blacklist_buffer.clear();
					for (const BlacklistInterval& bl : this->blacklist_edge_edge) {
						if (bl.from.begin <= edge_i && edge_i < bl.from.end) {
							thread_buffer.blacklist_buffer.push_back(bl.to);
						}
					}
				}

				for (int32_t edge_j = edge_i + 1; edge_j < n_edges; edge_j++) {
					const AABB& edge_b = aabbs.get_edge_aabb(edge_j);
					const std::array<int32_t, 2> eb = this->meshes.get_edge_connectivity(edge_b.set, edge_b.idx);


					if (overlap(edge_a, edge_b)) {

						// Orphan set
						bool orphan_set_discard = (edge_a.set == edge_b.set) && ((ea[0] == eb[0]) || (ea[0] == eb[1]) || (ea[1] == eb[0]) || (ea[1] == eb[1]));
						if (!orphan_set_discard) {

							// Blacklist ranges
							bool blacklist_discard = false;
							if (has_blacklist) {
								for (const Interval& interval : thread_buffer.blacklist_buffer) {
									if (interval.begin <= edge_j && edge_j < interval.end) {
										blacklist_discard = true;
										break;
									}
								}
							}
							if (!blacklist_discard) {
								thread_buffer.broad_phase_results_pt_ee.edge_edge.push_back({ {edge_a.set, edge_a.idx}, {edge_b.set, edge_b.idx} });
							}
						}
					}
				}
			}
		}
	}
}
void tmcd::internals::BroadPhasePTEEBase::_run_octree_leaf_scalar(const OctreeNode& leaf)
{
	// Preparation
	const int thread_id = omp_get_thread_num();
	ThreadBuffer& thread_buffer = this->thread_buffers[thread_id];

	//// Find n entities
	const int32_t n_aabbs = (int32_t)leaf.aabb_indices.size();
	const int32_t first_triangle_idx_loc = (int32_t)std::distance(leaf.aabb_indices.begin(), std::lower_bound(leaf.aabb_indices.begin(), leaf.aabb_indices.end(), this->aabbs.first_triangle_idx));
	const int32_t first_edge_idx_loc = (int32_t)std::distance(leaf.aabb_indices.begin(), std::lower_bound(leaf.aabb_indices.begin(), leaf.aabb_indices.end(), this->aabbs.first_edge_idx));
	const int32_t n_points = first_triangle_idx_loc;
	const int32_t n_triangles = first_edge_idx_loc - first_triangle_idx_loc;
	const int32_t n_edges = n_aabbs - first_edge_idx_loc;

	//// Copy AABBs to a contiguous local container
	thread_buffer.aabbs.resize(n_aabbs);
	for (int i = 0; i < n_aabbs; i++) {
		thread_buffer.aabbs[i] = this->aabbs.aabbs[leaf.aabb_indices[i]];
	}

	// Bruteforce
	//// Point - Triangle
	if (this->is_point_triangle_active && n_triangles > 0 && n_points > 0) {
		for (int32_t loc_tri_i = 0; loc_tri_i < n_triangles; loc_tri_i++) {
			const int32_t loc_aabb_i = loc_tri_i + first_triangle_idx_loc;
			const AABB& triangle = thread_buffer.aabbs[loc_aabb_i];
			const std::array<int32_t, 3> t = this->meshes.get_triangle_connectivity(triangle.set, triangle.idx);

			const int32_t tri_i = this->meshes.triangles_offsets[triangle.set] + triangle.idx;
			const bool has_blacklist = (bool)this->has_blacklist_triangle_point[tri_i];
			if (has_blacklist) {
				thread_buffer.blacklist_buffer.clear();
				for (const BlacklistInterval& bl : this->blacklist_triangle_point) {
					if (bl.from.begin <= tri_i && tri_i < bl.from.end) {
						thread_buffer.blacklist_buffer.push_back(bl.to);
					}
				}
			}

			for (int32_t loc_point_i = 0; loc_point_i < n_points; loc_point_i++) {
				const int32_t loc_aabb_j = loc_point_i;
				const AABB& point = thread_buffer.aabbs[loc_point_i];

				// Geometrical intersection of AABBs
				if (overlap(triangle, point)) {

					// Orphan set
					bool orphan_set_discard = (triangle.set == point.set) && ((point.idx == t[0]) || (point.idx == t[1]) || (point.idx == t[2]));
					if (!orphan_set_discard) {

						// Octree duplicity avoidance
						//// Intersection bottom
						const std::array<float, 3> ib = {
							std::max(triangle.bottom[0], point.bottom[0]),
							std::max(triangle.bottom[1], point.bottom[1]),
							std::max(triangle.bottom[2], point.bottom[2]),
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
								const int32_t point_i = this->meshes.points_offsets[point.set] + point.idx;
								for (const Interval& interval : thread_buffer.blacklist_buffer) {
									if (interval.begin <= point_i && point_i < interval.end) {
										blacklist_discard = true;
										break;
									}
								}
							}
							if (!blacklist_discard) {
								thread_buffer.broad_phase_results_pt_ee.point_triangle.push_back({ {point.set, point.idx}, {triangle.set, triangle.idx} });
							}
						}
					}
				}
			}
		}
	}

	//// Edge - Edge
	if (this->is_edge_edge_active && n_edges > 1) {
		for (int32_t loc_edge_i = 0; loc_edge_i < n_edges; loc_edge_i++) {

			const int32_t loc_aabb_i = loc_edge_i + first_edge_idx_loc;
			const AABB& edge_a = thread_buffer.aabbs[loc_aabb_i];
			const std::array<int32_t, 2> ea = this->meshes.get_edge_connectivity(edge_a.set, edge_a.idx);

			const int32_t edge_i = this->meshes.edges_offsets[edge_a.set] + edge_a.idx;
			const bool has_blacklist = (bool)this->has_blacklist_edge_edge[edge_i];
			if (has_blacklist) {
				thread_buffer.blacklist_buffer.clear();
				for (const BlacklistInterval& bl : this->blacklist_edge_edge) {
					if (bl.from.begin <= edge_i && edge_i < bl.from.end) {
						thread_buffer.blacklist_buffer.push_back(bl.to);
					}
				}
			}

			for (int32_t loc_edge_j = loc_edge_i + 1; loc_edge_j < n_edges; loc_edge_j++) {
				const int32_t loc_aabb_j = loc_edge_j + first_edge_idx_loc;
				const AABB& edge_b = thread_buffer.aabbs[loc_aabb_j];
				const std::array<int32_t, 2> eb = this->meshes.get_edge_connectivity(edge_b.set, edge_b.idx);

				// Geometrical intersection of AABBs
				if (overlap(edge_a, edge_b)) {

					// Orphan set
					bool orphan_set_discard = (edge_a.set == edge_b.set) && ((ea[0] == eb[0]) || (ea[0] == eb[1]) || (ea[1] == eb[0]) || (ea[1] == eb[1]));
					if (!orphan_set_discard) {

						// Octree duplicity avoidance
						//// Intersection bottom
						const std::array<float, 3> ib = {
							std::max(edge_a.bottom[0], edge_b.bottom[0]),
							std::max(edge_a.bottom[1], edge_b.bottom[1]),
							std::max(edge_a.bottom[2], edge_b.bottom[2]),
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
								const int32_t edge_j = this->meshes.edges_offsets[edge_b.set] + edge_b.idx;
								for (const Interval& interval : thread_buffer.blacklist_buffer) {
									if (interval.begin <= edge_j && edge_j < interval.end) {
										blacklist_discard = true;
										break;
									}
								}
							}
							if (!blacklist_discard) {
								thread_buffer.broad_phase_results_pt_ee.edge_edge.push_back({ {edge_a.set, edge_a.idx}, {edge_b.set, edge_b.idx} });
							}
						}
					}
				}
			}
		}
	}
}
#ifdef TMCD_ENABLE_AVX
void tmcd::internals::BroadPhasePTEEBase::_run_octree_leaf_simd(const OctreeNode& leaf)
{
	// Preparation
	const int thread_id = omp_get_thread_num();
	ThreadBuffer& thread_buffer = this->thread_buffers[thread_id];

	//// Find n entities
	const int32_t n_aabbs = (int32_t)leaf.aabb_indices.size();
	const int32_t first_triangle_idx_loc = (int32_t)std::distance(leaf.aabb_indices.begin(), std::lower_bound(leaf.aabb_indices.begin(), leaf.aabb_indices.end(), this->aabbs.first_triangle_idx));
	const int32_t first_edge_idx_loc = (int32_t)std::distance(leaf.aabb_indices.begin(), std::lower_bound(leaf.aabb_indices.begin(), leaf.aabb_indices.end(), this->aabbs.first_edge_idx));
	const int32_t n_points = first_triangle_idx_loc;
	const int32_t n_triangles = first_edge_idx_loc - first_triangle_idx_loc;
	const int32_t n_edges = n_aabbs - first_edge_idx_loc;

	// Precomputations
	const std::array<__m256, 3> leaf_bottom = { _mm256_set1_ps(leaf.bottom[0]), _mm256_set1_ps(leaf.bottom[1]), _mm256_set1_ps(leaf.bottom[2]) };
	const std::array<__m256, 3> leaf_top = { _mm256_set1_ps(leaf.top[0]), _mm256_set1_ps(leaf.top[1]), _mm256_set1_ps(leaf.top[2]) };
	const __m256i true_simd = _mm256_cmpeq_epi32(_mm256_set1_epi32(1), _mm256_set1_epi32(1));
	thread_buffer.simd.local_collision_sets.resize(n_aabbs + 8);
	thread_buffer.simd.local_collision_idxs.resize(n_aabbs + 8);

	// Point - Triangle
	if (this->is_point_triangle_active && n_triangles > 0 && n_points > 0) {
		//// SIMD shuffle
		const int32_t n_point_simd_groups = (n_points % 8 == 0) ? n_points / 8 : n_points / 8 + 1;
		thread_buffer.simd.points.bottom.resize(n_point_simd_groups);
		thread_buffer.simd.points.top.resize(n_point_simd_groups);
		thread_buffer.simd.points.set.resize(n_point_simd_groups);
		thread_buffer.simd.points.idx.resize(n_point_simd_groups);
		{
			std::array<std::array<float, 8>, 3>* bottom_point_scalar = reinterpret_cast<std::array<std::array<float, 8>, 3>*>(&thread_buffer.simd.points.bottom[0][0]);
			std::array<std::array<float, 8>, 3>* top_point_scalar = reinterpret_cast<std::array<std::array<float, 8>, 3>*>(&thread_buffer.simd.points.top[0][0]);
			std::array<int32_t, 8>* set_point_scalar = reinterpret_cast<std::array<int32_t, 8>*>(&thread_buffer.simd.points.set[0]);
			std::array<int32_t, 8>* idx_point_scalar = reinterpret_cast<std::array<int32_t, 8>*>(&thread_buffer.simd.points.idx[0]);

			int32_t cursor = 0;
			for (int32_t i = 0; i < n_points; i++) {
				const AABB& aabb = this->aabbs.aabbs[leaf.aabb_indices[i]];
				const int32_t group = cursor / 8;
				const int32_t in_group = cursor % 8;
				bottom_point_scalar[group][0][in_group] = aabb.bottom[0];
				bottom_point_scalar[group][1][in_group] = aabb.bottom[1];
				bottom_point_scalar[group][2][in_group] = aabb.bottom[2];
				top_point_scalar[group][0][in_group] = aabb.top[0];
				top_point_scalar[group][1][in_group] = aabb.top[1];
				top_point_scalar[group][2][in_group] = aabb.top[2];
				set_point_scalar[group][in_group] = aabb.set;
				idx_point_scalar[group][in_group] = aabb.idx;
				cursor++;
			}
			while (cursor % 8 != 0) {
				const int32_t group = cursor / 8;
				const int32_t in_group = cursor % 8;
				bottom_point_scalar[group][0][in_group] = std::numeric_limits<float>::max();
				bottom_point_scalar[group][1][in_group] = std::numeric_limits<float>::max();
				bottom_point_scalar[group][2][in_group] = std::numeric_limits<float>::max();
				top_point_scalar[group][0][in_group] = std::numeric_limits<float>::max();
				top_point_scalar[group][1][in_group] = std::numeric_limits<float>::max();
				top_point_scalar[group][2][in_group] = std::numeric_limits<float>::max();
				set_point_scalar[group][in_group] = 0;
				idx_point_scalar[group][in_group] = 0;
				cursor++;
			}
		}


		//// Point - Triangle Bruteforce
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
			const bool has_blacklist = (bool)this->has_blacklist_triangle_point[tri_i];
			if (has_blacklist) {
				thread_buffer.blacklist_buffer.clear();
				for (const BlacklistInterval& bl : this->blacklist_triangle_point) {
					if (bl.from.begin <= tri_i && tri_i < bl.from.end) {
						thread_buffer.blacklist_buffer.push_back(bl.to);
					}
				}
			}

			for (int32_t point_group_i = 0; point_group_i < n_point_simd_groups; point_group_i++) {
				const std::array<__m256, 3>& point_bottom = thread_buffer.simd.points.bottom[point_group_i];
				const std::array<__m256, 3>& point_top = thread_buffer.simd.points.top[point_group_i];
				const __m256i& point_set = thread_buffer.simd.points.set[point_group_i];
				const __m256i& point_idx = thread_buffer.simd.points.idx[point_group_i];

				// Geometrical intersection of AABBs
				//// Bottom comparisons
				__m256 overlap = _mm256_cmp_ps(triangle_bottom[0], point_top[0], _CMP_LE_OS);	  // x
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(triangle_bottom[1], point_top[1], _CMP_LE_OS));  // y
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(triangle_bottom[2], point_top[2], _CMP_LE_OS));  // z

				//// Top comparisons
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(point_bottom[0], triangle_top[0], _CMP_LE_OS));  // x
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(point_bottom[1], triangle_top[1], _CMP_LE_OS));  // y
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(point_bottom[2], triangle_top[2], _CMP_LE_OS));  // z
				const __m256i overlapi = _mm256_castps_si256(overlap);

				// Early exit
				if (_mm256_movemask_ps(overlap) == 0) { continue; }

				// Orphan set
				// bool orphan_set_discard = (triangle.set == edge.set) && ((edge.idx == tri[0]) || (edge.idx == tri[1]) || (edge.idx == tri[2]));
				__m256i same_set = _mm256_cmpeq_epi32(triangle_set, point_set);
				__m256i same_idx = _mm256_cmpeq_epi32(point_idx, triangle_0);
				same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(point_idx, triangle_1));
				same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(point_idx, triangle_2));
				__m256i orphan_set_discard = _mm256_and_si256(same_set, same_idx);
				__m256i orphan_set_approved = _mm256_andnot_si256(orphan_set_discard, true_simd);

				// Octree duplicity avoidance: Is the bottom corner of intersection contained in the current leaf?
				//// Intersection
				const std::array<__m256, 3> ib = {
					_mm256_max_ps(triangle_bottom[0], point_bottom[0]),
					_mm256_max_ps(triangle_bottom[1], point_bottom[1]),
					_mm256_max_ps(triangle_bottom[2], point_bottom[2]),
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
				const __m256i shuffled_sets = _mm256_permutevar8x32_epi32(point_set, permutation);
				const __m256i shuffled_idxs = _mm256_permutevar8x32_epi32(point_idx, permutation);
				_mm256_storeu_si256((__m256i*)local_collisions_sets_cursor, shuffled_sets);
				_mm256_storeu_si256((__m256i*)local_collisions_idxs_cursor, shuffled_idxs);
				local_collisions_sets_cursor += n_positives;
				local_collisions_idxs_cursor += n_positives;
			}

			const int n_positives = (int)std::distance(thread_buffer.simd.local_collision_sets.data(), local_collisions_sets_cursor);
			for (int i = 0; i < n_positives; i++) {
				const int32_t point_set = thread_buffer.simd.local_collision_sets[i];
				const int32_t point_idx = thread_buffer.simd.local_collision_idxs[i];

				bool blacklist_discard = false;
				if (has_blacklist) {
					const int32_t point_i = this->meshes.points_offsets[point_set] + point_idx;
					for (const Interval& interval : thread_buffer.blacklist_buffer) {
						if (interval.begin <= point_i && point_i < interval.end) {
							blacklist_discard = true;
							break;
						}
					}
				}
				if (!blacklist_discard) {
					thread_buffer.broad_phase_results_pt_ee.point_triangle.push_back({ {point_set, point_idx}, {triangle.set, triangle.idx} });
				}
			}
		}
	}

	// Edge - Edge
	if (this->is_edge_edge_active && n_edges > 1) {
		//// SIMD Shuffle
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

		for (int32_t loc_edge_i = 0; loc_edge_i < n_edges; loc_edge_i++) {
			const int32_t loc_aabb_i = first_edge_idx_loc + loc_edge_i;
			const AABB& edge_a = this->aabbs.aabbs[leaf.aabb_indices[loc_aabb_i]];
			const std::array<int32_t, 2> ea = this->meshes.get_edge_connectivity(edge_a.set, edge_a.idx);

			const std::array<__m256, 3> edge_a_bottom = { _mm256_set1_ps(edge_a.bottom[0]), _mm256_set1_ps(edge_a.bottom[1]), _mm256_set1_ps(edge_a.bottom[2]) };
			const std::array<__m256, 3> edge_a_top = { _mm256_set1_ps(edge_a.top[0]), _mm256_set1_ps(edge_a.top[1]), _mm256_set1_ps(edge_a.top[2]) };
			const __m256i edge_a_set = _mm256_set1_epi32(edge_a.set);
			const __m256i edge_a_idx = _mm256_set1_epi32(edge_a.idx);
			const __m256i edge_a_0 = _mm256_set1_epi32(ea[0]);
			const __m256i edge_a_1 = _mm256_set1_epi32(ea[1]);
			int32_t* local_collisions_sets_cursor = thread_buffer.simd.local_collision_sets.data();
			int32_t* local_collisions_idxs_cursor = thread_buffer.simd.local_collision_idxs.data();

			const int32_t edge_i = this->meshes.edges_offsets[edge_a.set] + edge_a.idx;
			const bool has_blacklist = (bool)this->has_blacklist_edge_edge[edge_i];
			if (has_blacklist) {
				thread_buffer.blacklist_buffer.clear();
				for (const BlacklistInterval& bl : this->blacklist_edge_edge) {
					if (bl.from.begin <= edge_i && edge_i < bl.from.end) {
						thread_buffer.blacklist_buffer.push_back(bl.to);
					}
				}
			}

			//// Edge - Edge Bruteforce
			const int32_t begin_group = (loc_edge_i % 8 == 7) ? loc_edge_i / 8 + 1 : loc_edge_i / 8;
			for (int32_t loc_edge_group_j = begin_group; loc_edge_group_j < n_edge_simd_groups; loc_edge_group_j++) {

				const std::array<__m256, 3>& edge_b_bottom = thread_buffer.simd.edges.bottom[loc_edge_group_j];
				const std::array<__m256, 3>& edge_b_top = thread_buffer.simd.edges.top[loc_edge_group_j];
				const __m256i& edge_b_set = thread_buffer.simd.edges.set[loc_edge_group_j];
				const __m256i& edge_b_idx = thread_buffer.simd.edges.idx[loc_edge_group_j];
				const __m256i edge_b_0 = thread_buffer.simd.edge_vertex0[loc_edge_group_j];
				const __m256i edge_b_1 = thread_buffer.simd.edge_vertex1[loc_edge_group_j];

				// Geometrical intersection of AABBs
				//// Bottom comparisons
				__m256 overlap = _mm256_cmp_ps(edge_a_bottom[0], edge_b_top[0], _CMP_LE_OS);	  // x
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(edge_a_bottom[1], edge_b_top[1], _CMP_LE_OS));  // y
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(edge_a_bottom[2], edge_b_top[2], _CMP_LE_OS));  // z

				//// Top comparisons
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(edge_b_bottom[0], edge_a_top[0], _CMP_LE_OS));  // x
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(edge_b_bottom[1], edge_a_top[1], _CMP_LE_OS));  // y
				overlap = _mm256_and_ps(overlap, _mm256_cmp_ps(edge_b_bottom[2], edge_a_top[2], _CMP_LE_OS));  // z
				const __m256i overlapi = _mm256_castps_si256(overlap);

				// Early exit
				if (_mm256_movemask_ps(overlap) == 0) { continue; }

				// Larger idx
				// bool idx_approved = (edge_b.set > edge_a.set) || ((edge_b.set == edge_a.set) && (edge_b.idx > edge_a.idx));
				__m256i greater_set = _mm256_cmpgt_epi32(edge_b_set, edge_a_set);
				__m256i same_set = _mm256_cmpeq_epi32(edge_a_set, edge_b_set);
				__m256i greater_idx = _mm256_cmpgt_epi32(edge_b_idx, edge_a_idx);
				__m256i idx_approved = _mm256_or_si256(greater_set, _mm256_and_si256(same_set, greater_idx));

				// Orphan set
				// bool orphan_set_discard = (edge_a.set == edge_b.set) && ((ea[0] == eb[0]) || (ea[0] == eb[1]) || (ea[1] == eb[0]) || (ea[1] == eb[1]));
				__m256i same_idx = _mm256_cmpeq_epi32(edge_a_0, edge_b_0);
				same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(edge_a_0, edge_b_1));
				same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(edge_a_1, edge_b_0));
				same_idx = _mm256_or_si256(same_idx, _mm256_cmpeq_epi32(edge_a_1, edge_b_1));
				__m256i orphan_set_discard = _mm256_and_si256(same_set, same_idx);
				__m256i orphan_set_approved = _mm256_andnot_si256(orphan_set_discard, true_simd);


				// Octree duplicity avoidance: Is the bottom corner of intersection contained in the current leaf?
				//// Intersection
				const std::array<__m256, 3> ib = {
					_mm256_max_ps(edge_a_bottom[0], edge_b_bottom[0]),
					_mm256_max_ps(edge_a_bottom[1], edge_b_bottom[1]),
					_mm256_max_ps(edge_a_bottom[2], edge_b_bottom[2]),
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
				const __m256i collision = _mm256_and_si256(idx_approved, _mm256_and_si256(overlapi, _mm256_and_si256(orphan_set_approved, containedi)));

				// Push back
				const int mask = _mm256_movemask_ps(_mm256_castsi256_ps(collision));
				const int n_positives = _mm_popcnt_u32(mask);
				const __m256i permutation = *((__m256i*) & shift_lut_32[mask]);
				const __m256i shuffled_sets = _mm256_permutevar8x32_epi32(edge_b_set, permutation);
				const __m256i shuffled_idxs = _mm256_permutevar8x32_epi32(edge_b_idx, permutation);
				_mm256_storeu_si256((__m256i*)local_collisions_sets_cursor, shuffled_sets);
				_mm256_storeu_si256((__m256i*)local_collisions_idxs_cursor, shuffled_idxs);
				local_collisions_sets_cursor += n_positives;
				local_collisions_idxs_cursor += n_positives;
			}

			const int n_positives = (int)std::distance(thread_buffer.simd.local_collision_sets.data(), local_collisions_sets_cursor);
			for (int i = 0; i < n_positives; i++) {
				const int32_t edge_b_set = thread_buffer.simd.local_collision_sets[i];
				const int32_t edge_b_idx = thread_buffer.simd.local_collision_idxs[i];

				bool blacklist_discard = false;
				if (has_blacklist) {
					const int32_t edge_j = this->meshes.edges_offsets[edge_b_set] + edge_b_idx;
					for (const Interval& interval : thread_buffer.blacklist_buffer) {
						if (interval.begin <= edge_j && edge_j < interval.end) {
							blacklist_discard = true;
							break;
						}
					}
				}
				if (!blacklist_discard) {
					thread_buffer.broad_phase_results_pt_ee.edge_edge.push_back({ {edge_a.set, edge_a.idx}, {edge_b_set, edge_b_idx} });
				}
			}
		}
	}
}
#endif
