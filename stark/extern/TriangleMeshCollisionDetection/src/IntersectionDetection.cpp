#include "IntersectionDetection.h"

#include <omp.h>

#include "helpers.h"
#include "ipc_toolkit_geometry_functions.h"

using namespace tmcd;
using namespace internals;

void IntersectionDetection::set_n_threads(const int32_t n_threads)
{
	this->bp.set_n_threads(n_threads);
}
int tmcd::IntersectionDetection::get_n_threads() const
{
	return this->bp.get_n_threads();
}
void IntersectionDetection::set_recursion_cap(const int32_t cap)
{
	this->bp.set_recursion_cap(cap);
}
void IntersectionDetection::set_max_recursion(const int32_t max_recursion)
{
	this->bp.set_max_recursion(max_recursion);
}
void IntersectionDetection::clear()
{
	this->bp.clear();
	this->results.clear();
	this->meshes.clear();
}
int32_t IntersectionDetection::get_n_meshes() const
{
	return this->bp.get_n_meshes();
}
int IntersectionDetection::add_mesh(const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges)
{
	this->meshes.add_mesh(nullptr, nullptr, xm, n_vertices, triangles, n_triangles, edges, n_edges);
	return this->bp.add_mesh(xm, n_vertices, triangles, n_triangles, edges, n_edges);
}
void tmcd::IntersectionDetection::add_blacklist_range_edge_triangle(const int32_t edge_mesh_id, const std::array<int32_t, 2>& loc_edge_interval, const int32_t triangle_mesh_id, const std::array<int32_t, 2>& loc_triangle_interval)
{
	this->bp.add_blacklist_range_edge_triangle(edge_mesh_id, loc_edge_interval, triangle_mesh_id, loc_triangle_interval);
}
void tmcd::IntersectionDetection::add_blacklist(const int32_t mesh_id_0, const int32_t mesh_id_1)
{
	this->add_blacklist_range_edge_triangle(mesh_id_0, { 0, this->meshes.n_edges_in_set[mesh_id_0] }, mesh_id_1, { 0, this->meshes.n_triangles_in_set[mesh_id_1] });
	if (mesh_id_0 != mesh_id_1) {
		this->add_blacklist_range_edge_triangle(mesh_id_1, { 0, this->meshes.n_edges_in_set[mesh_id_1] }, mesh_id_0, { 0, this->meshes.n_triangles_in_set[mesh_id_0] });
	}
}

const IntersectionResults& tmcd::IntersectionDetection::run(const BroadPhaseStrategy strat)
{
	const double total_t0 = omp_get_wtime();

	// Run broad phase
	this->bp.run(strat);

	// Clear thread buffers
	const int32_t n_threads = this->bp.get_n_threads();
	this->thread_results.resize(n_threads);
	for (IntersectionResults& thread_buffer : this->thread_results) {
		thread_buffer.clear();
	}

	// Run
	double t0 = omp_get_wtime();
	const std::vector<std::pair<SetIndex, SetIndex>>& et_pairs = this->bp.get_results().edge_triangle;
	#pragma omp parallel for schedule(static) num_threads(n_threads)
	for (int32_t pair_i = 0; pair_i < (int32_t)et_pairs.size(); pair_i++) {
		const SetIndex& edge = et_pairs[pair_i].first;
		const SetIndex& triangle = et_pairs[pair_i].second;

		const std::array<int32_t, 2> e = this->meshes.get_edge_connectivity(edge.set, edge.idx);
		const std::array<int32_t, 3> t = this->meshes.get_triangle_connectivity(triangle.set, triangle.idx);

		const Vec3d e0 = this->meshes.get_coords(edge.set, e[0]);
		const Vec3d e1 = this->meshes.get_coords(edge.set, e[1]);
		const Vec3d t0 = this->meshes.get_coords(triangle.set, t[0]);
		const Vec3d t1 = this->meshes.get_coords(triangle.set, t[1]);
		const Vec3d t2 = this->meshes.get_coords(triangle.set, t[2]);

		// Narrow phase
		if (is_edge_intersecting_triangle(e0, e1, t0, t1, t2)) {
			const int thread_id = omp_get_thread_num();
			this->thread_results[thread_id].edge_triangle.push_back({ {edge.set, edge.idx, {e[0], e[1]}}, {triangle.set, triangle.idx, {t[0], t[1], t[2]}} });
		}
	}
	double t1 = omp_get_wtime();
	this->runtime_solve = t1 - t0;

	// Merge thread results
	t0 = omp_get_wtime();
	std::vector<std::vector<std::pair<Edge, Triangle>>*> thread_edge_triangle(n_threads);
	for (int thread_id = 0; thread_id < n_threads; thread_id++) {
		thread_edge_triangle[thread_id] = &this->thread_results[thread_id].edge_triangle;
	}
	parallel_concat(this->results.edge_triangle, thread_edge_triangle);
	t1 = omp_get_wtime();
	this->runtime_merge = t1 - t0;

	const double total_t1 = omp_get_wtime();
	this->runtime_total = total_t1 - total_t0;

	return this->results;
}
const BroadPhaseETResults& tmcd::IntersectionDetection::get_broad_phase_results() const
{
	return this->bp.get_results();
}
const IntersectionResults& tmcd::IntersectionDetection::get_narrow_phase_results() const
{
	return this->results;
}

info::IntersectionDetection tmcd::IntersectionDetection::get_info() const
{
	info::IntersectionDetection info;
	info.broad_phase = this->bp.get_info();
	info.n_intersecting_edge_triangle = (int32_t)this->results.edge_triangle.size();
	info.n_threads = this->bp.get_n_threads();
	info.runtime_compute_aabbs = info.broad_phase.runtime_compute_aabbs;
	info.runtime_octree = info.broad_phase.runtime_octree;
	info.runtime_broad_phase_solve = info.broad_phase.runtime_solve;
	info.runtime_broad_phase_merge = info.broad_phase.runtime_merge;
	info.runtime_narrow_phase_solve = this->runtime_solve;
	info.runtime_narrow_phase_merge = this->runtime_merge;
	info.runtime_total = this->runtime_total;
	return info;
}
