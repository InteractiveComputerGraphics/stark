#include "ProximityDetection.h"

#include <iostream>
#include <omp.h>

#include "helpers.h"
#include "ipc_toolkit_geometry_functions.h"

using namespace tmcd;
using namespace internals;

void ProximityDetection::set_n_threads(const int32_t n_threads)
{
	this->bp.set_n_threads(n_threads);
}
int tmcd::ProximityDetection::get_n_threads() const
{
	return this->bp.get_n_threads();
}
void ProximityDetection::set_recursion_cap(const int32_t cap)
{
	this->bp.set_recursion_cap(cap);
}
void ProximityDetection::set_max_recursion(const int32_t max_recursion)
{
	this->bp.set_max_recursion(max_recursion);
}
void ProximityDetection::clear()
{
	this->bp.clear();
	this->results.clear();
}
void tmcd::ProximityDetection::set_edge_edge_parallel_threshold(const double cross_norm_sq)
{
	this->edge_edge_parallel_cross_norm_sq_threshold = cross_norm_sq;
}
void tmcd::ProximityDetection::set_edge_edge_parallel_cutoff(const double cross_norm_sq)
{
	this->edge_edge_parallel_cross_norm_sq_cutoff = cross_norm_sq;
}
int32_t ProximityDetection::get_n_meshes() const
{
	return this->bp.get_n_meshes();
}
int ProximityDetection::add_mesh(const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges)
{
	this->meshes.add_mesh(nullptr, nullptr, xm, n_vertices, triangles, n_triangles, edges, n_edges);
	return this->bp.add_mesh(xm, n_vertices, triangles, n_triangles, edges, n_edges);
}
void tmcd::ProximityDetection::add_blacklist_range_point_triangle(const int32_t point_mesh_id, const std::array<int32_t, 2>& point_interval, const int32_t triangle_mesh_id, const std::array<int32_t, 2>& triangle_interval)
{
	this->bp.add_blacklist_range_point_triangle(point_mesh_id, point_interval, triangle_mesh_id, triangle_interval);
}
void tmcd::ProximityDetection::add_blacklist_range_edge_edge(const int32_t mesh_id_0, const std::array<int32_t, 2>& interval_0, const int32_t mesh_id_1, const std::array<int32_t, 2>& interval_1)
{
	this->bp.add_blacklist_range_edge_edge(mesh_id_0, interval_0, mesh_id_1, interval_1);
}
void tmcd::ProximityDetection::disable_point_triangle(const bool disable)
{
	this->is_point_triangle_disabled = disable;
}
void tmcd::ProximityDetection::disable_edge_edge(const bool disable)
{
	this->is_edge_edge_disabled = disable;
}

const ProximityResults& tmcd::ProximityDetection::run(const double enlargement, const BroadPhaseStrategy strat)
{
	//if (this->edge_edge_parallel_cross_norm_sq_threshold < 0.0) {
	//	std::cout << "tmcd::ProximityDetection::run() error: edge-edge parallel threshold not set. Use ProximityDetection.set_edge_edge_parallel_threshold()." << std::endl;
	//	exit(-1);
	//}

	const double total_t0 = omp_get_wtime();

	// Run broad phase
	this->bp.run(enlargement, strat);

	// Clear thread buffers
	const int32_t n_threads = this->bp.get_n_threads();
	this->thread_results.resize(n_threads);
	for (ProximityResults& thread_buffer : this->thread_results) {
		thread_buffer.clear();
	}

	// Run
	const double enlargement_sq = enlargement * enlargement;

	//// Point - Triangle
	double t0 = omp_get_wtime();
	const std::vector<std::pair<SetIndex, SetIndex>>& pt_pairs = this->bp.get_results().point_triangle;
	if (!this->is_point_triangle_disabled) {
		#pragma omp parallel for schedule(static) num_threads(n_threads)
		for (int32_t pair_i = 0; pair_i < (int32_t)pt_pairs.size(); pair_i++) {
			const SetIndex& point = pt_pairs[pair_i].first;
			const SetIndex& triangle = pt_pairs[pair_i].second;
			const std::array<int32_t, 3> t = this->meshes.get_triangle_connectivity(triangle.set, triangle.idx);

			const Vec3d p = this->meshes.get_coords(point.set, point.idx);
			const Vec3d a = this->meshes.get_coords(triangle.set, t[0]);
			const Vec3d b = this->meshes.get_coords(triangle.set, t[1]);
			const Vec3d c = this->meshes.get_coords(triangle.set, t[2]);

			// Narrow phase
			PointTriangleDistanceType nearest_entity;
			const double dist_sq = point_triangle_sq_distance(nearest_entity, p, a, b, c);
			//const double dist_sq = point_triangle_sq_unsigned_jan(nearest_entity, p, a, b, c);  // Much faster but doesn't correctly detect cornercases
			if (dist_sq < enlargement_sq) {

				const int thread_id = omp_get_thread_num();
				switch (nearest_entity) {
				case PointTriangleDistanceType::P_T0:
					this->thread_results[thread_id].point_point.push_back({ {point.set, point.idx}, {triangle.set, t[0]} });
					break;

				case PointTriangleDistanceType::P_T1:
					this->thread_results[thread_id].point_point.push_back({ {point.set, point.idx}, {triangle.set, t[1]} });
					break;

				case PointTriangleDistanceType::P_T2:
					this->thread_results[thread_id].point_point.push_back({ {point.set, point.idx}, {triangle.set, t[2]} });
					break;

				case PointTriangleDistanceType::P_E0:
					this->thread_results[thread_id].point_edge.push_back({ {point.set, point.idx}, {triangle.set, t[0], t[1]} });
					break;

				case PointTriangleDistanceType::P_E1:
					this->thread_results[thread_id].point_edge.push_back({ {point.set, point.idx}, {triangle.set, t[1], t[2]} });
					break;

				case PointTriangleDistanceType::P_E2:
					this->thread_results[thread_id].point_edge.push_back({ {point.set, point.idx}, {triangle.set, t[2], t[0]} });
					break;

				case PointTriangleDistanceType::P_T:
					this->thread_results[thread_id].point_triangle.push_back({ {point.set, point.idx}, {triangle.set, {t[0], t[1], t[2]}} });
					break;
				}
			}
		}
	}
	double t1 = omp_get_wtime();
	this->runtime_solve_pt = t1 - t0;

	//// Edge - Edge
	t0 = omp_get_wtime();
	const std::vector<std::pair<SetIndex, SetIndex>>& ee_pairs = this->bp.get_results().edge_edge;
	if (!this->is_edge_edge_disabled) {
		#pragma omp parallel for schedule(static) num_threads(n_threads)
		for (int32_t pair_i = 0; pair_i < (int32_t)ee_pairs.size(); pair_i++) {
			const SetIndex& edge_a = ee_pairs[pair_i].first;
			const SetIndex& edge_b = ee_pairs[pair_i].second;

			const std::array<int32_t, 2> ea = this->meshes.get_edge_connectivity(edge_a.set, edge_a.idx);
			const std::array<int32_t, 2> eb = this->meshes.get_edge_connectivity(edge_b.set, edge_b.idx);

			const Vec3d a = this->meshes.get_coords(edge_a.set, ea[0]);
			const Vec3d b = this->meshes.get_coords(edge_a.set, ea[1]);
			const Vec3d p = this->meshes.get_coords(edge_b.set, eb[0]);
			const Vec3d q = this->meshes.get_coords(edge_b.set, eb[1]);

			EdgeEdgeDistanceType nearest_entity;
			const double dist_sq = edge_edge_sq_distance(nearest_entity, a, b, p, q, this->edge_edge_parallel_cross_norm_sq_threshold);
			if (dist_sq < enlargement_sq) {
				const int thread_id = omp_get_thread_num();

				// All cases (not only edge-edge) that are almost parallel are classified to edge-edge
				const double cross_norm_sq = (b - a).cross(q - p).squaredNorm();
				if (cross_norm_sq <= this->edge_edge_parallel_cross_norm_sq_cutoff) {
					continue;
				}
				//const bool almost_parallel = cross_norm_sq < this->edge_edge_parallel_cross_norm_sq_threshold;

				// TODO: I have to mollify all cases, not just EA_EB

				const Edge EA = { edge_a.set, { ea[0], ea[1] } };
				const Edge EB = { edge_b.set, { eb[0], eb[1] } };
				const Point EA0 = { edge_a.set, ea[0] };
				const Point EA1 = { edge_a.set, ea[1] };
				const Point EB0 = { edge_b.set, eb[0] };
				const Point EB1 = { edge_b.set, eb[1] };

				switch (nearest_entity) {
				case EdgeEdgeDistanceType::EA0_EB0: this->thread_results[thread_id].ee_point_point.push_back({ {EA, EA0}, {EB, EB0} }); break;
				case EdgeEdgeDistanceType::EA0_EB1: this->thread_results[thread_id].ee_point_point.push_back({ {EA, EA0}, {EB, EB1} }); break;
				case EdgeEdgeDistanceType::EA1_EB0: this->thread_results[thread_id].ee_point_point.push_back({ {EA, EA1}, {EB, EB0} }); break;
				case EdgeEdgeDistanceType::EA1_EB1: this->thread_results[thread_id].ee_point_point.push_back({ {EA, EA1}, {EB, EB1} }); break;

				case EdgeEdgeDistanceType::EA_EB0: this->thread_results[thread_id].ee_point_edge.push_back({ {EB, EB0}, EA }); break;
				case EdgeEdgeDistanceType::EA_EB1: this->thread_results[thread_id].ee_point_edge.push_back({ {EB, EB1}, EA }); break;
				case EdgeEdgeDistanceType::EA0_EB: this->thread_results[thread_id].ee_point_edge.push_back({ {EA, EA0}, EB }); break;
				case EdgeEdgeDistanceType::EA1_EB: this->thread_results[thread_id].ee_point_edge.push_back({ {EA, EA1}, EB }); break;

				case EdgeEdgeDistanceType::EA_EB: this->thread_results[thread_id].ee_edge_edge.push_back({ EA, EB }); break;
				}
			}
		}
	}
	t1 = omp_get_wtime();
	this->runtime_solve_ee = t1 - t0;

	// Merge thread results
	t0 = omp_get_wtime();
	std::vector<std::vector<std::pair<Point, Point>>*> thread_point_point(n_threads);
	std::vector<std::vector<std::pair<Point, Edge>>*> thread_point_edge(n_threads);
	std::vector<std::vector<std::pair<Point, Triangle>>*> thread_point_triangle(n_threads);
	std::vector<std::vector<std::pair<Edge, Edge>>*> thread_edge_edge(n_threads);

	// DEBUG
	std::vector<std::vector<std::pair<EdgePoint, EdgePoint>>*> thread_ee_point_point(n_threads);
	std::vector<std::vector<std::pair<EdgePoint, Edge>>*> thread_ee_point_edge(n_threads);
	std::vector<std::vector<std::pair<Edge, Edge>>*> thread_ee_edge_edge(n_threads);

	for (int thread_id = 0; thread_id < n_threads; thread_id++) {
		thread_point_point[thread_id] = &this->thread_results[thread_id].point_point;
		thread_point_edge[thread_id] = &this->thread_results[thread_id].point_edge;
		thread_point_triangle[thread_id] = &this->thread_results[thread_id].point_triangle;
		//thread_edge_edge[thread_id] = &this->thread_results[thread_id].edge_edge;

		// DEBUG
		thread_ee_point_point[thread_id] = &this->thread_results[thread_id].ee_point_point;
		thread_ee_point_edge[thread_id] = &this->thread_results[thread_id].ee_point_edge;
		thread_ee_edge_edge[thread_id] = &this->thread_results[thread_id].ee_edge_edge;
	}
	parallel_concat(this->results.point_point, thread_point_point);
	parallel_concat(this->results.point_edge, thread_point_edge);
	parallel_concat(this->results.point_triangle, thread_point_triangle);
	//parallel_concat(this->results.edge_edge, thread_edge_edge);

	// DEBUG
	parallel_concat(this->results.ee_point_point, thread_ee_point_point);
	parallel_concat(this->results.ee_point_edge, thread_ee_point_edge);
	parallel_concat(this->results.ee_edge_edge, thread_ee_edge_edge);


	t1 = omp_get_wtime();
	this->runtime_merge = t1 - t0;

	const double total_t1 = omp_get_wtime();
	this->runtime_total = total_t1 - total_t0;

	return this->results;
}
const BroadPhasePTEEResults& tmcd::ProximityDetection::get_broad_phase_results() const
{
	return this->bp.get_results();
}
const ProximityResults& tmcd::ProximityDetection::get_narrow_phase_results() const
{
	return this->results;
}

const info::ProximityDetection tmcd::ProximityDetection::get_info() const
{
	info::ProximityDetection info;
	info.broad_phase = this->bp.get_info();
	info.n_proximal_point_point = (int32_t)this->results.point_point.size();
	info.n_proximal_point_edge = (int32_t)this->results.point_edge.size();
	info.n_proximal_point_triangle = (int32_t)this->results.point_triangle.size();
	//info.n_proximal_edge_edge = (int32_t)this->results.edge_edge.size();
	info.n_threads = this->bp.get_n_threads();
	info.runtime_compute_aabbs = info.broad_phase.runtime_compute_aabbs;
	info.runtime_octree = info.broad_phase.runtime_octree;
	info.runtime_broad_phase_solve = info.broad_phase.runtime_solve;
	info.runtime_broad_phase_merge = info.broad_phase.runtime_merge;
	info.runtime_narrow_phase_solve_pt = this->runtime_solve_pt;
	info.runtime_narrow_phase_solve_ee = this->runtime_solve_ee;
	info.runtime_narrow_phase_merge = this->runtime_merge;
	info.runtime_total = this->runtime_total;
	return info;
}
