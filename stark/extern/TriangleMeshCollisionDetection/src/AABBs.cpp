#include "AABBs.h"

#include <limits>
#include <omp.h>


void expand_AABB(tmcd::AABB& aabb, const double* points, const int point_idx)
{
	for (int dim = 0; dim < 3; dim++) {
		const float x = (float)points[3*point_idx + dim];
		aabb.bottom[dim] = std::min(aabb.bottom[dim], x);
		aabb.top[dim] = std::max(aabb.top[dim], x);
	}
}
void enlarge_AABB(tmcd::AABB& aabb, const float enlargement)
{
	for (int dim = 0; dim < 3; dim++) {
		aabb.bottom[dim] -= enlargement;
		aabb.top[dim] += enlargement;
	}
}
void tmcd::internals::AABBs::compute(const Meshes& meshes, const double enlargement, const bool ccd, const bool add_points, const bool add_triangles, const bool add_edges, const int n_threads)
{
	if (ccd) {
		if (meshes.set_x0[0] == nullptr) {
			std::cout << "TriangleMeshCollisionDetection::AABBs error: ccd requested with x0 as nullptr." << std::endl;
			exit(-1);
		}
	}

	double t0 = omp_get_wtime();

	// Number of entities
	const int n_sets = meshes.get_n_meshes();
	const int n_points = (add_points) ? meshes.get_total_n_points() : 0;
	const int n_triangles = (add_triangles) ? meshes.get_total_n_triangles() : 0;
	const int n_edges = (add_edges) ? meshes.get_total_n_edges() : 0;
	const float extra = (float)enlargement + std::numeric_limits<float>::epsilon();
	this->aabbs.resize(n_points + n_triangles + n_edges);

	this->first_point_idx = 0;
	this->first_triangle_idx = n_points;
	this->first_edge_idx = n_points + n_triangles;

	// Points
	if (add_points) {
		for (int set_i = 0; set_i < n_sets; set_i++) {
			const int set_begin = meshes.points_offsets[set_i] + this->first_point_idx;

			#pragma omp parallel for schedule(static) num_threads(n_threads)
			for (int point_i = 0; point_i < meshes.n_points_in_set[set_i]; point_i++) {

				AABB aabb;
				aabb.set = set_i;
				aabb.idx = point_i;
				aabb.bottom = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
				aabb.top = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

				if (ccd) { // This should get branch predicted every single time
					expand_AABB(aabb, meshes.set_x0[set_i], point_i);
					expand_AABB(aabb, meshes.set_x1[set_i], point_i);
				}
				else {
					expand_AABB(aabb, meshes.set_xm[set_i], point_i);
				}

				enlarge_AABB(aabb, extra);
				this->aabbs[set_begin + point_i] = aabb;
			}
		}
	}

	// Triangles
	if (add_triangles) {
		for (int set_i = 0; set_i < n_sets; set_i++) {
			const int set_begin = meshes.triangles_offsets[set_i] + this->first_triangle_idx;

			#pragma omp parallel for schedule(static) num_threads(n_threads)
			for (int tri_i = 0; tri_i < meshes.n_triangles_in_set[set_i]; tri_i++) {

				AABB aabb;
				aabb.set = set_i;
				aabb.idx = tri_i;
				aabb.bottom = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
				aabb.top = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

				for (int loc_point_i = 0; loc_point_i < 3; loc_point_i++) {
					const int point_i = meshes.set_triangles[set_i][3 * tri_i + loc_point_i];

					if (ccd) { // This should get branch predicted every single time
						expand_AABB(aabb, meshes.set_x0[set_i], point_i);
						expand_AABB(aabb, meshes.set_x1[set_i], point_i);
					}
					else {
						expand_AABB(aabb, meshes.set_xm[set_i], point_i);
					}
				}

				enlarge_AABB(aabb, extra);
				this->aabbs[set_begin + tri_i] = aabb;
			}
		}
	}

	// Edges
	if (add_triangles) {
		for (int set_i = 0; set_i < n_sets; set_i++) {
			const int set_begin = meshes.edges_offsets[set_i] + this->first_edge_idx;

			#pragma omp parallel for schedule(static) num_threads(n_threads)
			for (int edge_i = 0; edge_i < meshes.n_edges_in_set[set_i]; edge_i++) {

				AABB aabb;
				aabb.set = set_i;
				aabb.idx = edge_i;
				aabb.bottom = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
				aabb.top = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

				for (int loc_point_i = 0; loc_point_i < 2; loc_point_i++) {
					const int point_i = meshes.set_edges[set_i][2 * edge_i + loc_point_i];

					if (ccd) { // This should get branch predicted every single time
						expand_AABB(aabb, meshes.set_x0[set_i], point_i);
						expand_AABB(aabb, meshes.set_x1[set_i], point_i);
					}
					else {
						expand_AABB(aabb, meshes.set_xm[set_i], point_i);
					}
				}

				enlarge_AABB(aabb, extra);
				this->aabbs[set_begin + edge_i] = aabb;
			}
		}
	}

	// World AABB
	this->world_bottom = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
	this->world_top = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
	for (int set_i = 0; set_i < n_sets; set_i++) {

		#pragma omp parallel num_threads(n_threads)
		{
			const int thread_id = omp_get_thread_num();
			const int n_points = meshes.n_points_in_set[set_i];
			const int chunksize = n_points/n_threads;
			const int begin = chunksize*thread_id;
			const int end = (thread_id == n_threads - 1) ? n_points : chunksize*(thread_id + 1);
				
			AABB thread_aabb;
			thread_aabb.bottom = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
			thread_aabb.top = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
			
			for (int point_i = begin; point_i < end; point_i++) {
				if (ccd) { // This should get branch predicted every single time
					expand_AABB(thread_aabb, meshes.set_x0[set_i], point_i);
					expand_AABB(thread_aabb, meshes.set_x1[set_i], point_i);
				}
				else {
					expand_AABB(thread_aabb, meshes.set_xm[set_i], point_i);
				}
			}

			#pragma omp critical
			{
				for (int dim = 0; dim < 3; dim++) {
					this->world_bottom[dim] = std::min(this->world_bottom[dim], thread_aabb.bottom[dim]);
					this->world_top[dim] = std::max(this->world_top[dim], thread_aabb.top[dim]);
				}
			}
		}
	}
	for (int dim = 0; dim < 3; dim++) {
		this->world_bottom[dim] -= ((float)(2.0*enlargement) + 10.0f*std::numeric_limits<float>::epsilon());
		this->world_top[dim] += ((float)(2.0*enlargement) + 10.0f*std::numeric_limits<float>::epsilon());
	}

	double t1 = omp_get_wtime();
	this->runtime = t1 - t0;
}

const tmcd::AABB& tmcd::internals::AABBs::get_point_aabb(const int32_t i) const
{
	return this->aabbs[this->first_point_idx + i];
}

const tmcd::AABB& tmcd::internals::AABBs::get_triangle_aabb(const int32_t i) const
{
	return this->aabbs[this->first_triangle_idx + i];
}

const tmcd::AABB& tmcd::internals::AABBs::get_edge_aabb(const int32_t i) const
{
	return this->aabbs[this->first_edge_idx + i];
}
