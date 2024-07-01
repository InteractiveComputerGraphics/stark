#pragma once
#include <array>
#include <iostream>

namespace tmcd
{
	namespace info
	{
		struct Octree
		{
		public:
			std::array<float, 3> bottom;
			std::array<float, 3> top;
			int32_t recursion_cap;
			int32_t max_recursion;
			int32_t n_threads;
			int32_t n_aabbs;
			int32_t n_leaves;
			int32_t avg_n_aabbs_per_leaf;
			double runtime;

			void print() const
			{
				std::cout << " ~~~~~~~~~~~~~~~ Octree ~~~~~~~~~~~~~~~" << std::endl;
				std::cout << "bottom: [" << bottom[0] << ", " << bottom[1] << ", " << bottom[2] << "]\n";
				std::cout << "top: [" << top[0] << ", " << top[1] << ", " << top[2] << "]\n";
				std::cout << "recursion cap: " << recursion_cap << "\n";
				std::cout << "max recursion: " << max_recursion << "\n";
				std::cout << "# threads: " << n_threads << "\n";
				std::cout << "# AABBs: " << n_aabbs << "\n";
				std::cout << "# octree leaves: " << n_leaves << "\n";
				std::cout << "avg # AABBs per leaf: " << avg_n_aabbs_per_leaf << "\n";
				std::cout << "runtime: " << 1000.0*runtime << "ms\n";
				std::cout << std::flush;
			}
		};
		struct Meshes
		{
			int32_t n_meshes;
			int32_t n_points;
			int32_t n_edges;
			int32_t n_triangles;

			void print() const
			{
				std::cout << " ~~~~~~~~~~~~~~~ Meshes ~~~~~~~~~~~~~~~" << std::endl;
				std::cout << "# meshes: " << n_meshes << "\n";
				std::cout << "# points: " << n_points << "\n";
				std::cout << "# edges: " << n_edges << "\n";
				std::cout << "# triangles: " << n_triangles << "\n";
				std::cout << std::flush;
			}
		};
		struct BroadPhaseET
		{
			Meshes meshes;
			Octree octree;
			int32_t n_blacklist_range_edge_triangle;
			int32_t n_overlaps_found_edge_triangle;
			double runtime_compute_aabbs;
			double runtime_octree;
			double runtime_solve;
			double runtime_merge;
			double runtime_total;
			int32_t n_threads;

			void print() const
			{
				std::cout << " --------------- BroadPhaseET ---------------" << std::endl;
				this->meshes.print();
				this->octree.print();
				std::cout << " ~~~~~~~~~~~~~~~ BroadPhaseET ~~~~~~~~~~~~~~~" << std::endl;
				std::cout << "# blacklist ranges edge-triangle: " << n_blacklist_range_edge_triangle << "\n";
				std::cout << "# overlaps found edge-triangle: " << n_overlaps_found_edge_triangle << "\n";
				std::cout << "# threads: " << n_threads << "\n";
				std::cout << "runtime compute AABBs: " << 1000.0*runtime_compute_aabbs << "ms\n";
				std::cout << "runtime octree: " << 1000.0*runtime_octree << "ms\n";
				std::cout << "runtime solve: " << 1000.0*runtime_solve << "ms\n";
				std::cout << "runtime merge: " << 1000.0*runtime_merge << "ms\n";
				std::cout << "runtime total: " << 1000.0*runtime_total << "ms\n";
				std::cout << std::flush;
			}
		};
		struct BroadPhasePTEE
		{
			Meshes meshes;
			Octree octree;
			int32_t n_blacklist_range_point_triangle;
			int32_t n_blacklist_range_edge_edge;
			int32_t n_overlaps_found_point_triangle;
			int32_t n_overlaps_found_edge_edge;
			double runtime_compute_aabbs;
			double runtime_octree;
			double runtime_solve;
			double runtime_merge;
			double runtime_total;
			int32_t n_threads;

			void print() const
			{
				std::cout << " --------------- BroadPhasePTEE ---------------" << std::endl;
				this->meshes.print();
				this->octree.print();
				std::cout << " ~~~~~~~~~~~~~~~ BroadPhasePTEE ~~~~~~~~~~~~~~~" << std::endl;
				std::cout << "# blacklist ranges point-triangle: " << n_blacklist_range_point_triangle << "\n";
				std::cout << "# blacklist ranges edge-edge: " << n_blacklist_range_edge_edge << "\n";
				std::cout << "# overlaps found point-triangle: " << n_overlaps_found_point_triangle << "\n";
				std::cout << "# overlaps found edge-edge: " << n_overlaps_found_edge_edge << "\n";
				std::cout << "# threads: " << n_threads << "\n";
				std::cout << "runtime compute AABBs: " << 1000.0 * runtime_compute_aabbs << "ms\n";
				std::cout << "runtime octree: " << 1000.0 * runtime_octree << "ms\n";
				std::cout << "runtime solve: " << 1000.0 * runtime_solve << "ms\n";
				std::cout << "runtime merge: " << 1000.0 * runtime_merge << "ms\n";
				std::cout << "runtime total: " << 1000.0 * runtime_total << "ms\n";
				std::cout << std::flush;
			}
		};
		struct IntersectionDetection
		{
			BroadPhaseET broad_phase;
			int32_t n_intersecting_edge_triangle;
			double runtime_compute_aabbs;
			double runtime_octree;
			double runtime_broad_phase_solve;
			double runtime_broad_phase_merge;
			double runtime_narrow_phase_solve;
			double runtime_narrow_phase_merge;
			double runtime_total;
			int32_t n_threads;

			void print(const bool only_runtime = false) const
			{
				if (!only_runtime) {
					std::cout << " =============== IntersectionDetection ===============" << std::endl;
					this->broad_phase.print();
					std::cout << " ~~~~~~~~~~~~~~~ IntersectionDetection ~~~~~~~~~~~~~~~" << std::endl;
					std::cout << "# intersecting edge-triangle: " << n_intersecting_edge_triangle << "\n";
					std::cout << "# threads: " << n_threads << "\n";
				}
				std::cout << "runtime compute AABBs: " << 1000.0 * runtime_compute_aabbs << "ms\n";
				std::cout << "runtime octree: " << 1000.0 * runtime_octree << "ms\n";
				std::cout << "runtime broad phase solve: " << 1000.0 * runtime_broad_phase_solve << "ms\n";
				std::cout << "runtime broad phase merge: " << 1000.0 * runtime_broad_phase_merge << "ms\n";
				std::cout << "runtime narrow phase solve: " << 1000.0 * runtime_narrow_phase_solve << "ms\n";
				std::cout << "runtime narrow phase merge: " << 1000.0 * runtime_narrow_phase_merge << "ms\n";
				std::cout << "runtime total: " << 1000.0 * runtime_total << "ms\n";
				std::cout << std::flush;
			}
		};
		struct ProximityDetection
		{
			BroadPhasePTEE broad_phase;
			int32_t n_proximal_pt_point_point;
			int32_t n_proximal_pt_point_edge;
			int32_t n_proximal_pt_point_triangle;
			int32_t n_proximal_ee_point_point;
			int32_t n_proximal_ee_point_edge;
			int32_t n_proximal_ee_edge_edge;
			double runtime_compute_aabbs;
			double runtime_octree;
			double runtime_broad_phase_solve;
			double runtime_broad_phase_merge;
			double runtime_narrow_phase_solve_pt;
			double runtime_narrow_phase_solve_ee;
			double runtime_narrow_phase_merge;
			double runtime_total;
			int32_t n_threads;

			void print(const bool only_runtime = false) const
			{
				if (!only_runtime) {
					std::cout << " =============== ProximityDetection ===============" << std::endl;
					this->broad_phase.print();
					std::cout << " ~~~~~~~~~~~~~~~ ProximityDetection ~~~~~~~~~~~~~~~" << std::endl;
					std::cout << "# proximal point-triangle point-point: " << n_proximal_pt_point_point << "\n";
					std::cout << "# proximal point-triangle point-edge: " << n_proximal_pt_point_edge << "\n";
					std::cout << "# proximal point-triangle point-triangle: " << n_proximal_pt_point_triangle << "\n";
					std::cout << "# proximal edge-edge point-point: " << n_proximal_ee_point_point << "\n";
					std::cout << "# proximal edge-edge point-edge: " << n_proximal_ee_point_edge << "\n";
					std::cout << "# proximal edge-edge edge-edge: " << n_proximal_ee_edge_edge << "\n";
					std::cout << "# threads: " << n_threads << "\n";
				}
				std::cout << "runtime compute AABBs: " << 1000.0 * runtime_compute_aabbs << "ms\n";
				std::cout << "runtime octree: " << 1000.0 * runtime_octree << "ms\n";
				std::cout << "runtime broad phase solve: " << 1000.0 * runtime_broad_phase_solve << "ms\n";
				std::cout << "runtime broad phase merge: " << 1000.0 * runtime_broad_phase_merge << "ms\n";
				std::cout << "runtime narrow phase solve point-triangle: " << 1000.0 * runtime_narrow_phase_solve_pt << "ms\n";
				std::cout << "runtime narrow phase solve edge-edge: " << 1000.0 * runtime_narrow_phase_solve_ee << "ms\n";
				std::cout << "runtime narrow phase merge: " << 1000.0 * runtime_narrow_phase_merge << "ms\n";
				std::cout << "runtime total: " << 1000.0 * runtime_total << "ms\n";
				std::cout << std::flush;
			}
		};
	}
}