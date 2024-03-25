#pragma once
#include <vector>
#include <array>
#include <cstdint>

#ifdef TMCD_ENABLE_AVX
#include <immintrin.h>
#include "alignment_allocator.h"
#endif

namespace tmcd
{
	enum class BroadPhaseStrategy
	{
		Bruteforce,
		Octree,
		OctreeSIMD
	};
	struct SetIndex
	{
		int32_t set;
		int32_t idx;
	};
	struct BroadPhasePTEEResults
	{
		std::vector<std::pair<SetIndex, SetIndex>> point_triangle;
		std::vector<std::pair<SetIndex, SetIndex>> edge_edge;
		void clear()
		{
			this->point_triangle.clear();
			this->edge_edge.clear();
		};
	};
	struct BroadPhaseETResults
	{
		std::vector<std::pair<SetIndex, SetIndex>> edge_triangle;
		void clear()
		{
			this->edge_triangle.clear();
		};
	};
	struct Point
	{
		int32_t set;
		int32_t idx;
	};
	struct Edge
	{
		int32_t set;
		int32_t idx;
		std::array<int32_t, 2> vertices;
	};
	struct Triangle
	{
		int32_t set;
		int32_t idx;
		std::array<int32_t, 3> vertices;
	};
	struct OctreeNode
	{
		std::array<float, 3> bottom;
		std::array<float, 3> top;
		std::vector<int> aabb_indices;
	};
	struct AABB
	{
		int32_t set;
		int32_t idx;
		std::array<float, 3> bottom;
		std::array<float, 3> top;
	};

	namespace internals
	{
		struct Interval
		{
			int32_t begin = -1;
			int32_t end = -1;
		};
		struct BlacklistInterval
		{
			Interval from; // Not offset by AABB idx
			Interval to;
		};

		struct ThreadBuffer
		{
#ifdef TMCD_ENABLE_AVX
			struct LeafSIMDBuffer
			{
				struct AABBList
				{
					avector<std::array<__m256, 3>, 32> bottom;
					avector<std::array<__m256, 3>, 32> top;
					std::vector<__m256i> set;
					std::vector<__m256i> idx;
				};

				AABBList points;
				AABBList edges;
				std::vector<__m256i> edge_vertex0;
				std::vector<__m256i> edge_vertex1;
				std::vector<int32_t> local_collision_sets;
				std::vector<int32_t> local_collision_idxs;
			};

			LeafSIMDBuffer simd;
#endif

			std::vector<Interval> blacklist_buffer;
			BroadPhasePTEEResults broad_phase_results_pt_ee;
			BroadPhaseETResults broad_phase_results_et;
			std::vector<AABB> aabbs;
		};
	}
}
