#pragma once
#include <vector>
#include <array>

#include "RecursiveBuffer.h"
#include "info_structs.h"
#include "types.h"


namespace tmcd
{
	/*
		This is just the Octree classification. The output will still need to be further process to actually obtain overlaps.
		The overlap tests are not performed in here because they need further specialization to account for Scalar/SIMD, orphan sets, blacklists, symmetric searches, etc
	*/
	class Octree
	{
	public:

		/* Methods */
		void set_recursion_cap(const int32_t cap);
		void set_max_recursion(const int32_t max_recursion);
		const std::vector<OctreeNode*>& run(const std::vector<AABB>& aabbs, const std::array<float, 3>& bottom, const std::array<float, 3>& top);
		const std::vector<OctreeNode*>& get_leaves() const;
		info::Octree get_info() const;
	
	private:
		using RecursiveOctreeNode = internals::RecursiveBuffer<OctreeNode, 8>;

		/* Fields */
		std::vector<OctreeNode*> octree_leaves;
		int32_t max_recursion = 20;
		int32_t recursion_cap = 1500;
		RecursiveOctreeNode root;
		info::Octree info;

		/* Methods */
		void _run_octree_node(RecursiveOctreeNode& node_buffer, const int32_t depth, const std::vector<AABB>& aabbs);
	};
}