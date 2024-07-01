#include "Octree.h"

#include <numeric>
#include <omp.h>

void tmcd::Octree::set_recursion_cap(const int32_t cap)
{
	this->recursion_cap = cap;
}

void tmcd::Octree::set_max_recursion(const int32_t max_recursion)
{
	this->max_recursion = max_recursion;
}

const std::vector<tmcd::OctreeNode*>& tmcd::Octree::run(const std::vector<AABB>& aabbs, const std::array<float, 3>& bottom, const std::array<float, 3>& top)
{
	// Clear leaves
	this->octree_leaves.clear();

	// Init root
	this->root.buffer.aabb_indices.resize(aabbs.size());
	std::iota(this->root.buffer.aabb_indices.begin(), this->root.buffer.aabb_indices.end(), 0);

	// Run
	this->root.buffer.bottom = bottom;
	this->root.buffer.top = top;
	const double t0 = omp_get_wtime();
	this->_run_octree_node(this->root, 0, aabbs);
	const double t1 = omp_get_wtime();

	// Info
	this->info.bottom = bottom;
	this->info.top = top;
	this->info.n_aabbs = (int32_t)aabbs.size();
	this->info.recursion_cap = this->recursion_cap;
	this->info.max_recursion = this->max_recursion;
	this->info.n_leaves = (int32_t)this->octree_leaves.size();
	this->info.runtime = t1 - t0;
	this->info.n_threads = 1;

	// Return
	return this->octree_leaves;
}

void tmcd::Octree::_run_octree_node(RecursiveOctreeNode& node_buffer, const int32_t depth, const std::vector<AABB>& aabbs)
{
	auto& buffer = node_buffer.buffer;
	const int n = (int)buffer.aabb_indices.size();

	// Early exit
	//// Impossible to find neighbors
	if (n == 0) {
		node_buffer.delete_children();
		return;
	}

	//// Max recursion
	if (depth > this->max_recursion) {
		std::cout << "TriangleMeshCollisionDetection error: max recursion reached in Octree mode. This is probably due to a bad collision mesh, especifically high concentration of very small elements or simply overlapping vertices, near-zero area triangles..." << std::endl;
		exit(-1);
		return;
	}

	// Cap criteria
	if (n <= this->recursion_cap) {
		node_buffer.delete_children();
		this->octree_leaves.push_back(&buffer);
	}
	else {
		// Classification
		node_buffer.populate_children();
		const std::array<float, 3> pivot = {
			0.5f * (buffer.bottom[0] + buffer.top[0]),
			0.5f * (buffer.bottom[1] + buffer.top[1]),
			0.5f * (buffer.bottom[2] + buffer.top[2]), };

		std::array<std::vector<int>*, 8> children_aabbs;
		for (int i = 0; i < 8; i++) {
			node_buffer.children[i]->buffer.bottom = buffer.bottom;
			node_buffer.children[i]->buffer.top = buffer.top;
			children_aabbs[i] = &node_buffer.children[i]->buffer.aabb_indices;
			children_aabbs[i]->clear();
		}

		/*
			Leaves order is imposed by libmorton to have a proper zsort:
				000, 100, 010, 110, 001, 101, 011, 111
		*/
		node_buffer.children[0]->buffer.top = pivot;

		node_buffer.children[4]->buffer.bottom[2] = pivot[2];
		node_buffer.children[4]->buffer.top[0] = pivot[0];
		node_buffer.children[4]->buffer.top[1] = pivot[1];

		node_buffer.children[2]->buffer.bottom[1] = pivot[1];
		node_buffer.children[2]->buffer.top[0] = pivot[0];
		node_buffer.children[2]->buffer.top[2] = pivot[2];

		node_buffer.children[6]->buffer.bottom[1] = pivot[1];
		node_buffer.children[6]->buffer.bottom[2] = pivot[2];
		node_buffer.children[6]->buffer.top[0] = pivot[0];

		node_buffer.children[1]->buffer.bottom[0] = pivot[0];
		node_buffer.children[1]->buffer.top[1] = pivot[1];
		node_buffer.children[1]->buffer.top[2] = pivot[2];

		node_buffer.children[5]->buffer.bottom[0] = pivot[0];
		node_buffer.children[5]->buffer.bottom[2] = pivot[2];
		node_buffer.children[5]->buffer.top[1] = pivot[1];

		node_buffer.children[3]->buffer.bottom[0] = pivot[0];
		node_buffer.children[3]->buffer.bottom[1] = pivot[1];
		node_buffer.children[3]->buffer.top[2] = pivot[2];

		node_buffer.children[7]->buffer.bottom = pivot;

		// Classify
		for (const int aabb_i : buffer.aabb_indices) {
			const AABB& aabb = aabbs[aabb_i];

			if (aabb.bottom[0] <= pivot[0]) {
				if (aabb.bottom[1] <= pivot[1]) {
					if (aabb.bottom[2] <= pivot[2]) {
						children_aabbs[0]->push_back(aabb_i); // (0, 0, 0)
					}
					if (aabb.top[2] >= pivot[2]) {
						children_aabbs[4]->push_back(aabb_i); // (0, 0, 1)
					}
				}
				if (aabb.top[1] >= pivot[1]) {
					if (aabb.bottom[2] <= pivot[2]) {
						children_aabbs[2]->push_back(aabb_i); // (0, 1, 0)
					}
					if (aabb.top[2] >= pivot[2]) {
						children_aabbs[6]->push_back(aabb_i); // (0, 1, 1)
					}
				}
			}
			if (aabb.top[0] >= pivot[0]) {
				if (aabb.bottom[1] <= pivot[1]) {
					if (aabb.bottom[2] <= pivot[2]) {
						children_aabbs[1]->push_back(aabb_i); // (1, 0, 0)
					}
					if (aabb.top[2] >= pivot[2]) {
						children_aabbs[5]->push_back(aabb_i); // (1, 0, 1)
					}
				}
				if (aabb.top[1] >= pivot[1]) {
					if (aabb.bottom[2] <= pivot[2]) {
						children_aabbs[3]->push_back(aabb_i); // (1, 1, 0)
					}
					if (aabb.top[2] >= pivot[2]) {
						children_aabbs[7]->push_back(aabb_i); // (1, 1, 1)
					}
				}
			}
		}

		// Spawn tasks for the children
		for (int child_i = 0; child_i < 8; child_i++) {
			this->_run_octree_node(*node_buffer.children[child_i], depth + 1, aabbs);
		}
	}
}

const std::vector<tmcd::OctreeNode*>& tmcd::Octree::get_leaves() const
{
	return this->octree_leaves;
}

tmcd::info::Octree tmcd::Octree::get_info() const
{
	int32_t sum = 0;
	for (const OctreeNode* leaf : this->octree_leaves) {
		sum += (int32_t)leaf->aabb_indices.size();
	}
	info::Octree info = this->info;
	info.avg_n_aabbs_per_leaf = sum / info.n_leaves;
	return info;
}
