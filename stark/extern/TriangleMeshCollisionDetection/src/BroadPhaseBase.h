#pragma once
#include <vector>
#include <array>

#include "types.h"
#include "Octree.h"
#include "Meshes.h"
#include "AABBs.h"


namespace tmcd
{
	namespace internals
	{
		/*
			Common functionalities for all types of BroadPhase specializations.
		*/
		class BroadPhaseBase
		{
		public:
			/* Methods */
			BroadPhaseBase() = default;
			~BroadPhaseBase() = default;

			void set_n_threads(const int32_t n_threads);
			int32_t get_n_threads() const;
			void set_max_recursion(const int32_t cap = 20);
			void set_recursion_cap(const int32_t cap = 1500);
			int32_t get_n_meshes() const;

		protected:

			/* Methods */
			void _init_threads();

			/* Fields */
			internals::Meshes meshes;
			internals::AABBs aabbs;
			Octree octree;

			// Multithreading
			std::vector<internals::ThreadBuffer> thread_buffers;
			int32_t n_threads = -1;
		};
	}
}