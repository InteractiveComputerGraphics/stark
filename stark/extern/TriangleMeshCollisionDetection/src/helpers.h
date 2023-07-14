#pragma once
#include <vector>
#include <cstring>
#include <omp.h>

#include "types.h"


namespace tmcd
{
	namespace internals
	{
		template<typename T>
		inline void parallel_concat(std::vector<T>& out, std::vector<std::vector<T>*>& in)
		{
			const int n_threads = (int)in.size();
			std::vector<int> prefix_sum(n_threads + 1);
			prefix_sum[0] = 0;
			for (int thread_id = 0; thread_id < n_threads; thread_id++) {
				prefix_sum[thread_id + 1] = prefix_sum[thread_id] + (int)in[thread_id]->size();
			}
			out.resize(prefix_sum.back());

			#pragma omp parallel num_threads(n_threads)
			{
				const int thread_id = omp_get_thread_num();
				const int begin = prefix_sum[thread_id];
				const std::vector<T>& loc = *in[thread_id];
				const int size = (int)loc.size();
				if (size > 0) {
					memcpy(&out[begin], &loc[0], sizeof(T) * size);
				}
			}
		};
		inline bool overlap(const AABB& a, const AABB& b)
		{
			bool overlap = true;
			for (int i = 0; i < 3; i++) {
				if ((a.bottom[i] > b.top[i]) || (b.bottom[i] > a.top[i])) {
					overlap = false;
				}
			}
			return overlap;
		};
	}
}


