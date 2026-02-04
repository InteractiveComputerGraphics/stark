#pragma once
#include <vector>

namespace bsm
{
	template<typename VECTOR>
	class ParallelVector
	{
	public:
		/* Fields */
		std::vector<VECTOR> vectors;

		/* Methods */
		ParallelVector() {};
		~ParallelVector() {};
		void reset(const int n_threads, const int size);
		VECTOR& end();
		VECTOR& get_solution();
		VECTOR& get(const int thread_id);
	};

	template<typename T>
	inline void ParallelVector<T>::reset(const int n_threads, const int size)
	{
		this->vectors.resize(n_threads);
		for (auto& vector : this->vectors) {
			vector.resize(size);
			for (int i = 0; i < size; i++) {
				vector[i] = 0.0;
			}
		}
	}
	template<typename VECTOR>
	inline VECTOR& ParallelVector<VECTOR>::end()
	{
		const int n = (int)this->vectors[0].size();
		const int n_vectors = (int)this->vectors.size();
		#pragma omp parallel for schedule(static) num_threads(n_vectors)
		for (int i = 0; i < n; i++) {
			for (int j = 1; j < n_vectors; j++) {
				this->vectors[0][i] += this->vectors[j][i];
			}
		}
		return this->vectors[0];
	}
	template<typename VECTOR>
	inline VECTOR& ParallelVector<VECTOR>::get_solution()
	{
		return this->vectors[0];
	}
	template<typename VECTOR>
	inline VECTOR& ParallelVector<VECTOR>::get(const int thread_id)
	{
		return this->vectors[thread_id];
	}
}
