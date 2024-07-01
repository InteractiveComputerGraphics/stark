#pragma once
#include <vector>

namespace bsm
{
	template<typename FLOAT>
	class ParallelNumber
	{
		const static int PADDING_BYTES = 64;
	public:
		/* Fields */
		std::vector<FLOAT> values; // Pointers to avoid false sharing
		int padding = -1;

		/* Methods */
		void reset(const int n_threads);
		FLOAT& end();
		FLOAT& get(const int thread_id);
		FLOAT& get_solution();
	};

	// ============================================================================
	template<typename FLOAT>
	inline void ParallelNumber<FLOAT>::reset(const int n_threads)
	{
		this->padding = PADDING_BYTES / sizeof(FLOAT);
		this->values.resize(n_threads*this->padding);
		for (int i = 0; i < n_threads; i++) {
			this->values[i * this->padding] = 0.0;
		}
	}
	template<typename FLOAT>
	inline FLOAT& ParallelNumber<FLOAT>::get(const int thread_id)
	{
		return this->values[thread_id*this->padding];
	}
	template<typename FLOAT>
	inline FLOAT& ParallelNumber<FLOAT>::end()
	{
		const int n = (int)this->values.size()/this->padding;
		for (int i = 1; i < n; i++) {
			this->values[0] += this->values[i * this->padding];
		}
		return this->values[0];
	}
	template<typename FLOAT>
	inline FLOAT& ParallelNumber<FLOAT>::get_solution()
	{
		return this->values[0];
	}
}
