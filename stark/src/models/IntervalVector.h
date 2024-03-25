#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <cassert>


namespace stark
{
	/**
	*	std::vector<T> with offsets to identify contiguous intervals as collections (sets).
	* 
	*	Note: It is intended that intervals cannot be removed to avoid messing with the shifting indices.
	*/
	template<typename T>
	class IntervalVector
	{
	public:
		/* Fields */
		std::vector<T> data;
		std::vector<int> offsets;

		/* Methods */
		// Setters
		void clear();
		int append(const std::vector<T>& input);
		int append(const int n, const T& v);
		int append_empty();
		void update(const int set_id, const std::vector<T>& input);

		template<typename T_It>
		int append(const T_It begin, const T_It end);
		template<typename T_It>
		void update(const int set_id, const T_It begin, const T_It end);
		void reserve(const int n);

		// Getters
		int size() const;
		int get_n_sets() const;
		int get_set_size(const int set_id) const;
		int get_begin(const int set_id) const;
		int get_end(const int set_id) const;
		std::array<int, 2> get_range(const int set_id) const;
		int get_global_index(const int set_id, const int local_idx) const;
		int get_set_bounding_index(const int global_idx) const;
		int get_set_bounding_index(const int set_id, const int local_idx) const;
		template<std::size_t N>
		std::array<int, N> get_global_indices(const int set_id, const std::array<int, N>& local_indices) const;
		IntervalVector<T> extract(const std::vector<int>& sets) const;
		std::vector<T> get_set(const int set_id) const;

		// Indexing
		T* get_begin_ptr(const int set_id);
		T* get_end_ptr(const int set_id);
		const T* get_begin_ptr(const int set_id) const;
		const T* get_end_ptr(const int set_id) const;
		T& operator[](const int global_idx);
		T& get(const int global_idx);
		T& get(const int set_id, const int local_idx);
		const T& operator[](const int global_idx) const;
		const T& get(const int global_idx) const;
		const T& get(const int set_id, const int local_idx) const;

	private:
		void _assert_existing_set(const int set_id) const;
		void _assert_local_idx(const int set_id, const int local_idx) const;
		void _assert_global_idx(const int global_idx) const;
	};





	// DEFINITIONS ===========================================================================
	template<typename T>
	inline int IntervalVector<T>::append(const std::vector<T>& input)
	{
		return this->append(input.begin(), input.end());
	}
	template<typename T>
	inline int IntervalVector<T>::append(const int n, const T& v)
	{
		const int set_id = this->get_n_sets();
		for (int i = 0; i < n; i++) {
			this->data.push_back(v);
		}
		if (set_id == 0) { this->offsets.push_back(0); }
		this->offsets.push_back((int)this->data.size());
		return set_id;
	}
	template<typename T>
	template<typename T_It>
	inline int IntervalVector<T>::append(const T_It begin, const T_It end)
	{
		const int n = (int)std::distance(begin, end);
		const int set_id = this->get_n_sets();
		this->data.insert(this->data.end(), begin, end);
		if (set_id == 0) { this->offsets.push_back(0); }
		this->offsets.push_back((int)this->data.size());
		return set_id;
	}
	template<typename T>
	inline int IntervalVector<T>::append_empty()
	{
		const int set_id = this->get_n_sets();
		if (set_id == 0) { this->offsets.push_back(0); }
		this->offsets.push_back((int)this->data.size());
		return set_id;
	}
	template<typename T>
	template<std::size_t N>
	inline std::array<int, N> IntervalVector<T>::get_global_indices(const int set_id, const std::array<int, N>& local_indices) const
	{
		std::array<int, N> glob;

		for (std::size_t i = 0; i < N; i++) {
			glob[i] = this->get_global_index(set_id, local_indices[i]);
		}

		return glob;
	}
	template<typename T>
	inline void IntervalVector<T>::update(const int set_id, const std::vector<T>& input)
	{
		this->_assert_existing_set(set_id);
		this->update(set_id, input.begin(), input.end());
	}
	template<typename T>
	template<typename T_It>
	inline void IntervalVector<T>::update(const int set_id, const T_It begin, const T_It end)
	{
		this->_assert_existing_set(set_id);
		const int n_diff = (int)std::distance(begin, end) - this->get_set_size(set_id);
		if (n_diff != 0) {
			std::cout << "IntervalVector error: can't .update() with a different size than the original one." << std::endl;
			exit(-1);
		}
		std::copy(begin, end, this->get_begin_ptr(set_id));
	}
	template<typename T>
	inline void IntervalVector<T>::clear()
	{
		this->data.clear();
		this->offsets.clear();
	}



	template<typename T>
	inline void IntervalVector<T>::reserve(const int n)
	{
		this->data.reserve(n);
	}

	template<typename T>
	inline int IntervalVector<T>::size() const
	{
		return (int)this->data.size();
	}
	template<typename T>
	inline int IntervalVector<T>::get_n_sets() const
	{
		if (this->offsets.size() == 0) {
			return 0;
		}
		else {
			return (int)this->offsets.size() - 1;
		}
	}
	template<typename T>
	inline int IntervalVector<T>::get_set_size(const int set_id) const
	{
		this->_assert_existing_set(set_id);
		return this->offsets[set_id + 1] - this->offsets[set_id];
	}
	template<typename T>
	inline int IntervalVector<T>::get_begin(const int set_id) const
	{
		this->_assert_existing_set(set_id);
		return this->offsets[set_id];
	}
	template<typename T>
	inline int IntervalVector<T>::get_end(const int set_id) const
	{
		this->_assert_existing_set(set_id);
		return this->offsets[set_id + 1];
	}
	template<typename T>
	inline std::array<int, 2> IntervalVector<T>::get_range(const int set_id) const
	{
		return {this->get_begin(set_id), this->get_end(set_id)};
	}
	template<typename T>
	inline int IntervalVector<T>::get_global_index(const int set_id, const int local_idx) const
	{
		this->_assert_existing_set(set_id);
		this->_assert_local_idx(set_id, local_idx);
		return this->offsets[set_id] + local_idx;
	}
	template<typename T>
	inline int IntervalVector<T>::get_set_bounding_index(const int global_idx) const
	{
		this->_assert_global_idx(global_idx);
		for (int set_i = 0; set_i < (int)this->offsets.size(); set_i++) {
			if (global_idx >= this->offsets[set_i]) {
				return global_idx - 1;
			}
		}
	}
	template<typename T>
	inline int IntervalVector<T>::get_set_bounding_index(const int set_id, const int local_idx) const
	{
		this->_assert_existing_set(set_id);
		this->_assert_local_idx(set_id, local_idx);
		return this->get_set_bounding_index(this->get_global_index(set_id, local_idx));
	}
	template<typename T>
	inline IntervalVector<T> IntervalVector<T>::extract(const std::vector<int>& sets) const
	{
		IntervalVector output;

		int n = 0;
		for (const int set : sets) {
			n += this->get_set_size(set);
		}
		output.reserve(n);

		for (const int set : sets) {
			output.append(this->get_begin_ptr(set), this->get_end_ptr(set));
		}

		return output;
	}

	template<typename T>
	inline std::vector<T> IntervalVector<T>::get_set(const int set_id) const
	{
		this->_assert_existing_set(set_id);
		return std::vector<T>(this->get_begin_ptr(set_id), this->get_end_ptr(set_id));
	}

	template<typename T>
	inline T* IntervalVector<T>::get_begin_ptr(const int set_id)
	{
		return this->data.data() + this->get_begin(set_id);
	}
	template<typename T>
	inline T* IntervalVector<T>::get_end_ptr(const int set_id)
	{
		return this->data.data() + this->get_end(set_id);
	}
	template<typename T>
	inline const T* IntervalVector<T>::get_begin_ptr(const int set_id) const
	{
		return this->data.data() + this->get_begin(set_id);
	}
	template<typename T>
	inline const T* IntervalVector<T>::get_end_ptr(const int set_id) const
	{
		return this->data.data() + this->get_end(set_id);
	}
	template<typename T>
	inline T& IntervalVector<T>::operator[](const int global_idx)
	{
		this->_assert_global_idx(global_idx);
		return this->get(global_idx);
	}
	template<typename T>
	inline T& IntervalVector<T>::get(const int global_idx)
	{
		this->_assert_global_idx(global_idx);
		return this->data[global_idx];
	}
	template<typename T>
	inline T& IntervalVector<T>::get(const int set_id, const int local_idx)
	{
		this->_assert_existing_set(set_id);
		this->_assert_local_idx(set_id, local_idx);
		return this->get(this->get_global_index(set_id, local_idx));
	}
	template<typename T>
	inline const T& IntervalVector<T>::operator[](const int global_idx) const
	{
		this->_assert_global_idx(global_idx);
		return this->get(global_idx);
	}
	template<typename T>
	inline const T& IntervalVector<T>::get(const int global_idx) const
	{
		this->_assert_global_idx(global_idx);
		return this->data[global_idx];
	}
	template<typename T>
	inline const T& IntervalVector<T>::get(const int set_id, const int local_idx) const
	{
		this->_assert_existing_set(set_id);
		this->_assert_local_idx(set_id, local_idx);
		return this->get(this->get_global_index(set_id, local_idx));
	}



	template<typename T>
	inline void IntervalVector<T>::_assert_existing_set(const int set_id) const
	{
		assert(set_id >= 0 && set_id < this->get_n_sets() && "IntervalVector set_id out of range");
	}
	template<typename T>
	inline void IntervalVector<T>::_assert_local_idx(const int set_id, const int local_idx) const
	{
		assert(local_idx >= 0 && local_idx < this->get_set_size(set_id) && "IntervalVector local_idx out of range");
	}
	template<typename T>
	inline void IntervalVector<T>::_assert_global_idx(const int global_idx) const
	{
		assert(global_idx >= 0 && global_idx < this->offsets.back() && "IntervalVector global_idx out of range");
	}
}
