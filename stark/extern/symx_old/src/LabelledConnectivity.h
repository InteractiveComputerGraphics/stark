#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <string>


namespace symx
{
	template<std::size_t N>
	struct LabelledConnectivity
	{
		std::array<std::string, N> labels;
		std::vector<std::array<int32_t, N>> data;

		LabelledConnectivity(const std::array<std::string, N>& labels) : labels(labels) {};
		std::array<int32_t, N>& operator[](const int idx) { return this->data[idx]; };
		const std::array<int32_t, N>& operator[](const int idx) const { return this->data[idx]; };
		void push_back(const std::array<int32_t, N>& v) { this->data.push_back(v); };
		void clear() { this->data.clear(); };
		std::size_t size() const { return this->data.size(); };
		int32_t get_label_idx(const std::string& label) const 
		{
			for (size_t i = 0; i < N; i++) {
				if (this->labels[i] == label) {
					return (int32_t)i;
				}
			}
			
			// Error
			std::cout << "stark error: LabelledConnectivity.get_label_idx() Label not found" << std::endl;
			exit(-1);
		};
		std::array<int32_t, N> get_label_indices(const std::array<std::string, N>& labels) const
		{
			std::array<int32_t, N> indices;
			for (size_t i = 0; i < N; i++) {
				indices[i] = this->get_label_idx(labels[i]);
			}
			return indices;
		};
		template<std::size_t M>
		std::array<int32_t, M> get(const int idx, const std::array<int32_t, M>& entries) const
		{
			std::array<int32_t, M> v;
			for (size_t i = 0; i < M; i++) {
				v[i] = this->data[idx][entries[i]];
			}
			return v;
		};
		int32_t numbered_push_back(const std::array<int32_t, N-1>& v)
		{ 
			std::array<int32_t, N> v_;
			v_[0] = (int32_t)this->data.size();
			for (size_t i = 0; i < N-1; i++) {
				v_[i + 1] = v[i];
			}
			this->push_back(v_);
			return v_[0];
		};
	};
}