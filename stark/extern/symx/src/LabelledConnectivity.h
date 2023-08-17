#pragma once
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
		void push_back(const std::array<int32_t, N>& v) { this->data.push_back(v); };
		void clear() { this->data.clear(); };
		std::size_t size() const { return this->data.size(); };
		void numbered_push_back(const std::array<int32_t, N-1>& v)
		{ 
			std::array<int32_t, N> v_;
			v_[0] = (int32_t)this->data.size();
			for (size_t i = 0; i < N-1; i++) {
				v_[i + 1] = v[0];
			}
			this->push_back(v_);
		};
	};
}