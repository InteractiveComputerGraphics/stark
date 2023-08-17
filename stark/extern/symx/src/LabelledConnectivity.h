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
	};
}