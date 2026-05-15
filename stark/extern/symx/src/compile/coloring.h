#pragma once

#include <vector>
#include <cstdint>

namespace symx
{
	void compute_color_graph(
		std::vector<std::vector<int32_t>>& color_bins,
		const int32_t* connectivity_data,
		const int32_t connectivity_n_elements,
		const int32_t connectivity_stride,
		const std::vector<int32_t>& coloring_use_indices
	);
}
