#pragma once
#include <vector>
#include <functional>

namespace stark
{
	struct Callbacks
	{
		std::vector<std::function<void()>> before_time_step;
		std::vector<std::function<void()>> before_energy_evaluation;
		std::vector<std::function<void()>> after_energy_evaluation;
		std::vector<std::function<void()>> after_time_step;
		std::vector<std::function<double()>> max_allowed_step;
		std::vector<std::function<bool()>> is_state_valid;
		std::vector<std::function<void()>> write_frame;
	};
}