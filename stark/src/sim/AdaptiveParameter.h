#pragma once

namespace stark
{
	class AdaptiveParameter
	{
	public:
		double value = 0.0;
		double min = 0.0;
		double max = 0.0;

		double success_multiplier = 1.2;
		double failure_multiplier = 0.5;
		int n_successful_iterations_to_increase = 5;

		void successful_iteration();
		void failed_iteration();
		
	private:
		int current_successful_iterations = 0;
	};
}