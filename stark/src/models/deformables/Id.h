#pragma once
#include <unordered_map>
#include <string>
#include <iostream>


namespace stark::models
{
	class Id
	{
	public:
		int idx;
		std::unordered_map<std::string, int> local_indices;  // e.g. {"EnergyTriangleStrain": 2, "EnergyFrictionalContact": 6}

		Id(const int global_idx);
		void set_local_idx(const std::string label, const int local_idx);
		int get_global_idx() const;
		int get_local_idx(const std::string label) const;
	};
}