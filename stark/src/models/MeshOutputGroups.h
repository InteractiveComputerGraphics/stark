#pragma once
#include <vector>
#include <array>
#include <unordered_set>
#include <unordered_map>
#include <string>

#include <Eigen/Dense>


namespace stark
{
	class MeshOutputGroups
	{
	public:
		/* Fields */
		std::unordered_map<std::string, std::unordered_set<int>> groups;

		/* Methods */
		void add_to_group(const std::string label, const int id);
		void add_to_group(const std::string label, const std::vector<int>& ids);
		int size() const;
	};
}
