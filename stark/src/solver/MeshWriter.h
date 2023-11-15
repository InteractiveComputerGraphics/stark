#pragma once
#include <vector>
#include <array>
#include <string>
#include <unordered_map>
#include <function>

#include <Eigen/Dense>


namespace stark
{
	class MeshWriter
	{
	public:
		/* Definitions */
		enum class Format { VTK };

		/* Fields */
		std::unordered_map<std::string, std::vector<int>> labeled_groups;


		/* Methods */
		int add(
			const std::string label, 
			std::function<std::tuple<Eigen::Vector3d*, Eigen::Vector3d*>()> point_range, 
			std::function<std::tuple<std::array<int, 3>*, std::array<int, 3>*>()> triangle_range,
			const int offset);

		void add_group(const std::string label, const int body_id);
		void add_group(const std::string label, const std::vector<int>& rb_indices);
		void write()

	};
}