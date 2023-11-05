#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace stark::models
{
	class PrescribedPointGroup
	{
	public:
		PrescribedPointGroup(const int obj, const std::string label);

		void set_stiffness(const double stiffness);
		void clear();
		void add(const int loc_idx, const Eigen::Vector3d& target_position);
		void set_target_position(const int i, const Eigen::Vector3d& target_position);
		int size() const;
		int get_idx(const int i) const;
		double get_stiffness() const;
		std::string get_label() const;
		int get_obj_idx() const;
		const Eigen::Vector3d& get_target_position(const int i) const;

	private:
		double stiffness = 1e6;
		std::vector<int> loc_indices;
		std::vector<Eigen::Vector3d> target_positions;
		const int obj_idx;
		std::string label;
	};
}
