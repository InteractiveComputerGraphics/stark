#pragma once
#include <vector>
#include <limits>

#include <Eigen/Dense>

namespace stark::models
{
	class PrescribedPointGroupWithTransformation
	{
	public:
		PrescribedPointGroupWithTransformation(const int obj, const std::string label);

		void set_stiffness(const double stiffness);
		void set_time_bounds(const double begin, const double end);
		void set_linear_velocity(const Eigen::Vector3d& v);
		void set_angular_velocity(const Eigen::Vector3d& w, const Eigen::Vector3d& rotation_center);
		void clear();
		void add(const int loc_idx);
		int size() const;
		int get_idx(const int i) const;

		Eigen::Vector3d get_transformed(const Eigen::Vector3d& rest_position, const double sim_time);

	private:
		double stiffness = 1e6;
		std::vector<int> loc_indices;
		const int obj_idx;

		// Transformation
		double t_begin = 0;
		double t_end = std::numeric_limits<double>::max();
		Eigen::Vector3d linear_velocity;
		Eigen::Vector3d angular_velocity;
		Eigen::Vector3d rotation_center;
	};
}
