#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace stark::models
{
	/*
		Low level point position prescriptions.

		Gives the user access to a collection of points and, during the simulation,
		they can change the target position of each of them or even modify the list of
		points themselves.

		See `PrescribedPointGroupWithTransformation` for a higher level variation where
		rigid transformations can be applied to the set of points.
	*/
	class PrescribedPointGroup
	{
	public:
		PrescribedPointGroup(const int obj_idx, const std::string label);

		void clear();
		void set_stiffness(const double stiffness);
		void add(const int loc_idx, const Eigen::Vector3d& target_position);
		void set_target_position(const int i, const Eigen::Vector3d& target_position);
		const Eigen::Vector3d& get_target_position(const int i) const;

		int size() const;
		int get_point_idx(const int i) const;
		double get_stiffness() const;
		std::string get_label() const;
		int get_obj_idx() const;

	private:
		double stiffness = 1e6;
		const int obj_idx;
		std::string label;
		std::vector<int> loc_indices;
		std::vector<Eigen::Vector3d> target_positions;
	};
}
