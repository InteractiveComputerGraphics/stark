#pragma once
#include <vector>
#include <limits>

#include <Eigen/Dense>

namespace stark::models
{
	/*
		High level point position prescriptions.

		Gives the user access to a collection of points, that they can edit and,
		at runtime, a rigid transformation is used to calculate the updated target
		position for the points in the collection.

		See `PrescribedPointGroup` for a lower level variation where per-point control
		is granted.
	*/
	class PrescribedPointGroupWithTransformation
	{
	public:
		PrescribedPointGroupWithTransformation(const int obj_idx, const std::string label);

		void clear();
		void set_stiffness(const double stiffness);
		void set_time_bounds(const double begin, const double end);
		void set_linear_velocity(const Eigen::Vector3d& v);
		void set_angular_velocity(const Eigen::Vector3d& w, const Eigen::Vector3d& rotation_center);
		void add(const int loc_idx);
		void add_from_range(const int loc_idx_begin, const int loc_idx_end);
		template<typename It>
		void add_from_aabb(It begin, It end, const Eigen::Vector3d& center, const Eigen::Vector3d& size);
		Eigen::Vector3d get_transformed(const Eigen::Vector3d& rest_position, const double sim_time);

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

		// Transformation
		double t_begin = 0;
		double t_end = std::numeric_limits<double>::max();
		Eigen::Vector3d linear_velocity;
		Eigen::Vector3d angular_velocity;
		Eigen::Vector3d rotation_center;
	};


	template<typename It>
	inline void PrescribedPointGroupWithTransformation::add_from_aabb(It begin, It end, const Eigen::Vector3d& center, const Eigen::Vector3d& size)
	{
		const Eigen::AlignedBox3d aabb = Eigen::AlignedBox3d(center - 0.5*size, center + 0.5*size);
		const int n = (int)std::distance(begin, end);
		for (int i = 0; i < n; i++) {
			if (aabb.contains(begin[i])) {
				this->add(i);
			}
		}
	}
}
