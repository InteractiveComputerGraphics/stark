#pragma once
#include <Eigen/Dense>

namespace stark
{
	// Forward declaration
	class PointDynamics;

	/**
	* @brief Handler to a set of points representing deformable material stored in PointDynamics.
	*/
	class PointSetHandler
	{
	private:
		int idx = -1;
		PointDynamics* dyn = nullptr;

	public:
		PointSetHandler(PointDynamics* dyn, int idx);
		int get_idx() const;
		bool is_valid() const;
		void exit_if_not_valid(const std::string& where_) const;
		std::string get_label() const;
		PointSetHandler& set_label(const std::string& label);
		int get_begin() const;
		int get_end() const;
		int size() const;
		int get_global_index(int local_index) const;
		template<std::size_t N>
		std::array<int, N> get_global_indices(const std::array<int, N>& local_indices) const;
		std::vector<int> all() const;

		Eigen::Vector3d get_position(int local_index) const;
		Eigen::Vector3d get_rest_position(int local_index) const;
		Eigen::Vector3d get_velocity(int local_index) const;
		Eigen::Vector3d get_acceleration(int local_index) const;
		Eigen::Vector3d get_force(int local_index) const;
		
		void set_position(int local_index, const Eigen::Vector3d& v);
		void set_rest_position(int local_index, const Eigen::Vector3d& v);
		void set_velocity(int local_index, const Eigen::Vector3d& v);
		void set_acceleration(int local_index, const Eigen::Vector3d& v);
		void set_force(int local_index, const Eigen::Vector3d& v);

		PointSetHandler& add_displacement(const Eigen::Vector3d& displacement, bool also_at_rest_pose = true);
		PointSetHandler& add_rotation(const double angle_deg, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot = { 0.0, 0.0, 0.0 }, bool also_at_rest_pose = true);
	};

	// Inline methods
    template<std::size_t N>
    inline std::array<int, N> PointSetHandler::get_global_indices(const std::array<int, N>& local_indices) const
    {
        // Lambda with templated argument to avoid "member access into incomplete type 'PointDynamics'" error
        // Note: Error only reported on clang (tested on: Apple clang version 15.0.0), classified "program is ill-formed, no diagnostic required"
        return [&](auto dyn) { return dyn->get_global_indices(this->get_idx(), local_indices); }(this->dyn);
    }
}
