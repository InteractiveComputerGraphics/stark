#pragma once
#include <memory>
#include <cassert>

#include "../../core/Stark.h"
#include "../IntervalVector.h"
#include "PointSetHandler.h"


namespace stark
{
	class PointDynamics
	{
	public:
		/* Fields */
		IntervalVector<Eigen::Vector3d> X;   // Node positions at rest
		IntervalVector<Eigen::Vector3d> x0;  // Node positions at time n
		IntervalVector<Eigen::Vector3d> x1;  // Node positions at time n+1
		IntervalVector<Eigen::Vector3d> v0;  // Node velocities at time n
		IntervalVector<Eigen::Vector3d> v1;  // Node velocities at time n+1
		IntervalVector<Eigen::Vector3d> a;   // Node accelerations
		IntervalVector<Eigen::Vector3d> f;   // Node forces
		symx::DoF dof;
		std::vector<std::string> labels;

		/* Methods */
		PointDynamics(core::Stark& stark);
		PointSetHandler add(const std::vector<Eigen::Vector3d>& x, const std::string& label = "");
		int size() const;
		int get_set_size(int idx_in_ps) const;
		int get_begin(int idx_in_ps) const;
		int get_end(int idx_in_ps) const;
		int get_global_index(int idx_in_ps, int local_index) const;
		template<std::size_t N>
		std::array<int, N> get_global_indices(int idx_in_ps, const std::array<int, N>& local_indices) const;

		Eigen::Vector3d get_x1(int global_index, double dt) const;
		template<std::size_t N>
		std::array<Eigen::Vector3d, N> get_x1(const std::array<int, N>& global_indices, double dt) const;
		template<std::size_t N>
		std::array<Eigen::Vector3d, N> get_X(const std::array<int, N>& global_indices) const;

	private:
		void _before_time_step(core::Stark& stark);
		void _on_time_step_accepted(core::Stark& stark);
	};
	using spPointDynamics = std::shared_ptr<PointDynamics>;

	// Inline methods ===================================================================================================
	template<std::size_t N>
	inline std::array<int, N> PointDynamics::get_global_indices(int idx_in_ps, const std::array<int, N>& local_indices) const
	{
		return this->X.get_global_indices(idx_in_ps, local_indices);
	}
	template<std::size_t N>
	inline std::array<Eigen::Vector3d, N> PointDynamics::get_x1(const std::array<int, N>& global_indices, double dt) const
	{
		std::array<Eigen::Vector3d, N> result;
		for (int i = 0; i < N; i++) {
			result[i] = this->get_x1(global_indices[i], dt);
		}
		return result;
	}
	template<std::size_t N>
	inline std::array<Eigen::Vector3d, N> PointDynamics::get_X(const std::array<int, N>& global_indices) const
	{
		std::array<Eigen::Vector3d, N> result;
		for (int i = 0; i < N; i++) {
			result[i] = this->X[global_indices[i]];
		}
		return result;
	}
}
