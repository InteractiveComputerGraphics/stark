#pragma once
#include <memory>
#include <cassert>

#include "../../core/Stark.h"
#include "../IntervalVector.h"
#include "Id.h"


namespace stark::models
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

		/* Methods */
		PointDynamics(stark::core::Stark& stark);
		Id add(const std::vector<Eigen::Vector3d>& x, const std::vector<Eigen::Vector3d>& v = std::vector<Eigen::Vector3d>());
		int get_begin(const Id& id) const;
		int size(const Id& id) const;
		int size() const;

	private:
		void _before_time_step(stark::core::Stark& stark);
		void _on_time_step_accepted(stark::core::Stark& stark);
	};
	using spPointDynamics = std::shared_ptr<PointDynamics>;
}
