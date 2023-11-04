#pragma once
#include <memory>

#include "../../solver/Stark.h"
#include "interval_types.h"


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
		symx::DoF dof;

		/* Methods */
		PointDynamics(Stark& sim);
	};
	using spPointDynamics = std::shared_ptr<PointDynamics>;
}
