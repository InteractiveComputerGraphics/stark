#pragma once
#include <Eigen/Dense>

#include "PIDController.h"

namespace stark::models
{
	struct MotorController
	{
		int rb_a_idx = -1;
		int rb_b_idx = -1;
		Eigen::Vector3d loc_da;
		double max_torque = -1.0;
		double target_w = -1.0;
		PIDController pid;
	};
}