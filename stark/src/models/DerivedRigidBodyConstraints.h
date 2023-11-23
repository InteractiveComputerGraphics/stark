#pragma once
#include <vector>
#include <array>


namespace stark::models
{
	struct DerivedRigidBodyConstraints
	{
		/* ===========================================   DEFINITIONS  =========================================== */

		/*
		*	Forces a translation and rotation to an object
		*/
		struct PrescribedTransforms
		{
			std::vector<std::array<int, 4>> anchor_points_idx;
		};

		/*
		*	Disables relative displacement but allows relative rotation around a local axis.
		*	Could also be named "AxisLock"
		*/
		struct HingeJoints
		{
			std::vector<int> ball_joint_idx;
			std::vector<int> relative_direction_lock_idx;
		};

		/*
		*	Disables relative displacement or rotation between two objects.
		*/
		struct WeldJoints
		{
			std::vector<std::array<int, 3>> ball_joints_idx;
		};

		/*
		*	Enforces two objects to move along a relative axis wrt each other enabling only relative rotations around such axis.
		*	Optionally, perpendicular rotations can be also disabled.
		*/
		struct Sliders
		{
			std::vector<int> point_on_direction_idx;
			std::vector<int> longitudinal_relative_direction_lock_idx;
			std::vector<int> perpendicular_relative_direction_lock_idx;  // -1 if allowed to rotate
		};

		/*
		*	Restricts the motion of two objects with a slider and adds a damped spring with limits.
		*/
		struct SpringSliders
		{
			std::vector<int> slider_idx;
			std::vector<int> spring_idx;
		};
		

		/* ===========================================   FIELDS   =========================================== */
		PrescribedTransforms prescribed_transforms;
		HingeJoints hinge_joints;
		WeldJoints weld_joints;
		Sliders sliders;
		SpringSliders spring_sliders;
	};
}
