#pragma once
#include <vector>
#include <array>

#include <Eigen/Dense>
#include <symx>


namespace stark::models
{
	struct BaseRigidBodyConstraints
	{
		/* ===========================================   DEFINITIONS  =========================================== */
		/*
			Terms:
				- absolute: with respect to the global frame of reference
				- local/relative: with respect to the local frame of reference of an object
				- direction: general direction withuot a starting point
				- axes: point and direction
		*/

		/*
		*	Disables absolute displacement of a point in a an object.
		*/
		struct AnchorPoints
		{
			symx::LabelledConnectivity<2> conn{ { "idx", "rb" } };
			std::vector<Eigen::Vector3d> loc;
			std::vector<Eigen::Vector3d> target_glob;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Disables absolute displacement of a point in a an object.
		*/
		struct BallJoints
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> a_loc;
			std::vector<Eigen::Vector3d> b_loc;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Enforces two local axes in two objects to be aligned.
		*/
		struct RelativeDirectionLocks
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<Eigen::Vector3d> db_loc;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Enforces a point on a object to move along a relative direction of another object.
		*/
		struct PointOnAxisConstraints
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> a_loc;
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<Eigen::Vector3d> b_loc;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Adds forces counteracting relative displacement and velocity of two local points from different objects to not be at a reference distance.
		*/
		struct DampedSprings
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> a_loc;
			std::vector<Eigen::Vector3d> b_loc;
			std::vector<double> rest_length;
			std::vector<double> damping;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Limits the min and max distance of two points from two objects.
		*/
		struct DistanceLimits
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> a_loc;
			std::vector<Eigen::Vector3d> b_loc;
			std::vector<double> min_length;
			std::vector<double> max_length;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Enforces a local direction in an object to be within the cone of admissible directions relative to another object.
		*/
		struct AngleLimits
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<Eigen::Vector3d> db_loc;
			std::vector<double> admissible_dot;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Add torque counteracting the difference in angular velocity between two objects.
		*/
		struct RelativeLinearVelocityMotors
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<double> target_v;
			std::vector<double> max_force;
			std::vector<double> delay;
			std::vector<double> is_active;
		};

		/*
		*	Add torque counteracting the difference in angular velocity between two objects.
		*/
		struct RelativeAngularVelocityMotors
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<double> target_w;
			std::vector<double> max_torque;
			std::vector<double> delay;
			std::vector<double> is_active;
		};


		/* ===========================================   FIELDS   =========================================== */
		AnchorPoints anchor_points;
		BallJoints ball_joints;
		RelativeDirectionLocks relative_direction_locks;
		PointOnAxisConstraints point_on_axis_constraints;
		DampedSprings damped_springs;
		DistanceLimits distance_limits;
		AngleLimits angle_limits;
		RelativeLinearVelocityMotors relative_linear_velocity_motors;
		RelativeAngularVelocityMotors relative_angular_velocity_motors;
	};
}
