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
			std::vector<Eigen::Vector3d> target;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Disables absolute displacement of a point in a an object.
		*/
		struct BallJoints
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> loc_a;
			std::vector<Eigen::Vector3d> loc_b;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Enforces two local axes in two objects to be aligned.
		*/
		struct RelativeDirectionLocks
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> loc_da;
			std::vector<Eigen::Vector3d> loc_db;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Enforces a point on a object to move along a relative direction of another object.
		*/
		struct PointOnAxisLocks
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> loc_a;
			std::vector<Eigen::Vector3d> loc_da;
			std::vector<Eigen::Vector3d> loc_b;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Enforces a local direction in an object to be within the cone of admisible directions relative to another object.
		*/
		struct ConeDirectionLocks
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> loc_da;
			std::vector<double> admisible_dot;
			std::vector<Eigen::Vector3d> loc_db;
			std::vector<double> stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Adds forces counteracting relative displacement of two local points from different objects to not be at a reference distance.
		*	Damping and displacement limits are supported.
		*/
		struct DampedSprings
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> loc_a;
			std::vector<Eigen::Vector3d> loc_b;
			std::vector<double> rest_length;
			std::vector<double> stiffness;
			std::vector<double> damping;
			std::vector<double> limit_length;
			std::vector<double> limit_stiffness;
			std::vector<double> is_active;
		};

		/*
		*	Add torque counteracting the difference in angular velocity between two objects.
		*/
		struct AngularVelocityMotors
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> loc_da;
			std::vector<double> max_torque;
			std::vector<double> target_w;
			std::vector<double> delay;
			std::vector<double> is_active;
		};


		/* ===========================================   FIELDS   =========================================== */
		AnchorPoints anchor_points;
		BallJoints ball_joints;
		RelativeDirectionLocks relative_direction_locks;
		PointOnAxisLocks point_on_axis_locks;
		ConeDirectionLocks cone_direction_locks;
		DampedSprings damped_springs;
		AngularVelocityMotors angular_velocity_motors;
	};
}
