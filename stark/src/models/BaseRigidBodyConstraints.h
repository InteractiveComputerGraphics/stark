#pragma once
#include <vector>
#include <array>
#include <memory>

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

			Notes:
				- is_active is 1.0 if true and -1.0 if false
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
			inline int add(int rb, const Eigen::Vector3d& loc, const Eigen::Vector3d& target_glob, double stiffness)
			{
				this->conn.numbered_push_back({rb});
				this->loc.push_back(loc);
				this->target_glob.push_back(target_glob);
				this->stiffness.push_back(stiffness);
				this->is_active.push_back(1.0);
				return (int)this->is_active.size() - 1;
			}
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
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double stiffness)
			{
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->b_loc.push_back(b_loc);
				this->stiffness.push_back(stiffness);
				this->is_active.push_back(1.0);
				return (int)this->is_active.size() - 1;
			}
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
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& db_loc, double stiffness)
			{
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc);
				this->db_loc.push_back(db_loc);
				this->stiffness.push_back(stiffness);
				this->is_active.push_back(1.0);
				return (int)this->is_active.size() - 1;
			}
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
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& b_loc, double stiffness)
			{
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->da_loc.push_back(da_loc);
				this->b_loc.push_back(b_loc);
				this->stiffness.push_back(stiffness);
				this->is_active.push_back(1.0);
				return (int)this->is_active.size() - 1;
			}
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
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double rest_length, double stiffness, double damping)
			{
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->b_loc.push_back(b_loc);
				this->rest_length.push_back(rest_length);
				this->stiffness.push_back(stiffness);
				this->damping.push_back(damping);
				this->is_active.push_back(1.0);
				return (int)this->is_active.size() - 1;
			}
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
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double min_length, double max_length, double stiffness)
			{
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->b_loc.push_back(b_loc);
				this->min_length.push_back(min_length);
				this->max_length.push_back(max_length);
				this->stiffness.push_back(stiffness);
				this->is_active.push_back(1.0);
				return (int)this->is_active.size() - 1;
			}
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
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& db_loc, double admissible_dot, double stiffness)
			{
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc);
				this->db_loc.push_back(db_loc);
				this->admissible_dot.push_back(admissible_dot);
				this->stiffness.push_back(stiffness);
				this->is_active.push_back(1.0);
				return (int)this->is_active.size() - 1;
			}
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
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, double target_v, double max_force, double delay)
			{
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc);
				this->target_v.push_back(target_v);
				this->max_force.push_back(max_force);
				this->is_active.push_back(1.0);
				return (int)this->is_active.size() - 1;
			}
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
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, double target_w, double max_torque, double delay)
			{
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc);
				this->target_w.push_back(target_w);
				this->max_torque.push_back(max_torque);
				this->delay.push_back(delay);
				this->is_active.push_back(1.0);
				return (int)this->is_active.size() - 1;
			}
		};
	};
}
