#pragma once
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <utility>

#include <Eigen/Dense>
#include <symx>

#include "../../utils/mesh_utils.h"

namespace stark::models
{
	struct RigidBodyConstraints
	{
		/* ===========================================   HELPERS  =========================================== */
		//inline static double get_angle_deg_from_distance_violation(double violation)
		//{
		//	return utils::rad2deg(std::asin(violation));
		//}
		//inline static double get_angle_deg_from_dot(double dot)
		//{
		//	return utils::rad2deg(std::acos(dot)); // converts from the dot product of normalized vectors with that angle
		//}

		/* ===========================================   DEFINITIONS  =========================================== */
		/*
			Terms:
				- absolute: with respect to the global frame of reference
				- local/relative/(missing): with respect to the local frame of reference of an object
				- direction: general direction withuot a starting point
				- axis: point and direction

			Notes:
				- is_active is 1.0 if true and -1.0 if false
		*/

		/*
			[1]
			Formulation of direction constraints:
				Direction constraints (alignments, target angles, ...) are formulated in terms of "displacement"
				between unitary direction vectors spawn from the global (0, 0, 0). Formulating in terms of 
				displacement has the advantage of producing clear forces when derivatives are taken. In constrast,
				formulations in terms of actual angles would require trigonometric functions which can be easily solved,
				but reconstructing the force (which would be highly non-linear) is not easy anymore.

				Example:
					Two local directions (da_loc, db_loc) are constrained to be aligned in the global frame of reference.
					After rotating them to obtain global directions (da, db), we can see them as two displacement vectors,
					which would represent two points at exactly 1 unit distance from the global origin. We can constrain these
					two points to have distance equals to zero with the potential E = 0.5*k*(da - db).norm(). This has a force
					magnitude to f = k*(da - db) with direction (da - db).normalized(), which is easy to convert to torque as
					we know that both da and db have length of 1.

				Direction constraints use linear distance stiffness [N/m]. Even though it is possible to find an equivalent
				stiffness in units of [Nm/rad], that relation it highly non-linear and we find it less intuitive. The principle
				of our *hard constraint* formulations is "to have enough stiffness to reach a tolerance". Theoretically,
				stiffnesses should be infinite in these cases. The only relevant values are force/torque and tolerances.
		*/


		/*
		*	Disables absolute displacement of a point in a an object.
		*/
		struct GlobalPoints
		{
			symx::LabelledConnectivity<2> conn{ { "idx", "rb" } };
			std::vector<Eigen::Vector3d> loc;
			std::vector<Eigen::Vector3d> target_glob;
			std::vector<double> stiffness;
			std::vector<double> tolerance_in_m;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb, const Eigen::Vector3d& loc, const Eigen::Vector3d& target_glob, double stiffness, double tolerance_in_m)
			{
				const int id = this->conn.numbered_push_back({rb});
				this->loc.push_back(loc);
				this->target_glob.push_back(target_glob);
				this->stiffness.push_back(stiffness);
				this->tolerance_in_m.push_back(tolerance_in_m);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& target, const symx::Vector& p)
			{
				return 0.5 * k * (target - p).squared_norm();  // E = 0.5*k*u.norm()**2
			}
			static std::pair<double, Eigen::Vector3d> violation_in_m_and_force(double k, const Eigen::Vector3d& target, const Eigen::Vector3d& p)
			{
				const Eigen::Vector3d u = target - p;
				const double C = u.norm();
				return { C, k * C * u/C };  // { [m], [N] }
			}
		};


		/*
		*	Forces the a local direction on an object to be aligned to a global direction.
		* 
		*	The formulation is "displacement" between global direction vectors equals to zero. See [1].
		*/
		struct GlobalDirections
		{
			symx::LabelledConnectivity<2> conn{ { "idx", "rb" } };
			std::vector<Eigen::Vector3d> d_loc;
			std::vector<Eigen::Vector3d> target_d_glob;
			std::vector<double> stiffness;
			std::vector<double> tolerance_in_deg;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb, const Eigen::Vector3d& d_loc, const Eigen::Vector3d& target_d_glob, double stiffness, double tolerance_in_deg)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({rb});
				this->d_loc.push_back(d_loc.normalized());
				this->target_d_glob.push_back(target_d_glob.normalized());
				this->stiffness.push_back(stiffness);
				this->tolerance_in_deg.push_back(tolerance_in_deg);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& d_target, const symx::Vector& d)
			{
				return 0.5 * k * (d_target - d).squared_norm();
			}
			static std::pair<double, Eigen::Vector3d> violation_in_deg_and_torque(double k, const Eigen::Vector3d& d_target, const Eigen::Vector3d& d)
			{
				const Eigen::Vector3d u = d_target - d;
				const double C = u.norm();
				const Eigen::Vector3d force = k*C*u/C;

				const double angle_deg = utils::rad2deg(std::asin(C));
				const Eigen::Vector3d torque = d_target.cross(force);

				return { angle_deg, torque };  // { [deg], [Nm] }
			}

		};


		/*
		*	Disables absolute displacement of a point in a an object. (Ball joint)
		*/
		struct Points
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> a_loc;
			std::vector<Eigen::Vector3d> b_loc;
			std::vector<double> stiffness;
			std::vector<double> tolerance_in_m;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double stiffness, double tolerance_in_m)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->b_loc.push_back(b_loc);
				this->stiffness.push_back(stiffness);
				this->tolerance_in_m.push_back(tolerance_in_m);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& p, const symx::Vector& q)
			{
				return 0.5 * k * (q - p).squared_norm();
			}
			static std::pair<double, Eigen::Vector3d> violation_in_m_and_force(double k, const Eigen::Vector3d& p, const Eigen::Vector3d& q)
			{
				const Eigen::Vector3d u = q - p;
				const double C = u.norm();
				return { C, k * C * u / C };  // { [m], [N] }
			}
		};

		/*
		*	Disables absolute displacement of a point in a an object. (Ball joint)
		*/
		struct Distance
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> a_loc;
			std::vector<Eigen::Vector3d> b_loc;
			std::vector<double> target_distance;
			std::vector<double> stiffness;
			std::vector<double> tolerance_in_m;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double target_distance, double stiffness, double tolerance_in_m)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->b_loc.push_back(b_loc);
				this->target_distance.push_back(target_distance);
				this->stiffness.push_back(stiffness);
				this->tolerance_in_m.push_back(tolerance_in_m);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& p, const symx::Vector& q, const symx::Scalar& target_distance)
			{
				return 0.5 * k * (target_distance - (q - p).norm()).powN(2);
			}
			static std::pair<double, Eigen::Vector3d> violation_in_m_and_force(double k, const Eigen::Vector3d& p, const Eigen::Vector3d& q, const double target_distance)
			{
				const Eigen::Vector3d u = q - p;
				const double d = u.norm();
				const double C = target_distance - d;
				return { C, k * C * u / d };  // { [m], [N] }
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
			std::vector<double> tolerance;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& db_loc, double stiffness, double tolerance)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc.normalized());
				this->db_loc.push_back(db_loc.normalized());
				this->stiffness.push_back(stiffness);
				this->tolerance.push_back(tolerance);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
		};

		/*
		*	Enforces a point on a object to move along a relative direction of another object.
		*/
		struct PointOnAxis
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> a_loc;
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<Eigen::Vector3d> b_loc;
			std::vector<double> stiffness;
			std::vector<double> tolerance;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& b_loc, double stiffness, double tolerance)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->da_loc.push_back(da_loc.normalized());
				this->b_loc.push_back(b_loc);
				this->stiffness.push_back(stiffness);
				this->tolerance.push_back(tolerance);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
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
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double rest_length, double stiffness, double damping)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->b_loc.push_back(b_loc);
				this->rest_length.push_back(rest_length);
				this->stiffness.push_back(stiffness);
				this->damping.push_back(damping);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
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
			std::vector<double> tolerance;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double min_length, double max_length, double stiffness, double tolerance)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->b_loc.push_back(b_loc);
				this->min_length.push_back(min_length);
				this->max_length.push_back(max_length);
				this->stiffness.push_back(stiffness);
				this->tolerance.push_back(tolerance);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
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
			std::vector<double> tolerance;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& db_loc, double admissible_dot, double stiffness, double tolerance)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc.normalized());
				this->db_loc.push_back(db_loc.normalized());
				this->admissible_dot.push_back(admissible_dot);
				this->stiffness.push_back(stiffness);
				this->tolerance.push_back(tolerance);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
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
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, double target_v, double max_force, double delay)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc.normalized());
				this->target_v.push_back(target_v);
				this->max_force.push_back(max_force);
				this->delay.push_back(delay);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
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
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, double target_w, double max_torque, double delay)
			{
				const int id = (int)this->is_active.size();
				this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc.normalized());
				this->target_w.push_back(target_w);
				this->max_torque.push_back(max_torque);
				this->delay.push_back(delay);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
		};
	};
}
