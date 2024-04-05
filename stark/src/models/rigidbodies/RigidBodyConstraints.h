#pragma once
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <utility>

#include <Eigen/Dense>
#include <symx>

#include "../../utils/include.h"
#include "../distances.h"

namespace stark
{
	struct RigidBodyConstraints
	{
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

				Direction constraints use linear distance bending_stiffness [N/m]. Even though it is possible to find an equivalent
				bending_stiffness in units of [Nm/rad], that relation it highly non-linear and we find it less intuitive. The principle
				of our *hard constraint* formulations is "to have enough bending_stiffness to reach a tolerance". Theoretically,
				stiffnesses should be infinite in these cases. The only relevant values are force/torque and tolerances.
		*/
		constexpr static double EPS = 100.0 * std::numeric_limits<double>::epsilon();

		static symx::Scalar c1_controller_energy(const symx::Vector& da1, const symx::Vector& va1,const symx::Vector& vb1, const symx::Scalar& target_v, const symx::Scalar& max_force, const symx::Scalar& delay, const symx::Scalar& dt)
		{
			// Constraint (Analogous to C1 friction)
			// The force is max_force when the relative velocity is larger than delay (both in positive and negative direction)
			// Otherwise, the force is proportional to the relative velocity
			// Important: derivatives wrt "positions", therefore needed chain rule and resulted in added product by dt
			symx::Scalar v = da1.dot(vb1 - va1);
			symx::Scalar k = max_force / delay;
			symx::Scalar eps = delay/2.0;
			symx::Scalar dv = v - target_v;
			symx::Scalar E_c = 0.5 * k * dv.powN(2) * dt;
			symx::Scalar E_r = max_force * (dv - eps) * dt;
			symx::Scalar E_l = -E_r;
			symx::Scalar E = symx::branch(dv < -delay, E_l, symx::branch(dv < delay, E_c, E_r));
			return E;
		}
		static std::array<double, 2> signed_c1_controller_violation_and_force(const Eigen::Vector3d& da1, const Eigen::Vector3d& va1, const Eigen::Vector3d& vb1, const double target_v, const double max_force, const double delay)
		{
			const double v = da1.dot(vb1 - va1);
			const double k = max_force / delay;
			const double dv = v - target_v;

			if (dv < -delay) {
				return { dv, -max_force };
			}
			else if (dv < delay) {
				return { dv, -k * dv };
			}
			else {
				return { dv, max_force };
			}
		}

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
			static std::array<double, 2> violation_in_m_and_force(double k, const Eigen::Vector3d& target, const Eigen::Vector3d& p)
			{
				const Eigen::Vector3d u = p - target;
				const double C = u.norm();
				return { C, k * C };  // { [m], [N] }
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
			static std::array<double, 2> violation_in_deg_and_torque(double k, const Eigen::Vector3d& d_target, const Eigen::Vector3d& d)
			{
				const Eigen::Vector3d u = d - d_target;
				const double C = u.norm();
				const Eigen::Vector3d force = -k*C*u/(C + EPS);

				const double angle_deg = rad2deg(std::asin(C));
				const Eigen::Vector3d torque = d_target.cross(force);

				return { angle_deg, torque.norm() };  // { [deg], [Nm] }
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
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& a, const symx::Vector& b)
			{
				return 0.5 * k * (b - a).squared_norm();
			}
			static std::array<double, 2> violation_in_m_and_force(double k, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
			{
				const Eigen::Vector3d u = b - a;
				const double C = u.norm();
				return { C, k * C };  // { [m], [N] }
			}
		};

		/*
		*	Enforces a point on a object to move along a direction relative to another object.
		*/
		struct PointOnAxes
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> a_loc;
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<Eigen::Vector3d> b_loc;
			std::vector<double> stiffness;
			std::vector<double> tolerance_in_m;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& b_loc, double stiffness, double tolerance_in_m)
			{
				const int id = this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->da_loc.push_back(da_loc.normalized());
				this->b_loc.push_back(b_loc);
				this->stiffness.push_back(stiffness);
				this->tolerance_in_m.push_back(tolerance_in_m);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& a, const symx::Vector& da, const symx::Vector& b)
			{
				return 0.5 * k * sq_distance_point_line(b, a, a + da);
			}
			static std::array<double, 2> violation_in_m_and_force(double k, const Eigen::Vector3d& a, const Eigen::Vector3d& da, const Eigen::Vector3d& b)
			{
				const double C = std::sqrt(sq_distance_point_line(b, a, a + da));
				return { C, k * C };  // { [m], [N] }
			}
		};

		/*
		*	Enforces two points on two objects to be at a certain distance.
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
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& a, const symx::Vector& b, const symx::Scalar& target_distance)
			{
				return 0.5 * k * (target_distance - (b - a).norm()).powN(2);
			}
			static std::array<double, 2> signed_violation_in_m_and_force(double k, const Eigen::Vector3d& a, const Eigen::Vector3d& b, double target_distance)
			{
				const Eigen::Vector3d u = b - a;
				const double d = u.norm();
				const double C = d - target_distance;
				return { C, -k * C };  // { [m], [N] }  When compression: Negative C, Positive restoration force
			}
		};

		/*
		*	Limits the min and max distance between two points from two different objects.
		*/
		struct DistanceLimits
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> a_loc;
			std::vector<Eigen::Vector3d> b_loc;
			std::vector<double> min_distance;
			std::vector<double> max_distance;
			std::vector<double> stiffness;
			std::vector<double> tolerance_in_m;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double min_distance, double max_distance, double stiffness, double tolerance_in_m)
			{
				const int id = this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->b_loc.push_back(b_loc);
				this->min_distance.push_back(min_distance);
				this->max_distance.push_back(max_distance);
				this->stiffness.push_back(stiffness);
				this->tolerance_in_m.push_back(tolerance_in_m);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& a, const symx::Vector& b, const symx::Scalar& min_distance, const symx::Scalar& max_distance)
			{
				const symx::Scalar length = (b - a).norm();
				const symx::Scalar E_min = symx::branch(length < min_distance, k * symx::powN(min_distance - length, 2) / 2.0, 0.0);
				const symx::Scalar E_max = symx::branch(length > max_distance, k * symx::powN(length - max_distance, 2) / 2.0, 0.0);
				return E_min + E_max;
			}
			static std::array<double, 2> signed_violation_in_m_and_force(double k, const Eigen::Vector3d& a, const Eigen::Vector3d& b, double min_distance, double max_distance)
			{
				const double d = (b - a).norm();

				if (d < min_distance) {
					const double C = d - min_distance;
					return { C, -k * C };  // { [m], [N] }  Compression: Negative C, Positive restoration force
				}
				else if (d > max_distance) {
					const double C = d - max_distance;
					return { C, -k * C };  // { [m], [N] }  Extension: Positive C, Negative restoration force
				}
				else {
					return { 0.0, 0.0 };
				}
			}
		};

		/*
		*	Enforces two local axes in two objects to be aligned.
		* 
		*	The formulation is "displacement" between global direction vectors equals to zero. See [1].
		*/
		struct Directions
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<Eigen::Vector3d> db_loc;
			std::vector<double> stiffness;
			std::vector<double> tolerance_in_deg;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			std::vector<Eigen::Vector3d> db_loc_rest;  // Used as reference to dynamically set a target angle
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& db_loc, double stiffness, double tolerance_in_deg)
			{
				const int id = this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc.normalized());
				this->db_loc.push_back(db_loc.normalized());
				this->stiffness.push_back(stiffness);
				this->tolerance_in_deg.push_back(tolerance_in_deg);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				this->db_loc_rest.push_back(db_loc.normalized());
				return id;
			}
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& da, const symx::Vector& db)
			{
				return 0.5 * k * (db - da).squared_norm();
			}
			static std::array<double, 2> violation_in_deg_and_torque(double k, const Eigen::Vector3d& da, const Eigen::Vector3d& db)
			{
				const Eigen::Vector3d u = db - da;
				const double C = u.norm();
				const Eigen::Vector3d force = k * C * u / (C + EPS);

				const double angle_deg = rad2deg(std::asin(C));
				const Eigen::Vector3d torque = da.cross(force);

				return { angle_deg, torque.norm() };  // { [deg], [Nm] }
			}
		};

		/*
		*	Enforces a local direction in an object to be within the cone of admissible directions relative to another object.
		* 
		*	The formulation is "displacement" between global direction vectors equals to zero. See [1].
		*/
		struct AngleLimits
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<Eigen::Vector3d> db_loc;
			std::vector<double> max_distance;
			std::vector<double> stiffness;
			std::vector<double> tolerance_in_deg;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& db_loc, double max_distance, double stiffness, double tolerance_in_deg)
			{
				const int id = this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc.normalized());
				this->db_loc.push_back(db_loc.normalized());
				this->max_distance.push_back(max_distance);
				this->stiffness.push_back(stiffness);
				this->tolerance_in_deg.push_back(tolerance_in_deg);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static double opening_distance_of_angle(double angle_deg)
			{
				return std::sqrt(1.0 + 1.0 - 2.0*std::cos(deg2rad(angle_deg))); // Law of cosines for sides of unit length
			}
			static double angle_of_opening_distance(double d)
			{
				return rad2deg(std::acos((1.0 + 1.0 - d*d) / 2.0)); // Law of cosines for sides of unit length
			}
			static symx::Scalar energy(const symx::Scalar& k, const symx::Vector& da, const symx::Vector& db, const symx::Scalar& max_distance)
			{
				const symx::Scalar length = (db - da).norm();
				return symx::branch(length > max_distance, k * symx::powN(length - max_distance, 3) / 3.0, 0.0);
			}
			static std::array<double, 2> violation_in_deg_and_torque(double k, const Eigen::Vector3d& da, const Eigen::Vector3d& db, double max_distance)
			{
				const Eigen::Vector3d u = db - da;
				const double d = u.norm();

				if (d > max_distance) {
					const double C = d - max_distance;
					const Eigen::Vector3d force = k * std::pow(C, 2) * u / (d + EPS);
					return { angle_of_opening_distance(C), da.cross(force).norm()};  // { [deg], [Nm] }
				}
				else {
					return { 0.0, 0.0 };
				}
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
				const int id = this->conn.numbered_push_back({ rb_a, rb_b });
				this->a_loc.push_back(a_loc);
				this->b_loc.push_back(b_loc);
				this->rest_length.push_back(rest_length);
				this->stiffness.push_back(stiffness);
				this->damping.push_back(damping);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static symx::Scalar energy(const symx::Scalar& stiffness, const symx::Scalar& damping, const symx::Vector& a0, const symx::Vector& a1, const symx::Vector& b0, const symx::Vector& b1, const symx::Scalar& rest_length, const symx::Scalar& dt)
			{
				symx::Vector r1 = b1 - a1;
				symx::Vector r0 = b0 - a0;
				symx::Scalar l1 = r1.norm();
				symx::Scalar l0 = r0.norm();

				symx::Scalar E_spring = 0.5 * stiffness * (l1 - rest_length).powN(2);
				symx::Scalar E_damper = 0.5 * damping * ((l1 - l0) / dt).powN(2);

				return E_spring + E_damper;
			}
			static std::array<double, 2> signed_spring_violation_in_m_and_force(const double stiffness, const Eigen::Vector3d& a1, const Eigen::Vector3d& b1, const double rest_length)
			{
				const Eigen::Vector3d r1 = b1 - a1;
				const double l1 = r1.norm();
				const double C = l1 - rest_length;
				const double f = -stiffness * C;
				return { C, f };  // { [m], [N] }
			}
			static std::array<double, 2> signed_damper_velocity_and_force(const double damping, const Eigen::Vector3d& a1, const Eigen::Vector3d& b1, const Eigen::Vector3d& va1, const Eigen::Vector3d& vb1, const double rest_length)
			{
				const Eigen::Vector3d r1 = (b1 - a1).normalized();
				const double dv = (vb1 - va1).transpose() *  r1;
				const double f = -damping * dv;
				return { dv, f };  // { [m/s], [N] }
			}
		};

		/*
		*	Add torque counteracting the difference in angular velocity between two objects.
		*/
		struct LinearVelocity
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
				const int id = this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc.normalized());
				this->target_v.push_back(target_v);
				this->max_force.push_back(max_force);
				this->delay.push_back(delay);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static symx::Scalar energy(const symx::Vector& da1, const symx::Vector& va1, const symx::Vector& vb1, const symx::Scalar& target_v, const symx::Scalar& max_force, const symx::Scalar& delay, const symx::Scalar& dt)
			{
				return c1_controller_energy(da1, va1, vb1, target_v, max_force, delay, dt);
			}
			static std::array<double, 2> signed_velocity_violation_and_force(const Eigen::Vector3d& da1, const Eigen::Vector3d& va1, const Eigen::Vector3d& vb1, const double target_v, const double max_force, const double delay)
			{
				return signed_c1_controller_violation_and_force(da1, va1, vb1, target_v, max_force, delay);
			}
		};

		/*
		*	Add torque counteracting the difference in angular velocity between two objects.
		*/
		struct AngularVelocity
		{
			symx::LabelledConnectivity<3> conn{ { "idx", "a", "b" } };
			std::vector<Eigen::Vector3d> da_loc;
			std::vector<double> target_w;  // [rad/s]
			std::vector<double> max_torque;
			std::vector<double> delay;
			std::vector<double> is_active;
			std::vector<std::string> labels;
			inline int add(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, double target_w, double max_torque, double delay)
			{
				const int id = this->conn.numbered_push_back({ rb_a, rb_b });
				this->da_loc.push_back(da_loc.normalized());
				this->target_w.push_back(target_w);
				this->max_torque.push_back(max_torque);
				this->delay.push_back(delay);
				this->is_active.push_back(1.0);
				this->labels.push_back("");
				return id;
			}
			static symx::Scalar energy(const symx::Vector& da1, const symx::Vector& wa1, const symx::Vector& wb1, const symx::Scalar& target_w, const symx::Scalar& max_torque, const symx::Scalar& delay, const symx::Scalar& dt)
			{
				return c1_controller_energy(da1, wa1, wb1, target_w, max_torque, delay, dt);
			}
			static std::array<double, 2> signed_angular_velocity_violation_in_deg_per_s_and_torque(const Eigen::Vector3d& da1, const Eigen::Vector3d& wa1, const Eigen::Vector3d& wb1, const double target_w, const double max_torque, const double delay)
			{
				auto [C, t] = signed_c1_controller_violation_and_force(da1, wa1, wb1, target_w, max_torque, delay);
				return { rad2deg(C), t };
			}
		};
	};
}
