#pragma once

#include "../../utils/include.h"

#include "rigidbody_transformations.h"
#include "RigidBodyConstraints.h"
#include "RigidBodyHandler.h"

/*
	Note: This file is very long but it does not contain simulation models or simulation logic.
	It only contains a lot of boilerplate code to make the API more user-friendly.
*/


#define _STARK_RB_CONSTRAINT_DEFINITION_HEAD_ONE_RB(HandlerClass, ConstraintClass) \
	private: \
		std::shared_ptr<ConstraintClass> constraints; \
		int idx = -1; \
		RigidBodyHandler rb; \
	public: \
		HandlerClass(RigidBodyHandler rb, std::shared_ptr<ConstraintClass> constraints, int idx) \
			: rb(rb), constraints(constraints), idx(idx) {}; \
		HandlerClass() = default; \
		inline bool is_valid() const { return this->idx != -1; }; \
		inline RigidBodyHandler& get_body() { return this->rb; }; \
		inline std::string get_label() const { return this->constraints->labels[this->idx]; }; \
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); }; \
		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); }; \

#define _STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(HandlerClass, ConstraintClass) \
	private: \
		std::shared_ptr<ConstraintClass> constraints; \
		int idx = -1; \
		RigidBodyHandler rb_a; \
		RigidBodyHandler rb_b; \
	public: \
		HandlerClass(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<ConstraintClass> constraints, int idx) \
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) {}; \
		HandlerClass() = default; \
		inline bool is_valid() const { return this->idx != -1; }; \
		inline RigidBodyHandler& get_body_a() { return this->rb_a; }; \
		inline RigidBodyHandler& get_body_b() { return this->rb_b; }; \
		inline std::string get_label() const { return this->constraints->labels[this->idx]; }; \
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); }; \
		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); }; \

#define _STARK_RB_CONSTRAINT_DEFINE_STIFFNESS() \
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; }; \
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); }; \

#define _STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_M() \
		inline double get_tolerance_in_m() const { return this->constraints->tolerance_in_m[this->idx]; }; \
		inline auto& set_tolerance_in_m(double tolerance_in_m) { this->constraints->tolerance_in_m[this->idx] = tolerance_in_m; return (*this); }; \

#define _STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_DEG() \
		inline double get_tolerance_in_deg() const { return this->constraints->tolerance_in_deg[this->idx]; }; \
		inline auto& set_tolerance_in_deg(double tolerance_in_deg) { this->constraints->tolerance_in_deg[this->idx] = tolerance_in_deg; return (*this); }; \

namespace stark
{
	/* ================================================================================================================================== */
	/* =======================================================  BASE CONSTRAINTS  ======================================================= */
	/* ================================================================================================================================== */
	
	class RBCGlobalPointHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_ONE_RB(RBCGlobalPointHandler, RigidBodyConstraints::GlobalPoints)
		_STARK_RB_CONSTRAINT_DEFINE_STIFFNESS()
		_STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_M()

		inline Eigen::Vector3d get_global_target_point() const { return this->constraints->target_glob[this->idx]; };
		inline auto& set_global_target_point(const Eigen::Vector3d& x) { this->constraints->target_glob[this->idx] = x; return (*this); };
		inline Eigen::Vector3d get_local_point() const { return this->constraints->loc[this->idx]; };
		inline auto& set_local_point(const Eigen::Vector3d& x) { this->constraints->loc[this->idx] = x; return (*this); };
		inline std::array<double, 2> get_violation_in_m_and_force() const
		{ 
			return RigidBodyConstraints::GlobalPoints::violation_in_m_and_force(get_stiffness(), get_global_target_point(), rb.transform_local_to_global_point(get_local_point()));
		};
	};

	class RBCGlobalDirectionHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_ONE_RB(RBCGlobalDirectionHandler, RigidBodyConstraints::GlobalDirections)
		_STARK_RB_CONSTRAINT_DEFINE_STIFFNESS()
		_STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_DEG()

		inline Eigen::Vector3d get_global_target_direction() const { return this->constraints->target_d_glob[this->idx]; };
		inline auto& set_global_target_direction(const Eigen::Vector3d& x) { this->constraints->target_d_glob[this->idx] = x; return (*this); };
		inline Eigen::Vector3d get_local_direction() const { return this->constraints->d_loc[this->idx]; };
		inline auto& set_local_direction(const Eigen::Vector3d& x) { this->constraints->d_loc[this->idx] = x; return (*this); };
		inline std::array<double, 2> get_violation_in_deg_and_torque() const
		{
			return RigidBodyConstraints::GlobalDirections::violation_in_deg_and_torque(get_stiffness(), get_global_target_direction(), rb.transform_local_to_global_direction(get_local_direction()));
		};
	};

	class RBCPointHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(RBCPointHandler, RigidBodyConstraints::Points)
		_STARK_RB_CONSTRAINT_DEFINE_STIFFNESS()
		_STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_M()

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };
		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };
		inline std::array<double, 2> get_violation_in_m_and_force() const
		{
			return RigidBodyConstraints::Points::violation_in_m_and_force(get_stiffness(),
				rb_a.transform_local_to_global_point(get_local_point_body_a()), rb_b.transform_local_to_global_point(get_local_point_body_b()));
		};
	};

	class RBCPointOnAxisHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(RBCPointOnAxisHandler, RigidBodyConstraints::PointOnAxes)
		_STARK_RB_CONSTRAINT_DEFINE_STIFFNESS()
		_STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_M()

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };
		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };
		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };
		inline std::array<double, 2> get_violation_in_m_and_force() const
		{
			return RigidBodyConstraints::PointOnAxes::violation_in_m_and_force(get_stiffness(),
				rb_a.transform_local_to_global_point(get_local_point_body_a()), rb_a.transform_local_to_global_direction(get_local_direction_body_a()), rb_b.transform_local_to_global_point(get_local_point_body_b()));
		};
	};

	class RBCDistanceHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(RBCDistanceHandler, RigidBodyConstraints::Distance)
		_STARK_RB_CONSTRAINT_DEFINE_STIFFNESS()
		_STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_M()

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };
		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };
		inline double get_target_distance() const { return this->constraints->target_distance[this->idx]; };
		inline auto& set_target_distance(double target_distance) { this->constraints->target_distance[this->idx] = target_distance; return (*this); };
		inline std::array<double, 2> get_signed_violation_in_m_and_force() const
		{
			return RigidBodyConstraints::Distance::signed_violation_in_m_and_force(get_stiffness(),
				rb_a.transform_local_to_global_point(get_local_point_body_a()), rb_b.transform_local_to_global_point(get_local_point_body_b()), get_target_distance());
		};
	};

	class RBCDistanceLimitHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(RBCDistanceLimitHandler, RigidBodyConstraints::DistanceLimits)
		_STARK_RB_CONSTRAINT_DEFINE_STIFFNESS()
		_STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_M()

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };
		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };
		inline double get_min_distance() const { return this->constraints->min_distance[this->idx]; };
		inline auto& set_min_distance(double distance) { this->constraints->min_distance[this->idx] = distance; return (*this); };
		inline double get_max_distance() const { return this->constraints->max_distance[this->idx]; };
		inline auto& set_max_distance(double distance) { this->constraints->max_distance[this->idx] = distance; return (*this); };
		inline std::array<double, 2> get_signed_violation_in_m_and_force() const
		{
			return RigidBodyConstraints::DistanceLimits::signed_violation_in_m_and_force(get_stiffness(),
				rb_a.transform_local_to_global_point(get_local_point_body_a()), rb_b.transform_local_to_global_point(get_local_point_body_b()), get_min_distance(), get_max_distance());
		};
	};

	class RBCDirectionHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(RBCDirectionHandler, RigidBodyConstraints::Directions)
		_STARK_RB_CONSTRAINT_DEFINE_STIFFNESS()
		_STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_DEG()

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };
		inline Eigen::Vector3d get_local_direction_body_b() const { return this->constraints->db_loc[this->idx]; };
		inline auto& set_local_direction_body_b(const Eigen::Vector3d& d) const { this->constraints->db_loc[this->idx] = d; return (*this); };
		inline std::array<double, 2> get_violation_in_deg_and_torque() const
		{
			return RigidBodyConstraints::Directions::violation_in_deg_and_torque(get_stiffness(), 
				rb_a.transform_local_to_global_direction(get_local_direction_body_a()), rb_b.transform_local_to_global_direction(get_local_direction_body_b()));
		};
		inline auto& set_opening_angle_rad(double angle_rad, const Eigen::Vector3d& normal_loc_da)
		{
			const Eigen::Matrix3d R = Eigen::AngleAxisd(angle_rad, normal_loc_da).toRotationMatrix();
			this->set_local_direction_body_b(R * this->constraints->db_loc_rest[this->idx]);
			return (*this);
		}
		inline auto& set_opening_angle_deg(double angle_deg, const Eigen::Vector3d& normal_loc_da)
		{
			return this->set_opening_angle_rad(deg2rad(angle_deg), normal_loc_da);
		}
	};

	class RBCAngleLimitHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(RBCAngleLimitHandler, RigidBodyConstraints::AngleLimits)
		_STARK_RB_CONSTRAINT_DEFINE_STIFFNESS()
		_STARK_RB_CONSTRAINT_DEFINE_TOLERANCE_IN_DEG()

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };
		inline Eigen::Vector3d get_local_direction_body_b() const { return this->constraints->db_loc[this->idx]; };
		inline auto& set_local_direction_body_b(const Eigen::Vector3d& d) const { this->constraints->db_loc[this->idx] = d; return (*this); };
		inline double get_limit_angle_in_deg() const
		{
			return RigidBodyConstraints::AngleLimits::angle_of_opening_distance(this->constraints->max_distance[this->idx]);
		};
		inline auto& set_limit_angle_in_deg(double angle_deg)
		{
			this->constraints->max_distance[this->idx] = RigidBodyConstraints::AngleLimits::opening_distance_of_angle(angle_deg);
			return (*this);
		};
		inline std::array<double, 2> get_violation_in_deg_and_torque() const
		{
			return RigidBodyConstraints::AngleLimits::violation_in_deg_and_torque(get_stiffness(),
				rb_a.transform_local_to_global_direction(get_local_direction_body_a()), rb_b.transform_local_to_global_direction(get_local_direction_body_b()), 
				this->constraints->max_distance[this->idx]);
		};
	};


	class RBCDampedSpringHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(RBCDampedSpringHandler, RigidBodyConstraints::DampedSprings)
		_STARK_RB_CONSTRAINT_DEFINE_STIFFNESS()

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };
		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };
		inline double get_rest_length() const { return this->constraints->rest_length[this->idx]; };
		inline auto& set_rest_length(double length) { this->constraints->rest_length[this->idx] = length; return (*this); };
		inline double get_damping() const { return this->constraints->damping[this->idx]; };
		inline auto& set_damping(double damping) { this->constraints->damping[this->idx] = damping; return (*this); };
		inline std::array<double, 2> get_signed_spring_displacement_in_m_and_force() const
		{
			return RigidBodyConstraints::DampedSprings::signed_spring_violation_in_m_and_force(get_stiffness(),
				rb_a.transform_local_to_global_point(get_local_point_body_a()), rb_b.transform_local_to_global_point(get_local_point_body_b()), get_rest_length());
		};
		inline std::array<double, 2> get_signed_damper_velocity_and_force() const
		{
			return RigidBodyConstraints::DampedSprings::signed_damper_velocity_and_force(get_damping(),
				rb_a.transform_local_to_global_point(get_local_point_body_a()), rb_b.transform_local_to_global_point(get_local_point_body_b()), 
				rb_a.get_velocity_at(get_local_point_body_a()), rb_b.get_velocity_at(get_local_point_body_b()),
				get_rest_length());
		};
	};

	class RBCLinearVelocityHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(RBCLinearVelocityHandler, RigidBodyConstraints::LinearVelocity)

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };
		inline double get_target_velocity() const { return this->constraints->target_v[this->idx]; };
		inline auto& set_target_velocity(double velocity) { this->constraints->target_v[this->idx] = velocity; return (*this); };
		inline double get_max_force() const { return this->constraints->max_force[this->idx]; };
		inline auto& set_max_force(double force) { 
			// Check that force is not negative
			if (force < 0.0) {
				std::cout << "stark error: RBCLinearVelocityHandler.set_max_force() found a negative force" << std::endl;
				exit(-1);
			}

			this->constraints->max_force[this->idx] = force; 
			return (*this); 
		};
		inline double get_delay() const { return this->constraints->delay[this->idx]; };
		inline auto& set_delay(double delay) { this->constraints->delay[this->idx] = delay; return (*this); };
		inline std::array<double, 2> get_signed_velocity_violation_and_force() const
		{
			return RigidBodyConstraints::LinearVelocity::signed_velocity_violation_and_force(
				rb_a.transform_local_to_global_direction(get_local_direction_body_a()),
				rb_a.get_velocity(),
				rb_b.get_velocity(),
				get_target_velocity(),
				get_max_force(),
				get_delay());
		};
	};

	class RBCAngularVelocityHandler
	{
		_STARK_RB_CONSTRAINT_DEFINITION_HEAD_TWO_RB(RBCAngularVelocityHandler, RigidBodyConstraints::AngularVelocity)

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };
		inline double get_target_angular_velocity_in_rad_per_s() const { return this->constraints->target_w[this->idx]; };
		inline auto& set_target_angular_velocity_in_rad_per_s(double w) { this->constraints->target_w[this->idx] = w; return (*this); };
		inline double get_target_angular_velocity_in_deg_per_s() const { return rad2deg(this->constraints->target_w[this->idx]); };
		inline auto& set_target_angular_velocity_in_deg_per_s(double w) { this->constraints->target_w[this->idx] = deg2rad(w); return (*this); };
		inline double get_max_torque() const { return this->constraints->max_torque[this->idx]; };
		inline auto& set_max_torque(double torque) { 
			// Check that torque is not negative
			if (torque < 0.0) { 
				std::cout << "stark error: RBCAngularVelocityHandler.set_max_abs_torque() found a negative torque" << std::endl;
				exit(-1);
			}

			this->constraints->max_torque[this->idx] = torque; 
			return (*this);
		};
		inline double get_delay() const { return this->constraints->delay[this->idx]; };
		inline auto& set_delay(double delay) { this->constraints->delay[this->idx] = delay; return (*this); };
		inline std::array<double, 2> get_signed_angular_velocity_violation_in_deg_per_s_and_torque() const
		{
			return RigidBodyConstraints::AngularVelocity::signed_angular_velocity_violation_in_deg_per_s_and_torque(
				rb_a.transform_local_to_global_direction(get_local_direction_body_a()),
				rb_a.get_angular_velocity(),
				rb_b.get_angular_velocity(),
				this->constraints->target_w[this->idx],
				get_max_torque(),
				get_delay());
		};
	};


	/* ===================================================================================================================================== */
	/* =======================================================  DERIVED CONSTRAINTS  ======================================================= */
	/* ===================================================================================================================================== */
	class RBCFixHandler
	{
	private:
		RBCGlobalPointHandler anchor_point;
		RBCGlobalDirectionHandler z_lock;
		RBCGlobalDirectionHandler x_lock;
		RigidBodyHandler rb;
		std::string label = "";

	public:
		RBCFixHandler(RigidBodyHandler rb, RBCGlobalPointHandler anchor_point, RBCGlobalDirectionHandler z_lock, RBCGlobalDirectionHandler x_lock)
			: rb(rb), anchor_point(anchor_point), z_lock(z_lock), x_lock(x_lock) {};
		RBCFixHandler() = default;
		inline bool is_valid() const { return this->anchor_point.is_valid(); };
		inline auto& get_body() { return this->rb; };
		inline auto& get_anchor_point() { return this->anchor_point; };
		inline auto& get_z_lock() { return this->z_lock; };
		inline auto& get_x_lock() { return this->x_lock; };

		inline auto& set_stiffness(double stiffness) 
		{
			this->anchor_point.set_stiffness(stiffness);
			this->z_lock.set_stiffness(stiffness);
			this->x_lock.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->anchor_point.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}
		inline auto& set_tolerance_in_deg(double tolerance_in_deg)
		{
			this->z_lock.set_tolerance_in_deg(tolerance_in_deg);
			this->x_lock.set_tolerance_in_deg(tolerance_in_deg);
			return (*this);
		}
		inline auto& set_transformation(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
		{
			this->anchor_point.set_global_target_point(translation);
			this->z_lock.set_global_target_direction(rotation.col(2));
			this->x_lock.set_global_target_direction(rotation.col(0));
			return (*this);
		}
		inline auto& set_transformation(const Eigen::Vector3d& translation, const double angle_deg, const Eigen::Vector3d& axis)
		{
			return this->set_transformation(translation, Eigen::AngleAxisd(deg2rad(angle_deg), axis).toRotationMatrix());
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label) 
		{ 
			this->label = label;
			this->anchor_point.set_label(label + "_anchor_point");
			this->z_lock.set_label(label + "_z_lock");
			this->x_lock.set_label(label + "_x_lock");
			return (*this); 
		};

		inline auto& enable(bool activation)
		{
			this->anchor_point.enable(activation);
			this->z_lock.enable(activation);
			this->x_lock.enable(activation);
			return (*this);
		};
	};

	class RBCAttachmentHandler
	{
	private:
		RBCPointHandler point;
		RBCDirectionHandler z_lock;
		RBCDirectionHandler x_lock;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;
		std::string label = "";

	public:
		RBCAttachmentHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCPointHandler point, RBCDirectionHandler z_lock, RBCDirectionHandler x_lock)
			: rb_a(rb_a), rb_b(rb_b), point(point), z_lock(z_lock), x_lock(x_lock) {};
		RBCAttachmentHandler() = default;
		inline bool is_valid() const { return this->point.is_valid(); };
		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };
		inline auto& get_point_constraint() { return this->point; };
		inline auto& get_z_lock() { return this->z_lock; };
		inline auto& get_x_lock() { return this->x_lock; };

		inline auto& set_stiffness(double stiffness) 
		{
			this->point.set_stiffness(stiffness);
			this->z_lock.set_stiffness(stiffness);
			this->x_lock.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->point.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}
		inline auto& set_tolerance_in_deg(double tolerance_in_deg)
		{
			this->z_lock.set_tolerance_in_deg(tolerance_in_deg);
			this->x_lock.set_tolerance_in_deg(tolerance_in_deg);
			return (*this);
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label) 
		{ 
			this->label = label;
			this->point.set_label(label + "_point");
			this->z_lock.set_label(label + "_z_lock");
			this->x_lock.set_label(label + "_x_lock");
			return (*this); 
		};
		inline auto& enable(bool activation)
		{
			this->point.enable(activation);
			this->z_lock.enable(activation);
			this->x_lock.enable(activation);
			return (*this);
		};
	};

	class RBCPointWithAngleLimitHandler
	{
	private:
		RBCPointHandler point;
		RBCAngleLimitHandler angle_limit;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;
		std::string label = "";

	public:
		RBCPointWithAngleLimitHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCPointHandler point, RBCAngleLimitHandler angle_limit)
			: rb_a(rb_a), rb_b(rb_b), point(point), angle_limit(angle_limit) {};
		RBCPointWithAngleLimitHandler() = default;
		inline bool is_valid() const { return this->point.is_valid(); };
		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };
		inline RBCPointHandler& get_point_constraint() { return this->point; };
		inline RBCAngleLimitHandler& get_angle_limit() { return this->angle_limit; };
		inline auto& set_stiffness(double stiffness)
		{
			this->point.set_stiffness(stiffness);
			this->angle_limit.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->point.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}
		inline auto& set_tolerance_in_deg(double tolerance_in_deg)
		{
			this->angle_limit.set_tolerance_in_deg(tolerance_in_deg);
			return (*this);
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label)
		{
			this->label = label;
			this->point.set_label(label + "_point");
			this->angle_limit.set_label(label + "_angle_limits");
			return (*this);
		};
		inline auto& enable(bool activation)
		{
			this->point.enable(activation);
			this->angle_limit.enable(activation);
			return (*this);
		};
	};

	class RBCHingeJointHandler
	{
	private:
		RBCPointHandler point;
		RBCDirectionHandler direction;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;
		std::string label = "";

	public:
		RBCHingeJointHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCPointHandler point, RBCDirectionHandler direction)
			: rb_a(rb_a), rb_b(rb_b), point(point), direction(direction) {};
		RBCHingeJointHandler() = default;
		inline bool is_valid() const { return this->point.is_valid(); };
		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };
		inline RBCPointHandler& get_point_constraint() { return this->point; };
		inline RBCDirectionHandler& get_direction_constraint() { return this->direction; };
		inline auto& set_stiffness(double stiffness)
		{
			this->point.set_stiffness(stiffness);
			this->direction.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->point.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}
		inline auto& set_tolerance_in_deg(double tolerance_in_deg)
		{
			this->direction.set_tolerance_in_deg(tolerance_in_deg);
			return (*this);
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label)
		{
			this->label = label;
			this->point.set_label(label + "_point");
			this->direction.set_label(label + "_direction");
			return (*this);
		};
		inline auto& enable(bool activation)
		{
			this->point.enable(activation);
			this->direction.enable(activation);
			return (*this);
		};
	};

	class RBCHingeJointWithAngleLimitHandler
	{
	private:
		RBCHingeJointHandler hinge_joint;
		RBCAngleLimitHandler angle_limit;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;
		std::string label = "";

	public:
		RBCHingeJointWithAngleLimitHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCHingeJointHandler hinge_joint, RBCAngleLimitHandler angle_limit)
			: rb_a(rb_a), rb_b(rb_b), hinge_joint(hinge_joint), angle_limit(angle_limit) {};
		RBCHingeJointWithAngleLimitHandler() = default;
		inline bool is_valid() const { return this->hinge_joint.is_valid(); };
		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };
		inline RBCHingeJointHandler& get_hinge_joint() { return this->hinge_joint; };
		inline RBCAngleLimitHandler& get_angle_limit() { return this->angle_limit; };

		inline auto& set_stiffness(double stiffness)
		{
			this->hinge_joint.set_stiffness(stiffness);
			this->angle_limit.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->hinge_joint.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}
		inline auto& set_tolerance_in_deg(double tolerance_in_deg)
		{
			this->hinge_joint.set_tolerance_in_deg(tolerance_in_deg);
			this->angle_limit.set_tolerance_in_deg(tolerance_in_deg);
			return (*this);
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label)
		{
			this->label = label;
			this->hinge_joint.set_label(label + "_hinge");
			this->angle_limit.set_label(label + "_angle_limits");
			return (*this);
		};
		inline auto& enable(bool activation)
		{
			this->hinge_joint.enable(activation);
			this->angle_limit.enable(activation);
			return (*this);
		};
	};

	class RBCSpringWithLimitsHandler
	{
	private:
		RBCDampedSpringHandler spring;
		RBCDistanceLimitHandler distance_limit;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;
		std::string label = "";

	public:
		RBCSpringWithLimitsHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCDampedSpringHandler spring, RBCDistanceLimitHandler distance_limit)
			: rb_a(rb_a), rb_b(rb_b), spring(spring), distance_limit(distance_limit) {};
		RBCSpringWithLimitsHandler() = default;
		inline bool is_valid() const { return this->spring.is_valid(); };
		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };
		inline RBCDampedSpringHandler& get_spring() { return this->spring; };
		inline RBCDistanceLimitHandler& get_distance_limit() { return this->distance_limit; };

		inline auto& set_spring_stiffness(double stiffness)
		{
			this->spring.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_stiffness(double stiffness)
		{
			this->distance_limit.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->distance_limit.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label)
		{
			this->label = label;
			this->spring.set_label(label + "_spring");
			this->distance_limit.set_label(label + "_distance_limit");
			return (*this);
		};
		inline auto& enable(bool activation)
		{
			this->spring.enable(activation);
			this->distance_limit.enable(activation);
			return (*this);
		};
	};

	class RBCSliderHandler
	{
	private:
		RBCPointOnAxisHandler point_on_axis;
		RBCDirectionHandler direction;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;
		std::string label = "";

	public:
		RBCSliderHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCPointOnAxisHandler point_on_axis, RBCDirectionHandler direction)
			: rb_a(rb_a), rb_b(rb_b), point_on_axis(point_on_axis), direction(direction) {};
		RBCSliderHandler() = default;
		inline bool is_valid() const { return this->point_on_axis.is_valid(); };
		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };
		inline RBCPointOnAxisHandler& get_point_on_axis() { return this->point_on_axis; };
		inline RBCDirectionHandler& get_direction_lock() { return this->direction; };

		inline auto& set_stiffness(double stiffness)
		{
			this->point_on_axis.set_stiffness(stiffness);
			this->direction.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->point_on_axis.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}
		inline auto& set_tolerance_in_deg(double tolerance_in_deg)
		{
			this->direction.set_tolerance_in_deg(tolerance_in_deg);
			return (*this);
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label)
		{
			this->label = label;
			this->point_on_axis.set_label(label + "_point_on_axis");
			this->direction.set_label(label + "_long_direction");
			return (*this);
		};
		inline auto& enable(bool activation)
		{
			this->point_on_axis.enable(activation);
			this->direction.enable(activation);
			return (*this);
		};
	};

	class RBCPrismaticSliderHandler
	{
	private:
		RBCSliderHandler slider;
		RBCDirectionHandler direction;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;
		std::string label = "";

	public:
		RBCPrismaticSliderHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCSliderHandler slider, RBCDirectionHandler direction)
			: rb_a(rb_a), rb_b(rb_b), slider(slider), direction(direction) {};
		RBCPrismaticSliderHandler() = default;
		inline bool is_valid() const { return this->slider.is_valid(); };
		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };
		inline RBCSliderHandler& get_slider() { return this->slider; };
		inline RBCDirectionHandler& get_direction_lock() { return this->direction; };

		inline auto& set_stiffness(double stiffness)
		{
			this->slider.set_stiffness(stiffness);
			this->direction.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->slider.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}
		inline auto& set_tolerance_in_deg(double tolerance_in_deg)
		{
			this->slider.set_tolerance_in_deg(tolerance_in_deg);
			this->direction.set_tolerance_in_deg(tolerance_in_deg);
			return (*this);
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label)
		{
			this->label = label;
			this->slider.set_label(label + "_slider");
			this->direction.set_label(label + "_orth_direction");
			return (*this);
		};
		inline auto& enable(bool activation)
		{
			this->slider.enable(activation);
			this->direction.enable(activation);
			return (*this);
		};
	};

	class RBCPrismaticPressHandler
	{
	private:
		RBCPrismaticSliderHandler prismatic_slider;
		RBCLinearVelocityHandler linear_velocity;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;
		std::string label = "";

	public:
		RBCPrismaticPressHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCPrismaticSliderHandler prismatic_slider, RBCLinearVelocityHandler linear_velocity)
			: rb_a(rb_a), rb_b(rb_b), prismatic_slider(prismatic_slider), linear_velocity(linear_velocity) {};
		RBCPrismaticPressHandler() = default;
		inline bool is_valid() const { return this->prismatic_slider.is_valid(); };
		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };
		inline RBCPrismaticSliderHandler& get_prismatic_slider() { return this->prismatic_slider; };
		inline RBCLinearVelocityHandler& get_linear_velocity_constraint() { return this->linear_velocity; };

		inline auto& set_stiffness(double stiffness)
		{
			this->prismatic_slider.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->prismatic_slider.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}
		inline auto& set_tolerance_in_deg(double tolerance_in_deg)
		{
			this->prismatic_slider.set_tolerance_in_deg(tolerance_in_deg);
			return (*this);
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label)
		{
			this->label = label;
			this->prismatic_slider.set_label(label + "_prismatic_slider");
			this->linear_velocity.set_label(label + "_linear_velocity");
			return (*this);
		};
		inline auto& enable(bool activation)
		{
			this->prismatic_slider.enable(activation);
			this->linear_velocity.enable(activation);
			return (*this);
		};
	};

	class RBCMotorHandler
	{
	private:
		RBCHingeJointHandler hinge;
		RBCAngularVelocityHandler angular_velocity;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;
		std::string label = "";

	public:
		RBCMotorHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCHingeJointHandler hinge, RBCAngularVelocityHandler angular_velocity)
			: rb_a(rb_a), rb_b(rb_b), hinge(hinge), angular_velocity(angular_velocity) {};
		RBCMotorHandler() = default;
		inline bool is_valid() const { return this->hinge.is_valid(); };
		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };
		inline RBCHingeJointHandler& get_hinge_joint() { return this->hinge; };
		inline RBCAngularVelocityHandler& get_angular_velocity_constraint() { return this->angular_velocity; };

		inline auto& set_stiffness(double stiffness)
		{
			this->hinge.set_stiffness(stiffness);
			return (*this);
		};
		inline auto& set_tolerance_in_m(double tolerance_in_m)
		{
			this->hinge.set_tolerance_in_m(tolerance_in_m);
			return (*this);
		}
		inline auto& set_tolerance_in_deg(double tolerance_in_deg)
		{
			this->hinge.set_tolerance_in_deg(tolerance_in_deg);
			return (*this);
		}

		inline std::string get_label() const { return this->label; };
		inline auto& set_label(std::string label)
		{
			this->label = label;
			this->hinge.set_label(label + "_hinge");
			this->angular_velocity.set_label(label + "_angular_velocity");
			return (*this);
		};
		inline auto& enable(bool activation)
		{
			this->hinge.enable(activation);
			this->angular_velocity.enable(activation);
			return (*this);
		};
	};
}