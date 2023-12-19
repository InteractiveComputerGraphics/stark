#pragma once

#include "../../utils/mesh_utils.h"

#include "RigidBodyConstraints.h"
#include "RigidBodyHandler.h"

namespace stark::models
{
	/* ================================================================================================================================== */
	/* =======================================================  BASE CONSTRAINTS  ======================================================= */
	/* ================================================================================================================================== */
	
	class RBCGlobalPointHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::GlobalPoints> constraints;
		int idx = -1;
		RigidBodyHandler rb;

	public:
		RBCGlobalPointHandler(RigidBodyHandler rb, std::shared_ptr<RigidBodyConstraints::GlobalPoints> constraints, int idx)
			: rb(rb), constraints(constraints), idx(idx) 
		{};

		inline RigidBodyHandler get_body() { return this->rb; };

		inline Eigen::Vector3d get_global_target_point() const { return this->constraints->target_glob[this->idx]; };
		inline auto& set_global_target_point(Eigen::Vector3d& x) { this->constraints->target_glob[this->idx] = x; return (*this); };

		inline Eigen::Vector3d get_local_point() const { return this->constraints->loc[this->idx]; };
		inline auto& set_local_point(Eigen::Vector3d& x) { this->constraints->loc[this->idx] = x; return (*this); };

		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); };

		inline double get_tolerance_in_m() const { return this->constraints->tolerance_in_m[this->idx]; };
		inline auto& set_tolerance_in_m(double tolerance_in_m) { this->constraints->tolerance_in_m[this->idx] = tolerance_in_m; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_violation_in_m_and_force() const
		{ 
			return RigidBodyConstraints::GlobalPoints::violation_in_m_and_force(get_stiffness(), get_global_target_point(), rb.local_to_global_point(get_local_point()));
		};

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};

	class RBCGlobalDirectionHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::GlobalDirections> constraints;
		int idx = -1;
		RigidBodyHandler rb;

	public:
		RBCGlobalDirectionHandler(RigidBodyHandler rb, std::shared_ptr<RigidBodyConstraints::GlobalDirections> constraints, int idx)
			: rb(rb), constraints(constraints), idx(idx) 
		{};

		inline RigidBodyHandler get_body() { return this->rb; };

		inline Eigen::Vector3d get_global_target_direction() const { return this->constraints->target_d_glob[this->idx]; };
		inline auto& set_global_target_direction(Eigen::Vector3d& x) { this->constraints->target_d_glob[this->idx] = x; return (*this); };

		inline Eigen::Vector3d get_local_direction() const { return this->constraints->d_loc[this->idx]; };
		inline auto& set_local_direction(Eigen::Vector3d& x) { this->constraints->d_loc[this->idx] = x; return (*this); };

		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); };

		inline double get_tolerance_in_deg() const { return this->constraints->tolerance_in_deg[this->idx]; };
		inline auto& set_tolerance_in_deg(double tolerance_in_deg) { this->constraints->tolerance_in_deg[this->idx] = tolerance_in_deg; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_violation_in_deg_and_torque() const
		{
			return RigidBodyConstraints::GlobalDirections::violation_in_deg_and_torque(get_stiffness(), get_global_target_direction(), rb.local_to_global_direction(get_local_direction()));
		};

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};

	class RBCPointHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::Points> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCPointHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::Points> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) 
		{};

		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };

		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };

		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); };

		inline double get_tolerance_in_m() const { return this->constraints->tolerance_in_m[this->idx]; };
		inline auto& set_tolerance_in_m(double tolerance_in_m) { this->constraints->tolerance_in_m[this->idx] = tolerance_in_m; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_violation_in_m_and_force() const
		{
			return RigidBodyConstraints::Points::violation_in_m_and_force(get_stiffness(),
				rb_a.local_to_global_point(get_local_point_body_a()), rb_b.local_to_global_point(get_local_point_body_b()));
		};

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};

	class RBCPointOnAxis
	{
	private:
		std::shared_ptr<RigidBodyConstraints::PointOnAxes> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCPointOnAxis(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::PointOnAxes> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx)
		{};

		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };

		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };

		inline double get_tolerance_in_m() const { return this->constraints->tolerance_in_m[this->idx]; };
		inline auto& set_tolerance_in_m(double tolerance_in_m) { this->constraints->tolerance_in_m[this->idx] = tolerance_in_m; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_violation_in_m_and_force() const
		{
			return RigidBodyConstraints::PointOnAxes::violation_in_m_and_force(get_stiffness(),
				rb_a.local_to_global_point(get_local_point_body_a()), rb_a.local_to_global_direction(get_local_direction_body_a()), rb_b.local_to_global_point(get_local_point_body_b()));
		};

		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); };

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};

	class RBCDistanceHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::Distance> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCDistanceHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::Distance> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) 
		{};

		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };

		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };

		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); };

		inline double get_target_distance() const { return this->constraints->target_distance[this->idx]; };
		inline auto& set_target_distance(double target_distance) { this->constraints->target_distance[this->idx] = target_distance; return (*this); };

		inline double get_tolerance_in_m() const { return this->constraints->tolerance_in_m[this->idx]; };
		inline auto& set_tolerance_in_m(double tolerance_in_m) { this->constraints->tolerance_in_m[this->idx] = tolerance_in_m; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_violation_in_m_and_force() const
		{
			return RigidBodyConstraints::Distance::violation_in_m_and_force(get_stiffness(),
				rb_a.local_to_global_point(get_local_point_body_a()), rb_b.local_to_global_point(get_local_point_body_b()), get_target_distance());
		};

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};

	class RBCDistanceLimitHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::DistanceLimits> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCDistanceLimitHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::DistanceLimits> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx)
		{};

		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };

		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };

		inline double get_min_distance() const { return this->constraints->min_distance[this->idx]; };
		inline auto& set_min_distance(double distance) { this->constraints->min_distance[this->idx] = distance; return (*this); };

		inline double get_max_distance() const { return this->constraints->max_distance[this->idx]; };
		inline auto& set_max_distance(double distance) { this->constraints->max_distance[this->idx] = distance; return (*this); };

		inline double get_tolerance_in_m() const { return this->constraints->tolerance_in_m[this->idx]; };
		inline auto& set_tolerance_in_m(double tolerance_in_m) { this->constraints->tolerance_in_m[this->idx] = tolerance_in_m; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_violation_in_m_and_force() const
		{
			return RigidBodyConstraints::DistanceLimits::violation_in_m_and_force(get_stiffness(),
				rb_a.local_to_global_point(get_local_point_body_a()), rb_b.local_to_global_point(get_local_point_body_b()), get_min_distance(), get_max_distance());
		};

		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); };

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};

	class RBCDirection
	{
	private:
		std::shared_ptr<RigidBodyConstraints::Directions> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCDirection(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::Directions> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx)
		{};

		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };

		inline Eigen::Vector3d get_local_direction_body_b() const { return this->constraints->db_loc[this->idx]; };
		inline auto& set_local_direction_body_b(const Eigen::Vector3d& d) const { this->constraints->db_loc[this->idx] = d; return (*this); };

		inline double get_tolerance_in_deg() const { return this->constraints->tolerance_in_deg[this->idx]; };
		inline auto& set_tolerance_in_deg(double tolerance_in_deg) { this->constraints->tolerance_in_deg[this->idx] = tolerance_in_deg; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_violation_in_deg_and_torque() const
		{
			return RigidBodyConstraints::Directions::violation_in_deg_and_torque(get_stiffness(), 
				rb_a.local_to_global_direction(get_local_direction_body_a()), rb_b.local_to_global_direction(get_local_direction_body_b()));
		};

		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); };

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};

	class RBCAngleLimitHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::AngleLimits> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCAngleLimitHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::AngleLimits> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx)
		{};

		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };

		inline Eigen::Vector3d get_local_direction_body_b() const { return this->constraints->db_loc[this->idx]; };
		inline auto& set_local_direction_body_b(const Eigen::Vector3d& d) const { this->constraints->db_loc[this->idx] = d; return (*this); };

		inline double get_tolerance_in_deg() const { return this->constraints->tolerance_in_deg[this->idx]; };
		inline auto& set_tolerance_in_deg(double tolerance_in_deg) { this->constraints->tolerance_in_deg[this->idx] = tolerance_in_deg; return (*this); };

		inline double get_limit_angle_in_deg() const
		{
			return RigidBodyConstraints::AngleLimits::angle_of_opening_distance(this->constraints->max_distance[this->idx]);
		};
		inline auto& set_limit_angle_in_deg(double angle_deg)
		{
			this->constraints->max_distance[this->idx] = RigidBodyConstraints::AngleLimits::opening_distance_of_angle(angle_deg);
			return (*this);
		};

		inline std::pair<double, Eigen::Vector3d> get_violation_in_deg_and_torque() const
		{
			return RigidBodyConstraints::AngleLimits::violation_in_deg_and_torque(get_stiffness(),
				rb_a.local_to_global_direction(get_local_direction_body_a()), rb_b.local_to_global_direction(get_local_direction_body_b()), 
				this->constraints->max_distance[this->idx]);
		};

		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); };

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};


	class RBCDampedSpringHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::DampedSprings> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCDampedSpringHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::DampedSprings> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) 
		{};

		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline auto& set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; return (*this); };

		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline auto& set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; return (*this); };

		inline double get_rest_length() const { return this->constraints->rest_length[this->idx]; };
		inline auto& set_rest_length(double length) { this->constraints->rest_length[this->idx] = length; return (*this); };

		inline double get_damping() const { return this->constraints->damping[this->idx]; };
		inline auto& set_damping(double damping) { this->constraints->damping[this->idx] = damping; return (*this); };

		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline auto& set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_spring_displacement_in_m_and_force() const
		{
			return RigidBodyConstraints::DampedSprings::spring_violation_in_m_and_force(get_stiffness(),
				rb_a.local_to_global_point(get_local_point_body_a()), rb_b.local_to_global_point(get_local_point_body_b()), get_rest_length());
		};

		inline std::pair<double, Eigen::Vector3d> get_damper_velocity_and_force() const
		{
			return RigidBodyConstraints::DampedSprings::damper_velocity_and_force(get_damping(),
				rb_a.local_to_global_point(get_local_point_body_a()), rb_b.local_to_global_point(get_local_point_body_b()), 
				rb_a.get_velocity_at(get_local_point_body_a()), rb_b.get_velocity_at(get_local_point_body_b()),
				get_rest_length());
		};

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};

	class RBCLinearVelocityHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::LinearVelocity> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCLinearVelocityHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::LinearVelocity> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) 
		{};

		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };

		inline double get_target_velocity() const { return this->constraints->target_v[this->idx]; };
		inline auto& set_target_velocity(double velocity) { this->constraints->target_v[this->idx] = velocity; return (*this); };

		inline double get_max_force() const { return this->constraints->max_force[this->idx]; };
		inline auto& set_max_force(double force) { this->constraints->max_force[this->idx] = force; return (*this); };

		inline double get_delay() const { return this->constraints->delay[this->idx]; };
		inline auto& set_delay(double delay) { this->constraints->delay[this->idx] = delay; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_velocity_violation_and_force() const
		{
			return RigidBodyConstraints::LinearVelocity::velocity_violation_and_force(
				rb_a.local_to_global_direction(get_local_direction_body_a()),
				rb_a.get_velocity(),
				rb_b.get_velocity(),
				get_target_velocity(),
				get_max_force(),
				get_delay());
		};

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
	};

	class RBCAngularVelocityHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::AngularVelocity> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCAngularVelocityHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::AngularVelocity> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx)
		{};

		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };

		inline double get_target_angular_velocity() const { return this->constraints->target_w[this->idx]; };
		inline auto& set_target_angular_velocity(double velocity) { this->constraints->target_w[this->idx] = velocity; return (*this); };

		inline double get_max_torque() const { return this->constraints->max_torque[this->idx]; };
		inline auto& set_max_torque(double torque) { this->constraints->max_torque[this->idx] = torque; return (*this); };

		inline double get_delay() const { return this->constraints->delay[this->idx]; };
		inline auto& set_delay(double delay) { this->constraints->delay[this->idx] = delay; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_angular_velocity_violation_and_torque() const
		{
			return RigidBodyConstraints::AngularVelocity::angular_velocity_violation_and_torque(
				rb_a.local_to_global_direction(get_local_direction_body_a()),
				rb_a.get_velocity(),
				rb_b.get_velocity(),
				get_target_angular_velocity(),
				get_max_torque(),
				get_delay());
		};

		inline std::string get_label() const { return this->constraints->labels[this->idx]; };
		inline auto& set_label(std::string label) { this->constraints->labels[this->idx] = label; return (*this); };

		inline auto& enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; return (*this); };
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

	//class HingeJointHandler
	//{
	//private:
	//	BallJointHandler ball_joint;
	//	RelativeDirectionLockHandler relative_direction_lock;
	//	RigidBodyHandler rb_a;
	//	RigidBodyHandler rb_b;

	//public:
	//	HingeJointHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, BallJointHandler ball_joint, RelativeDirectionLockHandler relative_direction_lock)
	//		: rb_a(rb_a), rb_b(rb_b), ball_joint(ball_joint), relative_direction_lock(relative_direction_lock) {};
	//	inline RigidBodyHandler get_body_a() { return this->rb_a; };
	//	inline RigidBodyHandler get_body_b() { return this->rb_b; };
	//	inline BallJointHandler get_ball_joint() { return this->ball_joint; };
	//	inline RelativeDirectionLockHandler get_relative_direction_lock() { return this->relative_direction_lock; };
	//	inline void enable(bool activation)
	//	{
	//		this->ball_joint.enable(activation);
	//		this->relative_direction_lock.enable(activation);
	//	};
	//	inline auto& set_label(std::string label);
	//};

	//class HingeJointWithLimitsHandler
	//{
	//private:
	//	HingeJointHandler hinge_joint;
	//	AngleLimitHandler angle_limit;
	//	RigidBodyHandler rb_a;
	//	RigidBodyHandler rb_b;

	//public:
	//	HingeJointWithLimitsHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, HingeJointHandler hinge_joint, AngleLimitHandler angle_limit)
	//		: rb_a(rb_a), rb_b(rb_b), hinge_joint(hinge_joint), angle_limit(angle_limit) {};
	//	inline RigidBodyHandler get_body_a() { return this->rb_a; };
	//	inline RigidBodyHandler get_body_b() { return this->rb_b; };
	//	inline HingeJointHandler get_hinge_joint() { return this->hinge_joint; };
	//	inline AngleLimitHandler get_angle_limit() { return this->angle_limit; };
	//	inline void enable(bool activation)
	//	{
	//		this->hinge_joint.enable(activation);
	//		this->angle_limit.enable(activation);
	//	};
	//	inline auto& set_label(std::string label);
	//};

	//class SpringWithLimitsHandler
	//{
	//private:
	//	DampedSpringHandler spring;
	//	DistanceLimitHandler distance_limit;
	//	RigidBodyHandler rb_a;
	//	RigidBodyHandler rb_b;

	//public:
	//	SpringWithLimitsHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, DampedSpringHandler spring, DistanceLimitHandler distance_limit)
	//		: rb_a(rb_a), rb_b(rb_b), spring(spring), distance_limit(distance_limit) {};
	//	inline RigidBodyHandler get_body_a() { return this->rb_a; };
	//	inline RigidBodyHandler get_body_b() { return this->rb_b; };
	//	inline DampedSpringHandler get_spring() { return this->spring; };
	//	inline DistanceLimitHandler get_distance_limit() { return this->distance_limit; };
	//	inline void enable(bool activation)
	//	{
	//		this->spring.enable(activation);
	//		this->distance_limit.enable(activation);
	//	};
	//	inline auto& set_label(std::string label);
	//};

	//class SliderHandler
	//{
	//private:
	//	PointOnAxisConstraintHandler point_on_axes;
	//	RelativeDirectionLockHandler relative_direction_lock;
	//	RigidBodyHandler rb_a;
	//	RigidBodyHandler rb_b;

	//public:
	//	SliderHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, PointOnAxisConstraintHandler point_on_axes, RelativeDirectionLockHandler relative_direction_lock)
	//		: rb_a(rb_a), rb_b(rb_b), point_on_axes(point_on_axes), relative_direction_lock(relative_direction_lock) {};
	//	inline RigidBodyHandler get_body_a() { return this->rb_a; };
	//	inline RigidBodyHandler get_body_b() { return this->rb_b; };
	//	inline PointOnAxisConstraintHandler get_point_on_axis() { return this->point_on_axes; };
	//	inline RelativeDirectionLockHandler get_relative_direction_lock() { return this->relative_direction_lock; };
	//	inline void enable(bool activation)
	//	{
	//		this->point_on_axes.enable(activation);
	//		this->relative_direction_lock.enable(activation);
	//	};
	//	inline auto& set_label(std::string label);
	//};

	//class PrismaticSliderHandler
	//{
	//private:
	//	SliderHandler slider;
	//	RelativeDirectionLockHandler relative_direction_lock;
	//	RigidBodyHandler rb_a;
	//	RigidBodyHandler rb_b;

	//public:
	//	PrismaticSliderHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, SliderHandler slider, RelativeDirectionLockHandler relative_direction_lock)
	//		: rb_a(rb_a), rb_b(rb_b), slider(slider), relative_direction_lock(relative_direction_lock) {};
	//	inline RigidBodyHandler get_body_a() { return this->rb_a; };
	//	inline RigidBodyHandler get_body_b() { return this->rb_b; };
	//	inline SliderHandler get_slider() { return this->slider; };
	//	inline RelativeDirectionLockHandler get_relative_direction_lock() { return this->relative_direction_lock; };
	//	inline void enable(bool activation)
	//	{
	//		this->slider.enable(activation);
	//		this->relative_direction_lock.enable(activation);
	//	};
	//	inline auto& set_label(std::string label);
	//};

	//class MotorHandler
	//{
	//private:
	//	HingeJointHandler hinge;
	//	RelativeAngularVelocityMotorHandler motor;
	//	RigidBodyHandler rb_a;
	//	RigidBodyHandler rb_b;

	//public:
	//	MotorHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, HingeJointHandler hinge, RelativeAngularVelocityMotorHandler motor)
	//		: rb_a(rb_a), rb_b(rb_b), hinge(hinge), motor(motor) {};
	//	inline RigidBodyHandler get_body_a() { return this->rb_a; };
	//	inline RigidBodyHandler get_body_b() { return this->rb_b; };
	//	inline HingeJointHandler get_hinge_joint() { return this->hinge; };
	//	inline RelativeAngularVelocityMotorHandler get_motor() { return this->motor; };
	//	inline void enable(bool activation)
	//	{
	//		this->hinge.enable(activation);
	//		this->motor.enable(activation);
	//	};
	//	inline auto& set_label(std::string label);
	//};

	//// Parallel Gripper, hydraulic press. Positive velocity for expansion.
	//class PrismaticPressHandler
	//{
	//private:
	//	PrismaticSliderHandler prismatic_slider;
	//	RelativeLinearVelocityMotorHandler motor;
	//	RigidBodyHandler rb_a;
	//	RigidBodyHandler rb_b;

	//public:
	//	PrismaticPressHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, PrismaticSliderHandler prismatic_slider, RelativeLinearVelocityMotorHandler motor)
	//		: rb_a(rb_a), rb_b(rb_b), prismatic_slider(prismatic_slider), motor(motor) {};
	//	inline RigidBodyHandler get_body_a() { return this->rb_a; };
	//	inline RigidBodyHandler get_body_b() { return this->rb_b; };
	//	inline PrismaticSliderHandler get_prismatic_slider() { return this->prismatic_slider; };
	//	inline RelativeLinearVelocityMotorHandler get_motor() { return this->motor; };
	//	inline void enable(bool activation)
	//	{
	//		this->prismatic_slider.enable(activation);
	//		this->motor.enable(activation);
	//	};
	//	inline auto& set_label(std::string label);
	//};
}