#pragma once

#include "../../utils/mesh_utils.h"

#include "RigidBodyConstraints.h"
#include "RigidBodyHandler.h"

/*
	Note: This file is very long but it does not contain simulation models or simulation logic.
	It only contains a lot of boilerplate code to make the API more user-friendly.
*/


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

		inline RigidBodyHandler& get_body() { return this->rb; };

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

		inline RigidBodyHandler& get_body() { return this->rb; };

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

		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };

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

	class RBCPointOnAxisHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::PointOnAxes> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCPointOnAxisHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::PointOnAxes> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx)
		{};

		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };

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

		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };

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

		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };

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

	class RBCDirectionHandler
	{
	private:
		std::shared_ptr<RigidBodyConstraints::Directions> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RBCDirectionHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<RigidBodyConstraints::Directions> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx)
		{};

		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };

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

		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };

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

		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };

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

		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };

		inline double get_target_velocity() const { return this->constraints->target_v[this->idx]; };
		inline auto& set_target_velocity_in_m_per_s(double velocity) { this->constraints->target_v[this->idx] = velocity; return (*this); };

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

		inline RigidBodyHandler& get_body_a() { return this->rb_a; };
		inline RigidBodyHandler& get_body_b() { return this->rb_b; };

		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline auto& set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; return (*this); };

		inline double get_target_angular_velocity_in_deg_per_s() const { return utils::rad2deg(this->constraints->target_w[this->idx]); };
		inline auto& set_target_angular_velocity_in_deg_per_s(double w) { this->constraints->target_w[this->idx] = utils::deg2rad(w); return (*this); };

		inline double get_max_torque() const { return this->constraints->max_torque[this->idx]; };
		inline auto& set_max_torque(double torque) { this->constraints->max_torque[this->idx] = torque; return (*this); };

		inline double get_delay() const { return this->constraints->delay[this->idx]; };
		inline auto& set_delay(double delay) { this->constraints->delay[this->idx] = delay; return (*this); };

		inline std::pair<double, Eigen::Vector3d> get_angular_velocity_violation_in_deg_per_s_and_torque() const
		{
			return RigidBodyConstraints::AngularVelocity::angular_velocity_violation_in_deg_per_s_and_torque(
				rb_a.local_to_global_direction(get_local_direction_body_a()),
				rb_a.get_velocity(),
				rb_b.get_velocity(),
				this->constraints->target_w[this->idx],
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
		RBCSliderHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, RBCPointOnAxisHandler point_on_axes, RBCDirectionHandler direction)
			: rb_a(rb_a), rb_b(rb_b), point_on_axis(point_on_axis), direction(direction) {};
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