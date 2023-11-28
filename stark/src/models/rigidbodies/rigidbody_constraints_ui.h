#pragma once

#include "../../utils/mesh_utils.h"

#include "BaseRigidBodyConstraints.h"
#include "RigidBodyHandler.h"

namespace stark::models
{

	/* ========================================================================================== */
	/* ===================================  BASE CONSTRAINTS  =================================== */
	/* ========================================================================================== */
	
	class AnchorPointHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::AnchorPoints> constraints;
		int idx = -1;
		RigidBodyHandler rb;

	public:
		AnchorPointHandler(RigidBodyHandler rb, std::shared_ptr<BaseRigidBodyConstraints::AnchorPoints> constraints, int idx) 
			: rb(rb), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body() { return this->rb; };
		inline Eigen::Vector3d get_global_target_point() const { return this->constraints->target_glob[this->idx]; };
		inline Eigen::Vector3d get_local_point() const { return this->constraints->loc[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline void set_global_target_point(Eigen::Vector3d& x) { this->constraints->target_glob[this->idx] = x; };
		inline void set_local_point(Eigen::Vector3d& x) { this->constraints->loc[this->idx] = x; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};

	class AbsoluteDirectionLockHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::AbsoluteDirectionLocks> constraints;
		int idx = -1;
		RigidBodyHandler rb;

	public:
		AbsoluteDirectionLockHandler(RigidBodyHandler rb, std::shared_ptr<BaseRigidBodyConstraints::AbsoluteDirectionLocks> constraints, int idx)
			: rb(rb), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body() { return this->rb; };
		inline Eigen::Vector3d get_global_target_direction() const { return this->constraints->target_d_glob[this->idx]; };
		inline Eigen::Vector3d get_local_direction() const { return this->constraints->d_loc[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline void set_global_target_direction(Eigen::Vector3d& x) { this->constraints->target_d_glob[this->idx] = x; };
		inline void set_local_direction(Eigen::Vector3d& x) { this->constraints->d_loc[this->idx] = x; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};

	class BallJointHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::BallJoints> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		BallJointHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<BaseRigidBodyConstraints::BallJoints> constraints, int idx) 
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_point_body_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; };
		inline void set_local_point_body_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};

	class RelativeDirectionLockHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::RelativeDirectionLocks> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RelativeDirectionLockHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<BaseRigidBodyConstraints::RelativeDirectionLocks> constraints, int idx) 
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline Eigen::Vector3d get_local_direction_body_b() const { return this->constraints->db_loc[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_direction_body_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; };
		inline void set_local_direction_body_b(const Eigen::Vector3d& d) const { this->constraints->db_loc[this->idx] = d; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};

	class PointOnAxisConstraintHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::PointOnAxis> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		PointOnAxisConstraintHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<BaseRigidBodyConstraints::PointOnAxis> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_point_body_a(const Eigen::Vector3d& x) { this->constraints->a_loc[this->idx] = x; };
		inline void set_local_direction_body_a(const Eigen::Vector3d& d) { this->constraints->da_loc[this->idx] = d; };
		inline void set_local_point_body_b(const Eigen::Vector3d& x) { this->constraints->b_loc[this->idx] = x; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};

	class DampedSpringHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::DampedSprings> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		DampedSpringHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<BaseRigidBodyConstraints::DampedSprings> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline double get_rest_length() const { return this->constraints->rest_length[this->idx]; };
		inline double get_damping() const { return this->constraints->damping[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_point_body_a(const Eigen::Vector3d& x) { this->constraints->a_loc[this->idx] = x; };
		inline void set_local_point_body_b(const Eigen::Vector3d& x) { this->constraints->b_loc[this->idx] = x; };
		inline void set_rest_length(double length) { this->constraints->rest_length[this->idx] = length; };
		inline void set_damping(double damping) { this->constraints->damping[this->idx] = damping; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};

	class DistanceLimitHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::DistanceLimits> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		DistanceLimitHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<BaseRigidBodyConstraints::DistanceLimits> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline Eigen::Vector3d get_local_point_body_a() const { return this->constraints->a_loc[this->idx]; };
		inline Eigen::Vector3d get_local_point_body_b() const { return this->constraints->b_loc[this->idx]; };
		inline double get_min_length() const { return this->constraints->min_length[this->idx]; };
		inline double get_max_length() const { return this->constraints->max_length[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_point_body_a(const Eigen::Vector3d& x) { this->constraints->a_loc[this->idx] = x; };
		inline void set_local_point_body_b(const Eigen::Vector3d& x) { this->constraints->b_loc[this->idx] = x; };
		inline void set_min_length(double length) { this->constraints->min_length[this->idx] = length; };
		inline void set_max_length(double length) { this->constraints->max_length[this->idx] = length; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};

	class AngleLimitHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::AngleLimits> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		AngleLimitHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<BaseRigidBodyConstraints::AngleLimits> constraints, int idx)
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline Eigen::Vector3d get_local_direction_body_b() const { return this->constraints->db_loc[this->idx]; };
		inline double get_limit_angle_deg() const { return utils::rad2deg(std::acos(this->constraints->admissible_dot[this->idx])); };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_direction_body_a(const Eigen::Vector3d& x) { this->constraints->da_loc[this->idx] = x; };
		inline void set_local_direction_body_b(const Eigen::Vector3d& x) { this->constraints->db_loc[this->idx] = x; };
		inline void set_limit_angle_deg(double angle) { this->constraints->admissible_dot[this->idx] = std::cos(utils::deg2rad(angle)); };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};

	class RelativeLinearVelocityMotorHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::RelativeLinearVelocityMotors> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RelativeLinearVelocityMotorHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<BaseRigidBodyConstraints::RelativeLinearVelocityMotors> constraints, int idx) 
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline Eigen::Vector3d get_local_direction_body_a() const { return this->constraints->da_loc[this->idx]; };
		inline double get_target_velocity() const { return this->constraints->target_v[this->idx]; };
		inline double get_max_force() const { return this->constraints->max_force[this->idx]; };
		inline double get_delay() const { return this->constraints->delay[this->idx]; };

		inline void set_local_direction_body_a(const Eigen::Vector3d& x) { this->constraints->da_loc[this->idx] = x; };
		inline void set_target_velocity(double velocity) { this->constraints->target_v[this->idx] = velocity; };
		inline void set_max_force(double force) { this->constraints->max_force[this->idx] = force; };
		inline void set_delay(double delay) { this->constraints->delay[this->idx] = delay; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};

	class RelativeAngularVelocityMotorHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::RelativeAngularVelocityMotors> constraints;
		int idx = -1;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		RelativeAngularVelocityMotorHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, std::shared_ptr<BaseRigidBodyConstraints::RelativeAngularVelocityMotors> constraints, int idx) 
			: rb_a(rb_a), rb_b(rb_b), constraints(constraints), idx(idx) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline double get_target_angular_velocity() const { return this->constraints->target_w[this->idx]; };
		inline double get_max_torque() const { return this->constraints->max_torque[this->idx]; };
		inline double get_delay() const { return this->constraints->delay[this->idx]; };

		inline void set_local_direction_body_a(const Eigen::Vector3d& x) { this->constraints->da_loc[this->idx] = x; };
		inline void set_target_angular_velocity(double velocity) { this->constraints->target_w[this->idx] = velocity; };
		inline void set_max_torque(double torque) { this->constraints->max_torque[this->idx] = torque; };
		inline void set_delay(double delay) { this->constraints->delay[this->idx] = delay; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = (activation) ? 1.0 : -1.0; };
	};


	/* ============================================================================================= */
	/* ===================================  DERIVED CONSTRAINTS  =================================== */
	/* ============================================================================================= */
	class FixedConstraintHandler
	{
	private:
		AnchorPointHandler anchor_point;
		AbsoluteDirectionLockHandler z_lock;
		AbsoluteDirectionLockHandler x_lock;
		RigidBodyHandler rb;

	public:
		FixedConstraintHandler(RigidBodyHandler rb, AnchorPointHandler anchor_point, AbsoluteDirectionLockHandler z_lock, AbsoluteDirectionLockHandler x_lock)
			: rb(rb), anchor_point(anchor_point), z_lock(z_lock), x_lock(x_lock) {};
		inline RigidBodyHandler get_body() { return this->rb; };
		inline AnchorPointHandler get_anchor_point() { return this->anchor_point; };
		inline AbsoluteDirectionLockHandler get_z_lock() { return this->z_lock; };
		inline AbsoluteDirectionLockHandler get_x_lock() { return this->x_lock; };
		inline void enable(bool activation)
		{
			this->anchor_point.enable(activation);
			this->z_lock.enable(activation);
			this->x_lock.enable(activation);
		};
	};

	class HingeJointHandler
	{
	private:
		BallJointHandler ball_joint;
		RelativeDirectionLockHandler relative_direction_lock;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		HingeJointHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, BallJointHandler ball_joint, RelativeDirectionLockHandler relative_direction_lock)
			: rb_a(rb_a), rb_b(rb_b), ball_joint(ball_joint), relative_direction_lock(relative_direction_lock) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline BallJointHandler get_ball_joint() { return this->ball_joint; };
		inline RelativeDirectionLockHandler get_relative_direction_lock() { return this->relative_direction_lock; };
		inline void enable(bool activation)
		{
			this->ball_joint.enable(activation);
			this->relative_direction_lock.enable(activation);
		};
	};

	class HingeJointWithLimitsHandler
	{
	private:
		HingeJointHandler hinge_joint;
		AngleLimitHandler angle_limit;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		HingeJointWithLimitsHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, HingeJointHandler hinge_joint, AngleLimitHandler angle_limit)
			: rb_a(rb_a), rb_b(rb_b), hinge_joint(hinge_joint), angle_limit(angle_limit) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline HingeJointHandler get_hinge_joint() { return this->hinge_joint; };
		inline AngleLimitHandler get_angle_limit() { return this->angle_limit; };
		inline void enable(bool activation)
		{
			this->hinge_joint.enable(activation);
			this->angle_limit.enable(activation);
		};
	};

	class SpringWithLimitsHandler
	{
	private:
		DampedSpringHandler spring;
		DistanceLimitHandler distance_limit;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		SpringWithLimitsHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, DampedSpringHandler spring, DistanceLimitHandler distance_limit)
			: rb_a(rb_a), rb_b(rb_b), spring(spring), distance_limit(distance_limit) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline DampedSpringHandler get_spring() { return this->spring; };
		inline DistanceLimitHandler get_distance_limit() { return this->distance_limit; };
		inline void enable(bool activation)
		{
			this->spring.enable(activation);
			this->distance_limit.enable(activation);
		};
	};
	
	class SliderHandler
	{
	private:
		PointOnAxisConstraintHandler point_on_axis;
		RelativeDirectionLockHandler relative_direction_lock;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		SliderHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, PointOnAxisConstraintHandler point_on_axis, RelativeDirectionLockHandler relative_direction_lock)
			: rb_a(rb_a), rb_b(rb_b), point_on_axis(point_on_axis), relative_direction_lock(relative_direction_lock) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline PointOnAxisConstraintHandler get_point_on_axis() { return this->point_on_axis; };
		inline RelativeDirectionLockHandler get_relative_direction_lock() { return this->relative_direction_lock; };
		inline void enable(bool activation)
		{
			this->point_on_axis.enable(activation);
			this->relative_direction_lock.enable(activation);
		};
	};
	
	class PrismaticSliderHandler
	{
	private:
		SliderHandler slider;
		RelativeDirectionLockHandler relative_direction_lock;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		PrismaticSliderHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, SliderHandler slider, RelativeDirectionLockHandler relative_direction_lock)
			: rb_a(rb_a), rb_b(rb_b), slider(slider), relative_direction_lock(relative_direction_lock) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline SliderHandler get_slider() { return this->slider; };
		inline RelativeDirectionLockHandler get_relative_direction_lock() { return this->relative_direction_lock; };
		inline void enable(bool activation)
		{
			this->slider.enable(activation);
			this->relative_direction_lock.enable(activation);
		};
	};
	
	class MotorHandler
	{
	private:
		HingeJointHandler hinge;
		RelativeAngularVelocityMotorHandler motor;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		MotorHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, HingeJointHandler hinge, RelativeAngularVelocityMotorHandler motor)
			: rb_a(rb_a), rb_b(rb_b), hinge(hinge), motor(motor) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline HingeJointHandler get_hinge_joint() { return this->hinge; };
		inline RelativeAngularVelocityMotorHandler get_motor() { return this->motor; };
		inline void enable(bool activation)
		{
			this->hinge.enable(activation);
			this->motor.enable(activation);
		};
	};

	// Parallel Gripper, hydraulic press. Positive velocity for expansion.
	class PrismaticPressHandler
	{
	private:
		PrismaticSliderHandler prismatic_slider;
		RelativeLinearVelocityMotorHandler motor;
		RigidBodyHandler rb_a;
		RigidBodyHandler rb_b;

	public:
		PrismaticPressHandler(RigidBodyHandler rb_a, RigidBodyHandler rb_b, PrismaticSliderHandler prismatic_slider, RelativeLinearVelocityMotorHandler motor)
			: rb_a(rb_a), rb_b(rb_b), prismatic_slider(prismatic_slider), motor(motor) {};
		inline RigidBodyHandler get_body_a() { return this->rb_a; };
		inline RigidBodyHandler get_body_b() { return this->rb_b; };
		inline PrismaticSliderHandler get_prismatic_slider() { return this->prismatic_slider; };
		inline RelativeLinearVelocityMotorHandler get_motor() { return this->motor; };
		inline void enable(bool activation)
		{
			this->prismatic_slider.enable(activation);
			this->motor.enable(activation);
		};
	};
}