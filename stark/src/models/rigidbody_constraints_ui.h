#pragma once

#include "BaseRigidBodyConstraints.h"

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

	public:
		AnchorPointHandler(std::shared_ptr<BaseRigidBodyConstraints::AnchorPoints> contraints, int idx) : constraints(constraints), idx(idx) {};
		inline Eigen::Vector3d get_global_point() const { return this->constraints->target_glob[this->idx]; };
		inline Eigen::Vector3d get_local_point() const { return this->constraints->loc[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };
		inline void set_global_point(Eigen::Vector3d& x) { this->constraints->target_glob[this->idx] = x; };
		inline void set_local_point(Eigen::Vector3d& x) { this->constraints->loc[this->idx] = x; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = activation; };
	};

	class BallJointHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::BallJoints> constraints;
		int idx = -1;

	public:
		BallJointHandler(std::shared_ptr<BaseRigidBodyConstraints::BallJoints> contraints, int idx) : constraints(constraints), idx(idx) {};
		inline Eigen::Vector3d get_local_point_obj_a() const { return this->constraints->a_loc[this->idx]; };
		inline Eigen::Vector3d get_local_point_obj_b() const { return this->constraints->b_loc[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void get_local_point_obj_a(const Eigen::Vector3d& x) const { this->constraints->a_loc[this->idx] = x; };
		inline void get_local_point_obj_b(const Eigen::Vector3d& x) const { this->constraints->b_loc[this->idx] = x; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = activation; };
	};

	class RelativeDirectionLockHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::RelativeDirectionLocks> constraints;
		int idx = -1;

	public:
		RelativeDirectionLockHandler(std::shared_ptr<BaseRigidBodyConstraints::RelativeDirectionLocks> contraints, int idx) : constraints(constraints), idx(idx) {};
		inline Eigen::Vector3d get_local_direction_obj_a() const { return this->constraints->da_loc[this->idx]; };
		inline Eigen::Vector3d get_local_direction_obj_b() const { return this->constraints->db_loc[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void get_local_direction_obj_a(const Eigen::Vector3d& d) const { this->constraints->da_loc[this->idx] = d; };
		inline void get_local_direction_obj_b(const Eigen::Vector3d& d) const { this->constraints->db_loc[this->idx] = d; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = activation; };
	};

	class PointOnAxisConstraintHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::PointOnAxisConstraints> constraints;
		int idx = -1;

	public:
		PointOnAxisConstraintHandler(std::shared_ptr<BaseRigidBodyConstraints::PointOnAxisConstraints> constraints, int idx) : constraints(constraints), idx(idx) {};
		inline Eigen::Vector3d get_local_point_obj_a() const { return this->constraints->a_loc[this->idx]; };
		inline Eigen::Vector3d get_local_direction_obj_a() const { return this->constraints->da_loc[this->idx]; };
		inline Eigen::Vector3d get_local_point_obj_b() const { return this->constraints->b_loc[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_point_obj_a(const Eigen::Vector3d& x) { this->constraints->a_loc[this->idx] = x; };
		inline void set_local_direction_obj_a(const Eigen::Vector3d& d) { this->constraints->da_loc[this->idx] = d; };
		inline void set_local_point_obj_b(const Eigen::Vector3d& x) { this->constraints->b_loc[this->idx] = x; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = activation; };
	};

	class DampedSpringHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::DampedSprings> constraints;
		int idx = -1;

	public:
		DampedSpringHandler(std::shared_ptr<BaseRigidBodyConstraints::DampedSprings> constraints, int idx) : constraints(constraints), idx(idx) {};
		inline Eigen::Vector3d get_local_point_obj_a() const { return this->constraints->a_loc[this->idx]; };
		inline Eigen::Vector3d get_local_point_obj_b() const { return this->constraints->b_loc[this->idx]; };
		inline double get_rest_length() const { return this->constraints->rest_length[this->idx]; };
		inline double get_damping() const { return this->constraints->damping[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_point_obj_a(const Eigen::Vector3d& x) { this->constraints->a_loc[this->idx] = x; };
		inline void set_local_point_obj_b(const Eigen::Vector3d& x) { this->constraints->b_loc[this->idx] = x; };
		inline void set_rest_length(double length) { this->constraints->rest_length[this->idx] = length; };
		inline void set_damping(double damping) { this->constraints->damping[this->idx] = damping; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = activation; };
	};

	class DistanceLimitHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::DistanceLimits> constraints;
		int idx = -1;

	public:
		DistanceLimitHandler(std::shared_ptr<BaseRigidBodyConstraints::DistanceLimits> constraints, int idx) : constraints(constraints), idx(idx) {};
		inline Eigen::Vector3d get_local_point_obj_a() const { return this->constraints->a_loc[this->idx]; };
		inline Eigen::Vector3d get_local_point_obj_b() const { return this->constraints->b_loc[this->idx]; };
		inline double get_min_length() const { return this->constraints->min_length[this->idx]; };
		inline double get_max_length() const { return this->constraints->max_length[this->idx]; };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_point_obj_a(const Eigen::Vector3d& x) { this->constraints->a_loc[this->idx] = x; };
		inline void set_local_point_obj_b(const Eigen::Vector3d& x) { this->constraints->b_loc[this->idx] = x; };
		inline void set_min_length(double length) { this->constraints->min_length[this->idx] = length; };
		inline void set_max_length(double length) { this->constraints->max_length[this->idx] = length; };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = activation; };
	};

	class AngleLimitHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::AngleLimits> constraints;
		int idx = -1;

	public:
		AngleLimitHandler(std::shared_ptr<BaseRigidBodyConstraints::AngleLimits> constraints, int idx) : constraints(constraints), idx(idx) {};
		inline Eigen::Vector3d get_local_direction_obj_a() const { return this->constraints->da_loc[this->idx]; };
		inline Eigen::Vector3d get_local_direction_obj_b() const { return this->constraints->db_loc[this->idx]; };
		inline double get_limit_angle_deg() const { return utils::rad2deg(std::acos(this->constraints->admissible_dot[this->idx])); };
		inline double get_stiffness() const { return this->constraints->stiffness[this->idx]; };

		inline void set_local_direction_obj_a(const Eigen::Vector3d& x) { this->constraints->da_loc[this->idx] = x; };
		inline void set_local_direction_obj_b(const Eigen::Vector3d& x) { this->constraints->db_loc[this->idx] = x; };
		inline void set_limit_angle_deg(double angle) { this->constraints->admissible_dot[this->idx] = std::cos(utils::deg2rad(angle)); };
		inline void set_stiffness(double stiffness) { this->constraints->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = activation; };
	};

	class RelativeLinearVelocityMotorHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::RelativeLinearVelocityMotors> constraints;
		int idx = -1;

	public:
		RelativeLinearVelocityMotorHandler(std::shared_ptr<BaseRigidBodyConstraints::RelativeLinearVelocityMotors> constraints, int idx) : constraints(constraints), idx(idx) {};
		inline Eigen::Vector3d get_local_direction_obj_a() const { return this->constraints->da_loc[this->idx]; };
		inline double get_target_velocity() const { return this->constraints->target_v[this->idx]; };
		inline double get_max_force() const { return this->constraints->max_force[this->idx]; };
		inline double get_delay() const { return this->constraints->delay[this->idx]; };

		inline void set_local_direction_obj_a(const Eigen::Vector3d& x) { this->constraints->da_loc[this->idx] = x; };
		inline void set_target_velocity(double velocity) { this->constraints->target_v[this->idx] = velocity; };
		inline void set_max_force(double force) { this->constraints->max_force[this->idx] = force; };
		inline void set_delay(double delay) { this->constraints->delay[this->idx] = delay; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = activation; };
	};

	class RelativeAngularVelocityMotorHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::RelativeAngularVelocityMotors> constraints;
		int idx = -1;

	public:
		RelativeAngularVelocityMotorHandler(std::shared_ptr<BaseRigidBodyConstraints::RelativeAngularVelocityMotors> constraints, int idx) : constraints(constraints), idx(idx) {};
		inline Eigen::Vector3d get_local_direction_obj_a() const { return this->constraints->da_loc[this->idx]; };
		inline double get_target_angular_velocity() const { return this->constraints->target_w[this->idx]; };
		inline double get_max_torque() const { return this->constraints->max_torque[this->idx]; };
		inline double get_delay() const { return this->constraints->delay[this->idx]; };

		inline void set_local_direction_obj_a(const Eigen::Vector3d& x) { this->constraints->da_loc[this->idx] = x; };
		inline void set_target_angular_velocity(double velocity) { this->constraints->target_w[this->idx] = velocity; };
		inline void set_max_torque(double torque) { this->constraints->max_torque[this->idx] = torque; };
		inline void set_delay(double delay) { this->constraints->delay[this->idx] = delay; };
		inline void enable(bool activation) { this->constraints->is_active[this->idx] = activation; };
	};

	/* ============================================================================================= */
	/* ===================================  DERIVED CONSTRAINTS  =================================== */
	/* ============================================================================================= */
	class HingeJointHandler
	{
	private:
		BallJointHandler ball_joint;
		RelativeDirectionLockHandler relative_direction_lock;

	public:
		HingeJointHandler(BallJointHandler ball_joint, RelativeDirectionLockHandler relative_direction_lock) 
			: ball_joint(ball_joint), relative_direction_lock(relative_direction_lock) {};
		
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
	public:

	}



}