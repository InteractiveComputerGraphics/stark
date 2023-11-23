#pragma once

#include "BaseRigidBodyConstraints.h"

namespace stark::models
{
	class AnchorPointHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::AnchorPoints> data;
		int idx = -1;

	public:
		inline double get_stiffness() const { return this->data->stiffness[this->idx]; };
		inline void set_stiffness(double stiffness) { this->data->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->data->is_active[this->idx] = activation; };
	};


	class DampedSpringHandler
	{
	private:
		std::shared_ptr<BaseRigidBodyConstraints::DampedSprings> data;
		int idx = -1;

	public:
		inline void set_stiffness(double stiffness) { this->data->stiffness[this->idx] = stiffness; };
		inline void enable(bool activation) { this->data->is_active[this->idx] = activation; };
	};


	class HingeJointHandler
	{
	private:
		AnchorPointHandler anchor_point;
		RelDirLockHandler relative_direction_lock;

	public:
		inline void enable(bool activation) 
		{ 
			this->anchor_point.enable(activation);
			this->relative_direction_lock.enable(activation);
		};
	};

}