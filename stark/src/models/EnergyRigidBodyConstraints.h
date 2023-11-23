#pragma once
#include <array>
#include <vector>

#include "RigidBodyDynamics.h"
#include "BaseRigidBodyConstraints.h"

namespace stark::models
{
	class EnergyRigidBodyConstraints
	{
	public:
        EnergyRigidBodyConstraints(Stark& stark, const spRigidBodyDynamics dyn);
        
        int add_anchor_point(int rb, const Eigen::Vector3d& loc, const Eigen::Vector3d& target_glob, double stiffness);
        int add_ball_joint(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double stiffness);
        int add_relative_direction_lock(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& db_loc, double stiffness);
        int add_point_on_axis_constraint(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& b_loc, double stiffness);
        int add_damped_spring(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double rest_length, double stiffness, double damping, double limit_length, double limit_stiffness);
        int add_distance_limits(int rb_a, int rb_b, const Eigen::Vector3d& a_loc, const Eigen::Vector3d& b_loc, double rest_length, double stiffness, double damping, double limit_length, double limit_stiffness);
        int add_angle_limits(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, const Eigen::Vector3d& db_loc, double admissible_dot, double stiffness);
        int add_relative_linear_velocity_motor(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, double target_v, double max_force, double delay);
        int add_relative_angular_velocity_motor(int rb_a, int rb_b, const Eigen::Vector3d& da_loc, double target_w, double max_torque, double delay);

    private:
        /* Fields */
        const spRigidBodyDynamics dyn;
        BaseRigidBodyConstraints constraints;

        /* Methods */
        symx::Scalar _set_c1_controller_energy(symx::Energy& energy, const symx::Scalar& v, const symx::Scalar& target_v, const symx::Scalar& max_force, const symx::Scalar& delay, const symx::Scalar& dt);

        symx::Vector _get_x1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc);
        symx::Vector _get_d1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& d_loc);
        std::array<symx::Vector, 2> _get_x1_d1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Vector& d_loc);
        std::array<symx::Vector, 2> _get_x0_x1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc);
	};
    using spEnergyRigidBodyConstraints = std::shared_ptr<EnergyRigidBodyConstraints>;
}
