#pragma once
#include "RigidBodyDynamics.h"

namespace stark
{
	class EnergyRigidBodyInertia
	{
	public:
        /* Fields */
        spRigidBodyDynamics rb;
        symx::LabelledConnectivity<1> conn{ { "rb" } };
        std::vector<double> linear_damping; // per obj
        std::vector<double> angular_damping; // per obj
        std::vector<Eigen::Matrix3d> J_loc;  // Inertia tensor local coordinates
        std::vector<double> mass;
        std::vector<std::array<double, 9>> J0_glob;  // Inertia tensor local coordinates
        std::vector<std::array<double, 9>> J0_inv_glob;  // Inverse inertia tensor local coordinates

        /* Methods */
        EnergyRigidBodyInertia(core::Stark& stark, spRigidBodyDynamics rb);
        void add(const int rb_idx, const double mass, const Eigen::Matrix3d& inertia_loc, const double linear_damping, const double angular_damping);

    private:
        // Stark callbacks
        void _before_time_step(core::Stark& stark);
	};
}
