#pragma once

#include "RigidBodyDynamics.h"

namespace stark::models
{
	class EnergyRigidBodyConstraints
	{
	public:
        /* Fields */
        const spRigidBodyDynamics dyn;
        symx::LabelledConnectivity<1> conn{ { "rb" } };
        std::vector<double> linear_damping; // per obj
        std::vector<double> angular_damping; // per obj
        std::vector<Eigen::Matrix3d> J_loc;  // Inertia tensor local coordinates
        std::vector<double> mass;
        std::vector<std::array<double, 9>> J0_glob;  // Inertia tensor local coordinates
        std::vector<std::array<double, 9>> J0_inv_glob;  // Inverse inertia tensor local coordinates

        /* Methods */
        EnergyRigidBodyInertia(Stark& stark, const spRigidBodyDynamics dyn);
        void add(const double mass, const Eigen::Matrix3d& inertia_loc, const double linear_damping, const double angular_damping);

    private:
        // Stark callbacks
        void _before_time_step(Stark& stark);
	};
}
