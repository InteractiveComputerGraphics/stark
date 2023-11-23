#pragma once
#include <array>
#include <vector>
#include <memory>

#include "RigidBodyDynamics.h"
#include "BaseRigidBodyConstraints.h"

namespace stark::models
{
	class EnergyRigidBodyConstraints
	{
	public:
        EnergyRigidBodyConstraints(Stark& stark, const spRigidBodyDynamics dyn);

    private:
        /* Fields */
        const spRigidBodyDynamics dyn;
        std::shared_ptr<BaseRigidBodyConstraints> constraints;

        /* Methods */
        symx::Scalar _set_c1_controller_energy(symx::Energy& energy, const symx::Scalar& v, const symx::Scalar& target_v, const symx::Scalar& max_force, const symx::Scalar& delay, const symx::Scalar& dt);

        symx::Vector _get_x1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc);
        symx::Vector _get_d1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& d_loc);
        std::array<symx::Vector, 2> _get_x1_d1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Vector& d_loc);
        std::array<symx::Vector, 2> _get_x0_x1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc);
	};
    using spEnergyRigidBodyConstraints = std::shared_ptr<EnergyRigidBodyConstraints>;
}
