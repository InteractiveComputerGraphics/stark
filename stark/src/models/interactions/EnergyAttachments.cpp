#include "EnergyAttachments.h"

#include "../time_integration.h"


stark::models::EnergyAttachments::EnergyAttachments(core::Stark& stark, const spPointDynamics dyn, const spRigidBodyDynamics rb)
	: dyn(dyn), rb(rb)
{
	stark.global_energy.add_energy("EnergyAttachments_d_d", this->conn_d_d,
		[&](symx::Energy& energy, symx::Element& pair)
		{
			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, { pair["a"], pair["b"] });
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, { pair["a"], pair["b"] });
			symx::Scalar k = energy.make_scalar(this->stiffness_d_d, pair["group"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1[1] - x1[0]).squared_norm();
			energy.set(E);
		}
	);

	stark.global_energy.add_energy("EnergyAttachments_rb_d", this->conn_rb_d,
		[&](symx::Energy& energy, symx::Element& pair)
		{
			// Create symbols
			symx::Scalar k = energy.make_scalar(this->stiffness_rb_d, pair["group"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			//// Deformable
			symx::Vector v1_d = energy.make_dof_vector(this->dyn->dof, this->dyn->v1.data, pair["p"]);
			symx::Vector x0_d = energy.make_vector(this->dyn->x0.data, pair["p"]);

			//// Rigid body
			symx::Vector x_loc = energy.make_vector(this->rb_points_loc, pair["idx"]);
			symx::Vector x1_rb = this->rb->get_x1(energy, pair["rb"], x_loc, dt);

			// Time integration
			symx::Vector x1_d = time_integration(x0_d, v1_d, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1_d - x1_rb).squared_norm();
			energy.set(E);
		}
	);
}
