#include "EnergyFrictionalContact.h"

#include "../time_integration.h"

std::vector<symx::Vector> stark::models::EnergyFrictionalContact::_get_rb_x1(const std::vector<symx::Index>& indices, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
{
	symx::Vector v1 = energy.make_dof_vector(this->rigid_bodies->dof_v, this->rigid_bodies->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->rigid_bodies->dof_w, this->rigid_bodies->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->rigid_bodies->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->rigid_bodies->q0_, rb_idx);

	symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
	symx::Vector t1 = time_integration(t0, v1, dt);

	std::vector<symx::Vector> x_loc = energy.make_vectors(this->rigid_bodies->mesh.vertices, indices);

	std::vector<symx::Vector> x1;
	for (const symx::Vector& x_loc_a : x_loc) {
		x1.push_back(local_to_global_point(x_loc_a, t1, R1));
	}
	return x1;
}
