#include "PointDynamics.h"

stark::models::PointDynamics::PointDynamics(Stark& sim)
{
	this->dof = sim.global_energy.add_dof_array(this->v1.data, "PointDynamics.v1");
}
