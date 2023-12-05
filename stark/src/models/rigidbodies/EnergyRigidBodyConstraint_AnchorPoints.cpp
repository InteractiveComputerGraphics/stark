#include "EnergyRigidBodyConstraint_AnchorPoints.h"

#include "rigid_body_constraint_utils.h"

stark::models::EnergyRigidBodyConstraint_AnchorPoints::EnergyRigidBodyConstraint_AnchorPoints(const spRigidBodyDynamics dyn, std::shared_ptr<core::Logger> rb_logger)
	: dyn(dyn), rb_logger(rb_logger)
{
}

int stark::models::EnergyRigidBodyConstraint_AnchorPoints::add(int rb, const Eigen::Vector3d& loc, const Eigen::Vector3d& target_glob, double stiffness, double tolerance)
{
	const int id = this->conn.numbered_push_back({ rb });
	this->loc.push_back(loc);
	this->target_glob.push_back(target_glob);
	this->stiffness.push_back(stiffness);
	this->tolerance.push_back(tolerance);
	this->is_active.push_back(1.0);
	this->labels.push_back("");
	return id;
}

std::array<double, 2> stark::models::EnergyRigidBodyConstraint_AnchorPoints::get_violation_and_force(int idx)
{
	const Eigen::Vector3d p = this->dyn->get_x1(this->conn[idx][1], this->loc[idx], dt);
	const Eigen::Vector3d target = this->target_glob[idx];
	const double C = inf_norm(p - target);

}

bool stark::models::EnergyRigidBodyConstraint_AnchorPoints::_adjust_constraints_stiffness(core::Stark& stark, double cap, double multiplier, bool log)
{
	const double dt = stark.settings.simulation.adaptive_time_step.value;

	for (int i = 0; i < (int)this->conn.size(); i++) {
		auto [idx, a] = this->conn[i];
		const Eigen::Vector3d p = this->dyn->get_x1(a, this->loc[idx], dt);
		const Eigen::Vector3d target = this->target_glob[idx];
		const double C = inf_norm(p - target);
		const double tol = this->tolerance[idx];

		if (log) {
			const double f = this->stiffness[idx] * std::abs(C);
			log_parameters(this->rb_logger, "anchor_point", idx, this->labels[idx], 
				"violation [m]", C, 
				"stiffness [N/m]", this->stiffness[idx], 
				"force [N]", f, 
				"tolerance [m]", tol);
		}

		if (C > tol) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}
}
