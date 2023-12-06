#include "EnergyRigidBodyConstraint_AnchorPoints.h"

#include "rigid_body_constraint_utils.h"

stark::models::EnergyRigidBodyConstraint_AnchorPoints::EnergyRigidBodyConstraint_AnchorPoints(core::Stark& stark, const spRigidBodyDynamics dyn, std::shared_ptr<core::Logger> rb_logger)
	: dyn(dyn), rb_logger(rb_logger)
{
	// Callbacks
	stark.callbacks.is_converged_state_valid.push_back([&]() { return this->_is_converged_state_valid(stark); });
	stark.callbacks.on_time_step_accepted.push_back([&]() { this->_on_time_step_accepted(stark); });

	// Energy
	/*
	*	A point on a rigid body is constrained to be at a certain global position.
	*	This is enforced by a penalty to the norm of the distance.
	*	This is equivalent to 3 individual distance penalties, one per dimension.
	*/
	stark.global_energy.add_energy("rb_constraint_anchor_point", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector loc = energy.make_vector(this->loc, conn["idx"]);
			symx::Vector target_glob = energy.make_vector(this->target_glob, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(this->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(this->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			symx::Vector glob = this->dyn->get_x1(energy, conn["rb"], loc, dt);
			symx::Scalar E = 0.5 * stiffness * (target_glob - glob).squared_norm();  // E = 0.5*k*u.norm()**2  ;  f = k*u.norm()
			energy.set_with_condition(E, is_active > 0.0);
		}
	);
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
	const Eigen::Vector3d p = this->dyn->get_x1(this->conn[idx][1], this->loc[idx]);
	const Eigen::Vector3d target = this->target_glob[idx];
	const Eigen::Vector3d u = target - p;
	const double C = u.norm();
	const double f = this->stiffness[idx] * C;
	return {C, f};
}

bool stark::models::EnergyRigidBodyConstraint_AnchorPoints::_adjust_constraints_stiffness_and_log(core::Stark& stark, double cap, double multiplier, bool log)
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

bool stark::models::EnergyRigidBodyConstraint_AnchorPoints::_is_converged_state_valid(core::Stark& stark)
{
	/*
		Hardens every constraints that has gone beyond the input tolerance.
		If no constraint needs to be hardened, return true.
	*/
	//constexpr double hard_multiplier = 2.0;
	return this->_adjust_constraints_stiffness_and_log(stark, 1.0, common_params.hard_multiplier, /* log = */ false);
}
void stark::models::EnergyRigidBodyConstraint_AnchorPoints::_on_time_step_accepted(core::Stark& stark)
{
	/*
	*	Logs the state of the constraints.
	*	Also, increases constraint stiffnesses at the end of a successful time step to preemtively adapt to harder conditions if occur smoothly.
	*	This is an easy and cheap way to avoid restarting future successful time steps due to predictable load increases.
	*	Adaptive soft decrease is not done as it would require a base stiffness value which is added responsibility to the user.
	*	It's too easy to have an overly soft constraint parametrization that runs into force time restarts too frequently.
	*/
	//const double soft_multiplier = 1.05;
	//const double constraint_capacity = 0.75;
	this->_adjust_constraints_stiffness_and_log(stark, common_params.constraint_capacity, common_params.soft_multiplier, /*log = */ true);
}
