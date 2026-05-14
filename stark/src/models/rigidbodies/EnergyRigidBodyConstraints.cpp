#include "EnergyRigidBodyConstraints.h"

#include <fmt/format.h>

#include "rigidbody_transformations.h"

using namespace symx;

stark::EnergyRigidBodyConstraints::EnergyRigidBodyConstraints(stark::core::Stark& stark, spRigidBodyDynamics rb)
	: rb(rb)
{
	// Callbacks
	stark.callbacks->newton->add_is_converged_state_valid([&]() { return this->_is_converged_state_valid(stark); });
	stark.callbacks->add_on_time_step_accepted([&]() { this->_on_time_step_accepted(stark); });

	// Constraint containers initialization
	this->global_points = std::make_shared<RigidBodyConstraints::GlobalPoints>();
	this->global_directions = std::make_shared<RigidBodyConstraints::GlobalDirections>();
	this->points = std::make_shared<RigidBodyConstraints::Points>();
	this->point_on_axes = std::make_shared<RigidBodyConstraints::PointOnAxes>();
	this->distances = std::make_shared<RigidBodyConstraints::Distance>();
	this->distance_limits = std::make_shared<RigidBodyConstraints::DistanceLimits>();
	this->directions = std::make_shared<RigidBodyConstraints::Directions>();
	this->angle_limits = std::make_shared<RigidBodyConstraints::AngleLimits>();
	this->damped_springs = std::make_shared<RigidBodyConstraints::DampedSprings>();
	this->linear_velocity = std::make_shared<RigidBodyConstraints::LinearVelocity>();
	this->angular_velocity = std::make_shared<RigidBodyConstraints::AngularVelocity>();

	// Energy declarations
	stark.global_potential->add_potential("rb_constraint_global_points", this->global_points->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->global_points;

			Vector loc = mws.make_vector(data->loc, conn["idx"]);
			Vector target_glob = mws.make_vector(data->target_glob, conn["idx"]);
			Scalar stiffness = mws.make_scalar(data->stiffness, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector glob = this->rb->get_x1(mws, conn["rb"], loc, dt);
			Scalar E = RigidBodyConstraints::GlobalPoints::energy(stiffness, target_glob, glob);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_global_directions", this->global_directions->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->global_directions;

			Vector d_loc = mws.make_vector(data->d_loc, conn["idx"]);
			Vector target_d_glob = mws.make_vector(data->target_d_glob, conn["idx"]);
			Scalar stiffness = mws.make_scalar(data->stiffness, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector d_glob = this->rb->get_d1(mws, conn["rb"], d_loc, dt);
			Scalar E = RigidBodyConstraints::GlobalDirections::energy(stiffness, target_d_glob, d_glob);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_points", this->points->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->points;

			Vector a_loc = mws.make_vector(data->a_loc, conn["idx"]);
			Vector b_loc = mws.make_vector(data->b_loc, conn["idx"]);
			Scalar stiffness = mws.make_scalar(data->stiffness, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector a1 = this->rb->get_x1(mws, conn["a"], a_loc, dt);
			Vector b1 = this->rb->get_x1(mws, conn["b"], b_loc, dt);
			Scalar E = RigidBodyConstraints::Points::energy(stiffness, a1, b1);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_point_on_axis", this->point_on_axes->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->point_on_axes;

			Vector a_loc = mws.make_vector(data->a_loc, conn["idx"]);
			Vector da_loc = mws.make_vector(data->da_loc, conn["idx"]);
			Vector b_loc = mws.make_vector(data->b_loc, conn["idx"]);
			Scalar stiffness = mws.make_scalar(data->stiffness, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Scalar dt = mws.make_scalar(stark.dt);

			auto [a1, da1] = this->rb->get_x1_d1(mws, conn["a"], a_loc, da_loc, dt);
			Vector b1 = this->rb->get_x1(mws, conn["b"], b_loc, dt);
			Scalar E = RigidBodyConstraints::PointOnAxes::energy(stiffness, a1, da1, b1);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_distances", this->distances->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->distances;

			Vector a_loc = mws.make_vector(data->a_loc, conn["idx"]);
			Vector b_loc = mws.make_vector(data->b_loc, conn["idx"]);
			Scalar target_distance = mws.make_scalar(data->target_distance, conn["idx"]);
			Scalar stiffness = mws.make_scalar(data->stiffness, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector a1 = this->rb->get_x1(mws, conn["a"], a_loc, dt);
			Vector b1 = this->rb->get_x1(mws, conn["b"], b_loc, dt);
			Scalar E = RigidBodyConstraints::Distance::energy(stiffness, a1, b1, target_distance);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_distance_limits", this->distance_limits->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->distance_limits;

			Vector a_loc = mws.make_vector(data->a_loc, conn["idx"]);
			Vector b_loc = mws.make_vector(data->b_loc, conn["idx"]);
			Scalar min_distance = mws.make_scalar(data->min_distance, conn["idx"]);
			Scalar max_distance = mws.make_scalar(data->max_distance, conn["idx"]);
			Scalar stiffness = mws.make_scalar(data->stiffness, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector a1 = this->rb->get_x1(mws, conn["a"], a_loc, dt);
			Vector b1 = this->rb->get_x1(mws, conn["b"], b_loc, dt);
			Scalar E = RigidBodyConstraints::DistanceLimits::energy(stiffness, a1, b1, min_distance, max_distance);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_directions", this->directions->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->directions;

			Vector da_loc = mws.make_vector(data->da_loc, conn["idx"]);
			Vector db_loc = mws.make_vector(data->db_loc, conn["idx"]);
			Scalar stiffness = mws.make_scalar(data->stiffness, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector da = this->rb->get_d1(mws, conn["a"], da_loc, dt);
			Vector db = this->rb->get_d1(mws, conn["b"], db_loc, dt);
			Scalar E = RigidBodyConstraints::Directions::energy(stiffness, da, db);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_angle_limits", this->angle_limits->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->angle_limits;

			Vector da_loc = mws.make_vector(data->da_loc, conn["idx"]);
			Vector db_loc = mws.make_vector(data->db_loc, conn["idx"]);
			Scalar max_distance = mws.make_scalar(data->max_distance, conn["idx"]);
			Scalar stiffness = mws.make_scalar(data->stiffness, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector da1 = this->rb->get_d1(mws, conn["a"], da_loc, dt);
			Vector db1 = this->rb->get_d1(mws, conn["b"], db_loc, dt);
			Scalar E = RigidBodyConstraints::AngleLimits::energy(stiffness, da1, db1, max_distance);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_damped_spring", this->damped_springs->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->damped_springs;

			Vector a_loc = mws.make_vector(data->a_loc, conn["idx"]);
			Vector b_loc = mws.make_vector(data->b_loc, conn["idx"]);
			Scalar rest_length = mws.make_scalar(data->rest_length, conn["idx"]);
			Scalar stiffness = mws.make_scalar(data->stiffness, conn["idx"]);
			Scalar damping = mws.make_scalar(data->damping, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Scalar dt = mws.make_scalar(stark.dt);

			auto [a0, a1] = this->rb->get_x0_x1(mws, conn["a"], a_loc, dt);
			auto [b0, b1] = this->rb->get_x0_x1(mws, conn["b"], b_loc, dt);

			Scalar E = RigidBodyConstraints::DampedSprings::energy(stiffness, damping, a0, a1, b0, b1, rest_length, dt);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_linear_velocity", this->linear_velocity->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->linear_velocity;

			Vector da_loc = mws.make_vector(data->da_loc, conn["idx"]);
			Scalar target_v = mws.make_scalar(data->target_v, conn["idx"]);
			Scalar max_force = mws.make_scalar(data->max_force, conn["idx"]);
			Scalar delay = mws.make_scalar(data->delay, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Vector va1 = mws.make_vector(this->rb->v1, conn["a"]);
			Vector vb1 = mws.make_vector(this->rb->v1, conn["b"]);
			Vector wa1 = mws.make_vector(this->rb->w1, conn["a"]);
			Vector qa0 = mws.make_vector(this->rb->q0_, conn["a"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector da1 = integrate_loc_direction(da_loc, qa0, wa1, dt);
			Scalar E = RigidBodyConstraints::LinearVelocity::energy(da1, va1, vb1, target_v, max_force, delay, dt);
			return std::pair(E, is_active > 0.0);
		}
	);

	stark.global_potential->add_potential("rb_constraint_angular_velocity", this->angular_velocity->conn,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			auto& data = this->angular_velocity;

			Vector da_loc = mws.make_vector(data->da_loc, conn["idx"]);
			Scalar target_w = mws.make_scalar(data->target_w, conn["idx"]);
			Scalar max_torque = mws.make_scalar(data->max_torque, conn["idx"]);
			Scalar delay = mws.make_scalar(data->delay, conn["idx"]);
			Scalar is_active = mws.make_scalar(data->is_active, conn["idx"]);
			Vector wa1 = mws.make_vector(this->rb->w1, conn["a"]);
			Vector wb1 = mws.make_vector(this->rb->w1, conn["b"]);
			Vector qa0 = mws.make_vector(this->rb->q0_, conn["a"]);
			Scalar dt = mws.make_scalar(stark.dt);

			Vector da1 = integrate_loc_direction(da_loc, qa0, wa1, dt);
			Scalar E = RigidBodyConstraints::AngularVelocity::energy(da1, wa1, wb1, target_w, max_torque, delay, dt);
			return std::pair(E, is_active > 0.0);
		}
	);
}

bool stark::EnergyRigidBodyConstraints::_is_converged_state_valid(core::Stark& stark)
{
	/*
		Hardens every constraints that has gone beyond the input tolerance.
		If no constraint needs to be hardened, return true.
	*/
	const bool valid = this->_adjust_constraints_stiffness_and_log(stark, 1.0, this->stiffness_hard_multiplier, /* are_positions_set = */ false);
	if (!valid) {
		stark.context->output->print("Rigid body constraints are not within tolerance. Hardening constraint stiffness.\n", symx::Verbosity::Summary);
	}
	return valid;
}

void stark::EnergyRigidBodyConstraints::_on_time_step_accepted(core::Stark& stark)
{
	/*
	*	Logs the state of the constraints.
	*	Also, increases constraint stiffnesses at the end of a successful time step to preemtively adapt to harder conditions if occur smoothly.
	*	This is an easy and cheap way to avoid restarting future successful time steps due to predictable load increases.
	*	Adaptive soft decrease is not done as it would require a base constraint stiffness.\n", symx::Verbosity::Summary);
	} value which is added responsibility to the user.
	*	It's too easy to have an overly soft constraint parametrization that runs into force time restarts too frequently.
	*/
	this->_adjust_constraints_stiffness_and_log(stark, this->soft_constraint_capacity_hardening_point, this->stiffness_soft_multiplier, /* are_positions_set = */ true);
}

bool stark::EnergyRigidBodyConstraints::_adjust_constraints_stiffness_and_log(core::Stark& stark, double cap, double multiplier, bool are_positions_set)
{
	/*
		This function evaluates all the constraints and serves multiple purposes:
			- Hard increase of constraint stiffness within the newton step
			- Soft increase constraint stiffness at the end of a successful time step
			- Log the state of constraints
	*/

	const double dt = stark.dt;

	// Get correct points and directions according to `are_positions_set`
	auto get_x1 = [&](int rb, Eigen::Vector3d& loc) { return (are_positions_set) ? this->rb->get_position_at(rb, loc) : this->rb->get_x1(rb, loc, dt); };
	auto get_d1 = [&](int rb, Eigen::Vector3d& loc) { return (are_positions_set) ? this->rb->get_direction(rb, loc) : this->rb->get_d1(rb, loc, dt); };

	// Set true and make false if a constraint is violated beyond tolerance
	bool is_valid = true;

	// Global Points
	for (int i = 0; i < (int)this->global_points->conn.size(); i++) {
		auto& data = this->global_points;
		auto [idx, a] = data->conn[i];
		const Eigen::Vector3d p = get_x1(a, data->loc[idx]);
		auto [C, f] = RigidBodyConstraints::GlobalPoints::violation_in_m_and_force(data->stiffness[idx], data->target_glob[idx], p);

		if (C > data->tolerance_in_m[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Global Directions
	for (int i = 0; i < (int)this->global_directions->conn.size(); i++) {
		auto& data = this->global_directions;
		auto [idx, a] = data->conn[i];
		const Eigen::Vector3d d = get_d1(a, data->d_loc[idx]);
		auto [C, t] = RigidBodyConstraints::GlobalDirections::violation_in_deg_and_torque(data->stiffness[idx], data->target_d_glob[idx], d);

		if (C > data->tolerance_in_deg[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Points
	for (int i = 0; i < (int)this->points->conn.size(); i++) {
		auto& data = this->points;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d a1 = get_x1(a, data->a_loc[idx]);
		const Eigen::Vector3d b1 = get_x1(b, data->b_loc[idx]);
		auto [C, f] = RigidBodyConstraints::Points::violation_in_m_and_force(data->stiffness[idx], a1, b1);

		if (C > data->tolerance_in_m[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// PointOnAxis
	for (int i = 0; i < (int)this->point_on_axes->conn.size(); i++) {
		auto& data = this->point_on_axes;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d a1 = get_x1(a, data->a_loc[idx]);
		const Eigen::Vector3d da1 = get_d1(a, data->da_loc[idx]);
		const Eigen::Vector3d b1 = get_x1(b, data->b_loc[idx]);
		auto [C, f] = RigidBodyConstraints::PointOnAxes::violation_in_m_and_force(data->stiffness[idx], a1, da1, b1);

		if (C > data->tolerance_in_m[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Distance
	for (int i = 0; i < (int)this->distances->conn.size(); i++) {
		auto& data = this->distances;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d a1 = get_x1(a, data->a_loc[idx]);
		const Eigen::Vector3d b1 = get_x1(b, data->b_loc[idx]);
		auto [C, f] = RigidBodyConstraints::Distance::signed_violation_in_m_and_force(data->stiffness[idx], a1, b1, data->target_distance[idx]);

		if (std::abs(C) > data->tolerance_in_m[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Distance Limits
	for (int i = 0; i < (int)this->distance_limits->conn.size(); i++) {
		auto& data = this->distance_limits;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d a1 = get_x1(a, data->a_loc[idx]);
		const Eigen::Vector3d b1 = get_x1(b, data->b_loc[idx]);
		auto [C, f] = RigidBodyConstraints::DistanceLimits::signed_violation_in_m_and_force(data->stiffness[idx], a1, b1, data->min_distance[idx], data->max_distance[idx]);

		if (std::abs(C) > data->tolerance_in_m[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Directions
	for (int i = 0; i < (int)this->directions->conn.size(); i++) {
		auto& data = this->directions;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d da = get_d1(a, data->da_loc[idx]);
		const Eigen::Vector3d db = get_d1(b, data->db_loc[idx]);
		auto [C, t] = RigidBodyConstraints::Directions::violation_in_deg_and_torque(data->stiffness[idx], da, db);

		if (C > data->tolerance_in_deg[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Angle Limits
	for (int i = 0; i < (int)this->angle_limits->conn.size(); i++) {
		auto& data = this->angle_limits;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d da = get_d1(a, data->da_loc[idx]);
		const Eigen::Vector3d db = get_d1(b, data->db_loc[idx]);
		auto [C, t] = RigidBodyConstraints::AngleLimits::violation_in_deg_and_torque(data->stiffness[idx], da, db, data->max_distance[idx]);

		if (C > data->tolerance_in_deg[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	return is_valid;
}
