#include "EnergyRigidBodyConstraints.h"

#include <fmt/format.h>

#include "rigidbody_transformations.h"

stark::EnergyRigidBodyConstraints::EnergyRigidBodyConstraints(stark::core::Stark& stark, spRigidBodyDynamics rb)
	: rb(rb)
{
	// Callbacks
	stark.callbacks.add_is_converged_state_valid([&]() { return this->_is_converged_state_valid(stark); });
	stark.callbacks.add_on_time_step_accepted([&]() { this->_on_time_step_accepted(stark); });
	stark.callbacks.add_write_frame([&]() { this->_write_frame(stark); });

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
	stark.global_energy.add_energy("rb_constraint_global_points", this->global_points->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->global_points;

			symx::Vector loc = energy.make_vector(data->loc, conn["idx"]);
			symx::Vector target_glob = energy.make_vector(data->target_glob, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector glob = this->rb->get_x1(energy, conn["rb"], loc, dt);
			symx::Scalar E = RigidBodyConstraints::GlobalPoints::energy(stiffness, target_glob, glob);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_global_directions", this->global_directions->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->global_directions;

			symx::Vector d_loc = energy.make_vector(data->d_loc, conn["idx"]);
			symx::Vector target_d_glob = energy.make_vector(data->target_d_glob, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector d_glob = this->rb->get_d1(energy, conn["rb"], d_loc, dt);
			symx::Scalar E = RigidBodyConstraints::GlobalDirections::energy(stiffness, target_d_glob, d_glob);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_points", this->points->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->points;

			symx::Vector a_loc = energy.make_vector(data->a_loc, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(data->b_loc, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector a1 = this->rb->get_x1(energy, conn["a"], a_loc, dt);
			symx::Vector b1 = this->rb->get_x1(energy, conn["b"], b_loc, dt);
			symx::Scalar E = RigidBodyConstraints::Points::energy(stiffness, a1, b1);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_point_on_axis", this->point_on_axes->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->point_on_axes;

			symx::Vector a_loc = energy.make_vector(data->a_loc, conn["idx"]);
			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(data->b_loc, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			auto [a1, da1] = this->rb->get_x1_d1(energy, conn["a"], a_loc, da_loc, dt);
			symx::Vector b1 = this->rb->get_x1(energy, conn["b"], b_loc, dt);
			symx::Scalar E = RigidBodyConstraints::PointOnAxes::energy(stiffness, a1, da1, b1);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_distances", this->distances->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->distances;

			symx::Vector a_loc = energy.make_vector(data->a_loc, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(data->b_loc, conn["idx"]);
			symx::Scalar target_distance = energy.make_scalar(data->target_distance, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector a1 = this->rb->get_x1(energy, conn["a"], a_loc, dt);
			symx::Vector b1 = this->rb->get_x1(energy, conn["b"], b_loc, dt);
			symx::Scalar E = RigidBodyConstraints::Distance::energy(stiffness, a1, b1, target_distance);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_distance_limits", this->distance_limits->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->distance_limits;

			symx::Vector a_loc = energy.make_vector(data->a_loc, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(data->b_loc, conn["idx"]);
			symx::Scalar min_distance = energy.make_scalar(data->min_distance, conn["idx"]);
			symx::Scalar max_distance = energy.make_scalar(data->max_distance, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector a1 = this->rb->get_x1(energy, conn["a"], a_loc, dt);
			symx::Vector b1 = this->rb->get_x1(energy, conn["b"], b_loc, dt);
			symx::Scalar E = RigidBodyConstraints::DistanceLimits::energy(stiffness, a1, b1, min_distance, max_distance);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_directions", this->directions->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->directions;

			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Vector db_loc = energy.make_vector(data->db_loc, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector da = this->rb->get_d1(energy, conn["a"], da_loc, dt);
			symx::Vector db = this->rb->get_d1(energy, conn["b"], db_loc, dt);
			symx::Scalar E = RigidBodyConstraints::Directions::energy(stiffness, da, db);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_angle_limits", this->angle_limits->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->angle_limits;

			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Vector db_loc = energy.make_vector(data->db_loc, conn["idx"]);
			symx::Scalar max_distance = energy.make_scalar(data->max_distance, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector da1 = this->rb->get_d1(energy, conn["a"], da_loc, dt);
			symx::Vector db1 = this->rb->get_d1(energy, conn["b"], db_loc, dt);
			symx::Scalar E = RigidBodyConstraints::AngleLimits::energy(stiffness, da1, db1, max_distance);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_damped_spring", this->damped_springs->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->damped_springs;

			symx::Vector a_loc = energy.make_vector(data->a_loc, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(data->b_loc, conn["idx"]);
			symx::Scalar rest_length = energy.make_scalar(data->rest_length, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar damping = energy.make_scalar(data->damping, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			auto [a0, a1] = this->rb->get_x0_x1(energy, conn["a"], a_loc, dt);
			auto [b0, b1] = this->rb->get_x0_x1(energy, conn["b"], b_loc, dt);

			symx::Scalar E = RigidBodyConstraints::DampedSprings::energy(stiffness, damping, a0, a1, b0, b1, rest_length, dt);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_linear_velocity", this->linear_velocity->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->linear_velocity;

			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Scalar target_v = energy.make_scalar(data->target_v, conn["idx"]);
			symx::Scalar max_force = energy.make_scalar(data->max_force, conn["idx"]);
			symx::Scalar delay = energy.make_scalar(data->delay, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Vector va1 = energy.make_dof_vector(this->rb->dof_v, this->rb->v1, conn["a"]);
			symx::Vector vb1 = energy.make_dof_vector(this->rb->dof_v, this->rb->v1, conn["b"]);
			symx::Vector wa1 = energy.make_dof_vector(this->rb->dof_w, this->rb->w1, conn["a"]);
			symx::Vector qa0 = energy.make_vector(this->rb->q0_, conn["a"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector da1 = integrate_loc_direction(da_loc, qa0, wa1, dt);
			symx::Scalar E = RigidBodyConstraints::LinearVelocity::energy(da1, va1, vb1, target_v, max_force, delay, dt);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_angular_velocity", this->angular_velocity->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->angular_velocity;

			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Scalar target_w = energy.make_scalar(data->target_w, conn["idx"]);
			symx::Scalar max_torque = energy.make_scalar(data->max_torque, conn["idx"]);
			symx::Scalar delay = energy.make_scalar(data->delay, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Vector wa1 = energy.make_dof_vector(this->rb->dof_w, this->rb->w1, conn["a"]);
			symx::Vector wb1 = energy.make_dof_vector(this->rb->dof_w, this->rb->w1, conn["b"]);
			symx::Vector qa0 = energy.make_vector(this->rb->q0_, conn["a"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			symx::Vector da1 = integrate_loc_direction(da_loc, qa0, wa1, dt);
			symx::Scalar E = RigidBodyConstraints::AngularVelocity::energy(da1, wa1, wb1, target_w, max_torque, delay, dt);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);
}

bool stark::EnergyRigidBodyConstraints::_is_converged_state_valid(core::Stark& stark)
{
	/*
		Hardens every constraints that has gone beyond the input tolerance.
		If no constraint needs to be hardened, return true.
	*/
	const bool valid = this->_adjust_constraints_stiffness_and_log(stark, 1.0, this->stiffness_hard_multiplier, /*log = */ false, /* are_positions_set = */ false);
	if (!valid) {
		stark.console.add_error_msg("Rigid body constraints are not within tolerance. Hardening bending_stiffness.");
	}
	return valid;
}

void stark::EnergyRigidBodyConstraints::_on_time_step_accepted(core::Stark& stark)
{
	/*
	*	Logs the state of the constraints.
	*	Also, increases constraint stiffnesses at the end of a successful time step to preemtively adapt to harder conditions if occur smoothly.
	*	This is an easy and cheap way to avoid restarting future successful time steps due to predictable load increases.
	*	Adaptive soft decrease is not done as it would require a base bending_stiffness value which is added responsibility to the user.
	*	It's too easy to have an overly soft constraint parametrization that runs into force time restarts too frequently.
	*/
	this->_adjust_constraints_stiffness_and_log(stark, this->soft_constraint_capacity_hardening_point, this->stiffness_soft_multiplier, /* log = */ true, /* are_positions_set = */ true);
}

void stark::EnergyRigidBodyConstraints::_write_frame(core::Stark& stark)
{
	auto& output = stark.settings.output;
	this->logger.save_to_disk(fmt::format("{}/rigidbody_constraints_logger_{}__{}.txt", output.output_directory, output.simulation_name, output.time_stamp));
}

bool stark::EnergyRigidBodyConstraints::_adjust_constraints_stiffness_and_log(core::Stark& stark, double cap, double multiplier, bool log, bool are_positions_set)
{
	/*
		This function evaluates all the constraints and serves multiple purposes:
			- Hard increase of bending_stiffness within the newton step
			- Soft increase bending_stiffness at the end of a successful time step
			- Log the state of constraints
	*/

	const double dt = stark.dt;

	// Get correct points and directions according to `are_positions_set`
	auto get_x1 = [&](int rb, Eigen::Vector3d& loc) { return (are_positions_set) ? this->rb->get_position_at(rb, loc) : this->rb->get_x1(rb, loc, dt); };
	auto get_d1 = [&](int rb, Eigen::Vector3d& loc) { return (are_positions_set) ? this->rb->get_direction(rb, loc) : this->rb->get_d1(rb, loc, dt); };

	if (log) {
		this->logger.append_to_series("time [s]", stark.current_time);
		this->logger.append_to_series("frame", stark.current_frame);
	}

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
