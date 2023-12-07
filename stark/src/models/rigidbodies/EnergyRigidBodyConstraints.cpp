#include "EnergyRigidBodyConstraints.h"

#include <fmt/format.h>

#include "../../utils/mesh_utils.h"
#include "rigidbody_transformations.h"
#include "../distances.h"

// Helpers -----------------------------------------------------------------------
template<typename T, typename... Args>
void log_parameters(stark::core::Logger& logger, const std::string& constraint, const int idx, const std::string& constraint_name, const std::string& param_name, T&& value, Args&&... args) {
	logger.append_to_series(fmt::format("{} {:d} {} | {}", constraint, idx, constraint_name, param_name), fmt::format("{:.4e}", std::forward<T>(value)));
	log_parameters(logger, constraint, idx, constraint_name, std::forward<Args>(args)...);
}
//template<>
void log_parameters(stark::core::Logger& logger, const std::string& constraint, const int idx, const std::string& constraint_name) {
	// Base case for variadic template
}
// Helpers -----------------------------------------------------------------------



stark::models::EnergyRigidBodyConstraints::EnergyRigidBodyConstraints(stark::core::Stark& stark, spRigidBodyDynamics dyn)
	: dyn(dyn)
{
	// Callbacks
	stark.callbacks.is_converged_state_valid.push_back([&]() { return this->_is_converged_state_valid(stark); });
	stark.callbacks.on_time_step_accepted.push_back([&]() { this->_on_time_step_accepted(stark); });
	stark.callbacks.write_frame.push_back([&]() { this->_write_frame(stark); });

	// Constraint containers initialization
	this->global_points = std::make_shared<RigidBodyConstraints::GlobalPoints>();
	this->global_directions = std::make_shared<RigidBodyConstraints::GlobalDirections>();
	this->points = std::make_shared<RigidBodyConstraints::Points>();
	this->distances = std::make_shared<RigidBodyConstraints::Distance>();
	this->relative_direction_locks = std::make_shared<RigidBodyConstraints::RelativeDirectionLocks>();
	this->point_on_axis = std::make_shared<RigidBodyConstraints::PointOnAxis>();
	this->damped_springs = std::make_shared<RigidBodyConstraints::DampedSprings>();
	this->distance_limits = std::make_shared<RigidBodyConstraints::DistanceLimits>();
	this->angle_limits = std::make_shared<RigidBodyConstraints::AngleLimits>();
	this->relative_linear_velocity_motors = std::make_shared<RigidBodyConstraints::RelativeLinearVelocityMotors>();
	this->relative_angular_velocity_motors = std::make_shared<RigidBodyConstraints::RelativeAngularVelocityMotors>();

	// Energy declarations
	stark.global_energy.add_energy("rb_constraint_global_points", this->global_points->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->global_points;

			symx::Vector loc = energy.make_vector(data->loc, conn["idx"]);
			symx::Vector target_glob = energy.make_vector(data->target_glob, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			symx::Vector glob = this->dyn->get_x1(energy, conn["rb"], loc, dt);
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
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			symx::Vector d_glob = this->dyn->get_d1(energy, conn["rb"], d_loc, dt);
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
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			symx::Vector a1 = this->dyn->get_x1(energy, conn["a"], a_loc, dt);
			symx::Vector b1 = this->dyn->get_x1(energy, conn["b"], b_loc, dt);
			symx::Scalar E = RigidBodyConstraints::Points::energy(stiffness, a1, b1);
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
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			symx::Vector a1 = this->dyn->get_x1(energy, conn["a"], a_loc, dt);
			symx::Vector b1 = this->dyn->get_x1(energy, conn["b"], b_loc, dt);
			symx::Scalar E = RigidBodyConstraints::Distance::energy(stiffness, a1, b1, target_distance);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);








	stark.global_energy.add_energy("rb_constraint_ball_joint", this->ball_joints->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->ball_joints;

			symx::Vector a_loc = energy.make_vector(data->a_loc, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(data->b_loc, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);

			symx::Vector a1 = this->_get_x1(energy, stark, conn["a"], a_loc);
			symx::Vector b1 = this->_get_x1(energy, stark, conn["b"], b_loc);
			symx::Scalar E = 0.5 * stiffness * (a1 - b1).squared_norm();
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_relative_direction_lock", this->relative_direction_locks->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->relative_direction_locks;

			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Vector db_loc = energy.make_vector(data->db_loc, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);

			symx::Vector da1 = this->_get_d1(energy, stark, conn["a"], da_loc);
			symx::Vector db1 = this->_get_d1(energy, stark, conn["b"], db_loc);
			symx::Scalar E = 0.5 * stiffness * (da1 - db1).squared_norm();
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_point_on_axis", this->point_on_axis->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->point_on_axis;

			symx::Vector a_loc = energy.make_vector(data->a_loc, conn["idx"]);
			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(data->b_loc, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);

			auto [a1, da1] = this->_get_x1_d1(energy, stark, conn["a"], a_loc, da_loc);
			symx::Vector b1 = this->_get_x1(energy, stark, conn["b"], b_loc);
			symx::Scalar E = 0.5 * stiffness * sq_distance_point_line(b1, a1, a1 + da1);
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_angle_limits", this->angle_limits->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->angle_limits;

			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Vector db_loc = energy.make_vector(data->db_loc, conn["idx"]);
			symx::Scalar admissible_dot = energy.make_scalar(data->admissible_dot, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);

			symx::Vector da1 = this->_get_d1(energy, stark, conn["a"], da_loc);
			symx::Vector db1 = this->_get_d1(energy, stark, conn["b"], db_loc);
			symx::Scalar dot = da1.dot(db1);
			symx::Scalar C = admissible_dot - dot; // dot must be larger than admissible_dot

			symx::Scalar E = stiffness * symx::powN(C, 3) / 3.0;
			energy.set_with_condition(E, symx::min(C, is_active) > 0.0);  // both conditions must be positive
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
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			auto [a0, a1] = this->_get_x0_x1(energy, stark, conn["a"], a_loc);
			auto [b0, b1] = this->_get_x0_x1(energy, stark, conn["b"], b_loc);

			symx::Vector r1 = b1 - a1;
			symx::Vector r0 = b0 - a0;
			symx::Scalar l1 = r1.norm();
			symx::Scalar l0 = r0.norm();
			
			symx::Scalar E_spring = 0.5 * stiffness * (l1 / rest_length - 1.0).powN(2);
			symx::Scalar E_damper = 0.5 * damping * ((l1 - l0) / dt).powN(2);
			energy.set_with_condition(E_spring + E_damper, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_distance_limits", this->distance_limits->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->distance_limits;

			symx::Vector a_loc = energy.make_vector(data->a_loc, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(data->b_loc, conn["idx"]);
			symx::Scalar min_length = energy.make_scalar(data->min_length, conn["idx"]);
			symx::Scalar max_length = energy.make_scalar(data->max_length, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);

			symx::Vector a1 = this->_get_x1(energy, stark, conn["a"], a_loc);
			symx::Vector b1 = this->_get_x1(energy, stark, conn["b"], b_loc);
			symx::Scalar length = (a1 - b1).norm();

			symx::Scalar E_min = symx::branch(length < min_length, stiffness*symx::powN(min_length - length, 3)/3.0, 0.0);
			symx::Scalar E_max = symx::branch(length > max_length, stiffness*symx::powN(length - max_length, 3)/3.0, 0.0);
			energy.set_with_condition(E_min + E_max, symx::max(symx::max(E_min, E_max), is_active) > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_relative_linear_velocity_motors", this->relative_linear_velocity_motors->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->relative_linear_velocity_motors;

			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Scalar target_v = energy.make_scalar(data->target_v, conn["idx"]);
			symx::Scalar max_force = energy.make_scalar(data->max_force, conn["idx"]);
			symx::Scalar delay = energy.make_scalar(data->delay, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Vector va1 = energy.make_dof_vector(this->dyn->dof_v, this->dyn->v1, conn["a"]);
			symx::Vector vb1 = energy.make_dof_vector(this->dyn->dof_v, this->dyn->v1, conn["b"]);
			symx::Vector wa1 = energy.make_dof_vector(this->dyn->dof_w, this->dyn->w1, conn["a"]);
			symx::Vector qa0 = energy.make_vector(this->dyn->q0_, conn["a"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			symx::Vector da1 = integrate_loc_direction(da_loc, qa0, wa1, dt);
			symx::Scalar v = da1.dot(vb1 - va1);
			this->_set_c1_controller_energy(energy, v, target_v, max_force, delay, dt, is_active);
		}
	);

	stark.global_energy.add_energy("rb_constraint_relative_angular_velocity_motors", this->relative_angular_velocity_motors->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->relative_angular_velocity_motors;

			symx::Vector da_loc = energy.make_vector(data->da_loc, conn["idx"]);
			symx::Scalar target_w = energy.make_scalar(data->target_w, conn["idx"]);
			symx::Scalar max_torque = energy.make_scalar(data->max_torque, conn["idx"]);
			symx::Scalar delay = energy.make_scalar(data->delay, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);
			symx::Vector wa1 = energy.make_dof_vector(this->dyn->dof_w, this->dyn->w1, conn["a"]);
			symx::Vector wb1 = energy.make_dof_vector(this->dyn->dof_w, this->dyn->w1, conn["b"]);
			symx::Vector qa0 = energy.make_vector(this->dyn->q0_, conn["a"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			symx::Vector da1 = integrate_loc_direction(da_loc, qa0, wa1, dt);
			symx::Scalar w = da1.dot(wb1 - wa1);
			this->_set_c1_controller_energy(energy, w, target_w, max_torque, delay, dt, is_active);
		}
	);
}

void stark::models::EnergyRigidBodyConstraints::_set_c1_controller_energy(symx::Energy& energy, const symx::Scalar& v, const symx::Scalar& target_v, const symx::Scalar& max_force, const symx::Scalar& delay, const symx::Scalar& dt, const symx::Scalar& is_active)
{
	// Constraint (Analogous to C1 friction)
	// Important: derivatives wrt "positions", therefore needed chain rule and resulted in added product by dt
	symx::Scalar k = max_force / delay;
	symx::Scalar eps = max_force / (2.0 * k);
	symx::Scalar dv = symx::sqrt((target_v - v).powN(2));
	symx::Scalar E_l = 0.5 * k * dv.powN(2) * dt;
	symx::Scalar E_r = max_force * (dv - eps) * dt;
	symx::Scalar E = symx::branch(dv < delay, E_l, E_r);
	energy.set_with_condition(E, is_active > 0.0);
}

bool stark::models::EnergyRigidBodyConstraints::_is_converged_state_valid(core::Stark& stark)
{
	/*
		Hardens every constraints that has gone beyond the input tolerance.
		If no constraint needs to be hardened, return true.
	*/
	return this->_adjust_constraints_stiffness(stark, 1.0, this->stiffness_hard_multiplier, /*log = */ false);
}

void stark::models::EnergyRigidBodyConstraints::_on_time_step_accepted(core::Stark& stark)
{
	/*
	*	Logs the state of the constraints.
	*	Also, increases constraint stiffnesses at the end of a successful time step to preemtively adapt to harder conditions if occur smoothly.
	*	This is an easy and cheap way to avoid restarting future successful time steps due to predictable load increases.
	*	Adaptive soft decrease is not done as it would require a base stiffness value which is added responsibility to the user.
	*	It's too easy to have an overly soft constraint parametrization that runs into force time restarts too frequently.
	*/
	this->_adjust_constraints_stiffness(stark, this->soft_constraint_capacity_hardening_point, this->stiffness_soft_multiplier, /* log = */ true);
}

void stark::models::EnergyRigidBodyConstraints::_write_frame(core::Stark& stark)
{
	auto& output = stark.settings.output;
	this->logger.save_to_disk(fmt::format("{}/rigidbody_constraints_logger_{}__{}.txt", output.output_directory, output.simulation_name, output.time_stamp));
}

bool stark::models::EnergyRigidBodyConstraints::_adjust_constraints_stiffness(core::Stark& stark, double cap, double multiplier, bool log)
{
	// Note: Equality vector constraints use component-wise enforcement. We check for inf_norm to detect if any component of the constraint has been violated.

	const double dt = stark.settings.simulation.adaptive_time_step.value;
	bool is_valid = true;

	if (log) {
		this->logger.append_to_series("time [s]", stark.current_time);
		this->logger.append_to_series("frame", stark.current_frame);
	}

	// Global Points
	for (int i = 0; i < (int)this->global_points->conn.size(); i++) {
		auto& data = this->global_points;
		auto [idx, a] = data->conn[i];
		const Eigen::Vector3d p = this->dyn->get_x1(a, data->loc[idx], dt);
		auto [C, f] = RigidBodyConstraints::GlobalPoints::violation_in_m_and_force(data->stiffness[idx], data->target_glob[idx], p);

		if (log) {
			log_parameters(this->logger, "absolute_position", idx, data->labels[idx], 
				"violation [m]", C, 
				"stiffness [N/m]", data->stiffness[idx], 
				"force [N]", f.norm(), 
				"force_x [N]", f[0], 
				"force_y [N]", f[1], 
				"force_z [N]", f[2], 
				"tolerance [m]", data->tolerance_in_m[idx],
				"is_active", data->is_active[idx]);
		}

		if (C > data->tolerance_in_m[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Global Directions
	for (int i = 0; i < (int)this->global_directions->conn.size(); i++) {
		auto& data = this->global_directions;
		auto [idx, a] = data->conn[i];
		const Eigen::Vector3d d = this->dyn->get_d1(a, data->d_loc[idx], dt);
		auto [C, t] = RigidBodyConstraints::GlobalDirections::violation_in_deg_and_torque(data->stiffness[idx], data->target_d_glob[idx], d);

		if (log) {
			log_parameters(this->logger, "absolute_direction", idx, data->labels[idx],
				"violation [m]", C,
				"stiffness [N/m]", data->stiffness[idx],
				"torque [Nm]", t.norm(),
				"torque_x [Nm]", t[0],
				"torque_y [Nm]", t[1],
				"torque_z [Nm]", t[2],
				"tolerance [deg]", data->tolerance_in_deg[idx],
				"is_active", data->is_active[idx]);
		}

		if (C > data->tolerance_in_deg[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Points
	for (int i = 0; i < (int)this->points->conn.size(); i++) {
		auto& data = this->points;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d a1 = this->dyn->get_x1(a, data->a_loc[idx], dt);
		const Eigen::Vector3d b1 = this->dyn->get_x1(b, data->b_loc[idx], dt);
		auto [C, f] = RigidBodyConstraints::Points::violation_in_m_and_force(data->stiffness[idx], a1, b1);

		if (log) {
			log_parameters(this->logger, "absolute_position", idx, data->labels[idx],
				"violation [m]", C,
				"stiffness [N/m]", data->stiffness[idx],
				"force [N]", f.norm(),
				"force_x [N]", f[0],
				"force_y [N]", f[1],
				"force_z [N]", f[2],
				"tolerance [m]", data->tolerance_in_m[idx],
				"is_active", data->is_active[idx]);
		}

		if (C > data->tolerance_in_m[idx]) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Relative Direction Locks
	for (int i = 0; i < (int)this->relative_direction_locks->conn.size(); i++) {
		auto& data = this->relative_direction_locks;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d da1 = this->_get_d1(a, data->da_loc[idx], dt);
		const Eigen::Vector3d db1 = this->_get_d1(b, data->db_loc[idx], dt);
		const double C = inf_norm(da1 - db1);
		const double tol = data->tolerance[idx];

		if (log) {
			const double f = data->stiffness[idx] * std::abs(C);  // force = torque in this case as the leverage is 1m
			const double angle_deg = RigidBodyConstraints::get_angle_deg_from_distance_violation(C);
			const double tol_deg = RigidBodyConstraints::get_angle_deg_from_distance_violation(tol);
			log_parameters(this->logger, "relative_direction_lock", idx, data->labels[idx], "violation [deg]", angle_deg, "stiffness [N/m2]", data->stiffness[idx], "torque [Nm]", f, "tolerance [deg]", tol_deg);
		}

		if (C > tol) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Point On Axis
	for (int i = 0; i < (int)this->point_on_axis->conn.size(); i++) {
		auto& data = this->point_on_axis;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d a1 = this->_get_x1(a, data->a_loc[idx], dt);
		const Eigen::Vector3d da1 = this->_get_d1(a, data->da_loc[idx], dt);
		const Eigen::Vector3d b1 = this->_get_x1(b, data->b_loc[idx], dt);
		const double C = distance_point_line(b1, a1, a1 + da1);
		const double tol = data->tolerance[idx];

		if (log) {
			const double f = data->stiffness[idx] * std::abs(C);
			log_parameters(this->logger, "point_on_axis", idx, data->labels[idx], "violation [m]", C, "stiffness [N/m]", data->stiffness[idx], "force [N]", f, "tolerance [m]", tol);
		}

		if (C > tol) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Distance Limits
	for (int i = 0; i < (int)this->distance_limits->conn.size(); i++) {
		auto& data = this->distance_limits;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d a1 = this->_get_x1(a, data->a_loc[idx], dt);
		const Eigen::Vector3d b1 = this->_get_x1(b, data->b_loc[idx], dt);
		const double& min_distance = data->min_length[idx];
		const double& max_distance = data->max_length[idx];
		const double d = (a1 - b1).norm();
		const double tol = data->tolerance[idx];

		if (log) {
			double C = 0.0;
			if (d < min_distance) {
				C = min_distance - d;
			}
			else if (d > max_distance) {
				C = d - max_distance;
			}
			const double f = data->stiffness[idx] * std::pow(C, 2);
			log_parameters(this->logger, "distance_limit", idx, data->labels[idx], "violation [m]", C, "stiffness [N/m]", data->stiffness[idx], "force [N]", f, "tolerance [m]", tol);
		}

		if (d < (min_distance - tol) || (max_distance + tol) < d) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}
	}

	// Angle Limits
	for (int i = 0; i < (int)this->angle_limits->conn.size(); i++) {
		auto& data = this->angle_limits;
		auto [idx, a, b] = data->conn[i];
		const Eigen::Vector3d da1 = this->_get_d1(a, data->da_loc[idx], dt);
		const Eigen::Vector3d db1 = this->_get_d1(b, data->db_loc[idx], dt);
		const double& admissible_dot = data->admissible_dot[idx];
		const double dot = da1.dot(db1);

		const double tol = data->tolerance[idx];
		if (dot < admissible_dot - tol) {
			is_valid = false;
			data->stiffness[idx] *= multiplier;
		}

		std::cout << "Angle limits not implemented correctly. Waiting for a refactor." << std::endl;
		exit(-1);

		//if (log) {
		//	const double C = std::max(0.0, angle_deg - admissible_angle_deg);
		//	const double f = data->stiffness[idx] * std::pow(C, 2);
		//	const double angle_deg = RigidBodyConstraints::get_angle_deg_from_dot(dot);
		//	const double tol_deg = RigidBodyConstraints::get_angle_deg_from_dot(tol);
		//	log_parameters(this->logger, "distance_limit", idx, data->labels[idx], "violation [m]", C, "stiffness [N/m]", data->stiffness[idx], "force [N]", f, "tolerance [m]", tol);
		//}

		//if (dot < admissible_dot - tol) {
		//	is_valid = false;
		//	data->stiffness[idx] *= multiplier;
		//}
	}


	// TODO: Log motors


	return is_valid;
}
