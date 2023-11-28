#include "EnergyRigidBodyConstraints.h"

#include "rigidbody_transformations.h"
#include "distances.h"

stark::models::EnergyRigidBodyConstraints::EnergyRigidBodyConstraints(Stark& stark, spRigidBodyDynamics dyn)
	: dyn(dyn)
{
	// Constraint containers initialization
	this->anchor_points = std::make_shared<BaseRigidBodyConstraints::AnchorPoints>();
	this->absolute_direction_locks = std::make_shared<BaseRigidBodyConstraints::AbsoluteDirectionLocks>();
	this->ball_joints = std::make_shared<BaseRigidBodyConstraints::BallJoints>();
	this->relative_direction_locks = std::make_shared<BaseRigidBodyConstraints::RelativeDirectionLocks>();
	this->point_on_axis = std::make_shared<BaseRigidBodyConstraints::PointOnAxis>();
	this->damped_springs = std::make_shared<BaseRigidBodyConstraints::DampedSprings>();
	this->distance_limits = std::make_shared<BaseRigidBodyConstraints::DistanceLimits>();
	this->angle_limits = std::make_shared<BaseRigidBodyConstraints::AngleLimits>();
	this->relative_linear_velocity_motors = std::make_shared<BaseRigidBodyConstraints::RelativeLinearVelocityMotors>();
	this->relative_angular_velocity_motors = std::make_shared<BaseRigidBodyConstraints::RelativeAngularVelocityMotors>();


	// Energy declarations
	stark.global_energy.add_energy("rb_constraint_anchor_point", this->anchor_points->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->anchor_points;

			symx::Vector loc = energy.make_vector(data->loc, conn["idx"]);
			symx::Vector target_glob = energy.make_vector(data->target_glob, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);

			symx::Vector glob = this->_get_x1(energy, stark, conn["rb"], loc);
			symx::Scalar E = 0.5 * stiffness * (target_glob - glob).squared_norm();
			energy.set_with_condition(E, is_active > 0.0);
		}
	);

	stark.global_energy.add_energy("rb_constraint_absolute_direction_lock", this->absolute_direction_locks->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto& data = this->absolute_direction_locks;

			symx::Vector d_loc = energy.make_vector(data->d_loc, conn["idx"]);
			symx::Vector target_d_glob = energy.make_vector(data->target_d_glob, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(data->stiffness, conn["idx"]);
			symx::Scalar is_active = energy.make_scalar(data->is_active, conn["idx"]);

			symx::Vector d_glob = this->_get_d1(energy, stark, conn["rb"], d_loc);
			symx::Scalar E = 0.5 * stiffness * (target_d_glob - d_glob).squared_norm();
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

			symx::Vector a_loc = energy.make_vector(data->da_loc, conn["idx"]);
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
			this->_set_c1_controller_energy(energy, v, target_v, max_force, delay, dt);
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
			this->_set_c1_controller_energy(energy, w, target_w, max_torque, delay, dt);
		}
	);



}


void stark::models::EnergyRigidBodyConstraints::_set_c1_controller_energy(symx::Energy& energy, const symx::Scalar& v, const symx::Scalar& target_v, const symx::Scalar& max_force, const symx::Scalar& delay, const symx::Scalar& dt)
{
	// Constraint (Analogous to C1 friction)
	// Important: derivatives wrt "positions", therefore needed chain rule and resulted in added product by dt
	symx::Scalar k = max_force / delay;
	symx::Scalar eps = max_force / (2.0 * k);
	symx::Scalar dw = target_v - v;
	symx::Scalar E_l = 0.5 * k * dw.powN(2) * dt;
	symx::Scalar E_r = max_force * (dw - eps) * dt;
	symx::Scalar E = symx::branch(dw < delay, E_l, E_r);
	energy.set_with_condition(E, dw > 0.0);
}

/* =================================================================================================== */
/* ===================================  SYMX DATA-SYMBOLS MAPPING  =================================== */
/* =================================================================================================== */
symx::Vector stark::models::EnergyRigidBodyConstraints::_get_x1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc)
{
	symx::Vector v1 = energy.make_dof_vector(this->dyn->dof_v, this->dyn->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dyn->dof_w, this->dyn->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->dyn->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->dyn->q0_, rb_idx);
	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
	return integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
}
symx::Vector stark::models::EnergyRigidBodyConstraints::_get_d1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& d_loc)
{
	symx::Vector w1 = energy.make_dof_vector(this->dyn->dof_w, this->dyn->w1, rb_idx);
	symx::Vector q0 = energy.make_vector(this->dyn->q0_, rb_idx);
	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
	return integrate_loc_direction(d_loc, q0, w1, dt);
}
std::array<symx::Vector, 2> stark::models::EnergyRigidBodyConstraints::_get_x1_d1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc, const symx::Vector& d_loc)
{
	symx::Vector v1 = energy.make_dof_vector(this->dyn->dof_v, this->dyn->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dyn->dof_w, this->dyn->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->dyn->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->dyn->q0_, rb_idx);
	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

	symx::Vector x1 = integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
	symx::Vector d1 = integrate_loc_direction(d_loc, q0, w1, dt);
	return { x1, d1 };
}
std::array<symx::Vector, 2> stark::models::EnergyRigidBodyConstraints::_get_x0_x1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const symx::Vector& x_loc)
{
	symx::Vector v1 = energy.make_dof_vector(this->dyn->dof_v, this->dyn->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->dyn->dof_w, this->dyn->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->dyn->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->dyn->q0_, rb_idx);
	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

	symx::Vector x1 = integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
	symx::Vector x0 = local_to_global_point(x_loc, t0, q0);
	return { x0, x1 };
}

