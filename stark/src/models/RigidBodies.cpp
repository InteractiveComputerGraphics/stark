#include "RigidBodies.h"

#include <symx>
#include <vtkio>

#include "../utils/mesh_utils.h"
#include "distances.h"
#include "time_integration.h"
#include "rigidbody_transformations.h"

void stark::models::RigidBodies::init(Stark& sim)
{
	// DoFs
	this->dof_v = sim.global_energy.add_dof_array(this->v1, "rb_v1");
	this->dof_w = sim.global_energy.add_dof_array(this->w1, "rb_w1");

	// Callbacks
	sim.callbacks.before_time_step.push_back([&]() { this->_before_time_step(sim); });
	sim.callbacks.after_time_step.push_back([&]() { this->_after_time_step(sim); });
	sim.callbacks.write_frame.push_back([&]() { this->_write_frame(sim); });

	// Energy declarations
	this->_energies_mechanical(sim);
}

int stark::models::RigidBodies::add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double mass, const Eigen::Matrix3d& inertia_loc)
{
	this->t0.push_back(Eigen::Vector3d::Zero());
	this->t1.push_back(Eigen::Vector3d::Zero());
	this->q0.push_back(Eigen::Quaterniond::Identity());
	this->q1.push_back(Eigen::Quaterniond::Identity());
	this->R0.push_back(Eigen::Matrix3d::Identity());
	this->R1.push_back(Eigen::Matrix3d::Identity());
	this->v0.push_back(Eigen::Vector3d::Zero());
	this->v1.push_back(Eigen::Vector3d::Zero());
	this->w0.push_back(Eigen::Vector3d::Zero());
	this->w1.push_back(Eigen::Vector3d::Zero());
	this->a.push_back(Eigen::Vector3d::Zero());
	this->aa.push_back(Eigen::Vector3d::Zero());
	this->force.push_back(Eigen::Vector3d::Zero());
	this->torque.push_back(Eigen::Vector3d::Zero());
	this->J_loc.push_back(inertia_loc);
	this->mass.push_back(mass);

	this->mesh.add_mesh(vertices, triangles);
	return this->get_n_bodies() - 1;
}
void stark::models::RigidBodies::add_constraint_anchor_point(const int body_id, const Eigen::Vector3d& p_glob)
{
	const int constraint_id = (int)this->constraints.anchor_points.loc.size();
	this->constraints.anchor_points.conn.push_back({ constraint_id, body_id });
	this->constraints.anchor_points.target.push_back(p_glob);
	this->constraints.anchor_points.loc.push_back(global_to_local_point(p_glob, this->R1[body_id], this->t1[body_id]));
}
void stark::models::RigidBodies::add_constraint_ball_joint(const int body_0, const int body_1, const Eigen::Vector3d& p_glob)
{
	const int constraint_id = (int)this->constraints.ball_joints.loc_a.size();
	this->constraints.ball_joints.conn.push_back({ constraint_id, body_0, body_1 });
	this->constraints.ball_joints.loc_a.push_back(global_to_local_point(p_glob, this->R1[body_0], this->t1[body_0]));
	this->constraints.ball_joints.loc_b.push_back(global_to_local_point(p_glob, this->R1[body_1], this->t1[body_1]));
}
void stark::models::RigidBodies::add_constraint_hinge_joint(const int body_0, const int body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global)
{
	this->add_constraint_ball_joint(body_0, body_1, c_global);
	this->add_constraint_ball_joint(body_0, body_1, c_global + d_global.normalized());
}
void stark::models::RigidBodies::add_constraint_slider(const int body_0, const int body_1, const Eigen::Vector3d& p0_global, const Eigen::Vector3d& p1_global, const double spring_stiffness, const double spring_damping)
{
	const Eigen::Vector3d d = (p1_global - p0_global).normalized();
	const int constraint_id = (int)this->constraints.sliders.loc_a.size();
	this->constraints.sliders.conn.push_back({ constraint_id, body_0, body_1 });
	this->constraints.sliders.loc_a.push_back(global_to_local_point(p0_global, this->R1[body_0], this->t1[body_0]));
	this->constraints.sliders.loc_b.push_back(global_to_local_point(p1_global, this->R1[body_1], this->t1[body_1]));
	this->constraints.sliders.loc_da.push_back(global_to_local_direction(d, this->R1[body_0]));
	this->constraints.sliders.rest_length.push_back((p1_global - p0_global).norm());
	this->constraints.sliders.spring_stiffness.push_back(spring_stiffness);
	this->constraints.sliders.spring_damping.push_back(spring_damping);
}
void stark::models::RigidBodies::add_constraint_relative_direction_lock(const int body_0, const int body_1, const Eigen::Vector3d& d_global)
{
	const int constraint_id = (int)this->constraints.relative_direction_lock.loc_da.size();
	this->constraints.relative_direction_lock.conn.push_back({ constraint_id, body_0, body_1 });
	this->constraints.relative_direction_lock.loc_da.push_back(global_to_local_direction(d_global, this->R1[body_0]));
	this->constraints.relative_direction_lock.loc_db.push_back(global_to_local_direction(d_global, this->R1[body_1]));
}
void stark::models::RigidBodies::add_constraint_freeze(const int body_id)
{
	this->add_constraint_anchor_point(body_id, this->t1[body_id]);
	this->add_constraint_anchor_point(body_id, this->t1[body_id] + Eigen::Vector3d::UnitX());
	this->add_constraint_anchor_point(body_id, this->t1[body_id] + Eigen::Vector3d::UnitY());
	this->add_constraint_anchor_point(body_id, this->t1[body_id] + Eigen::Vector3d::UnitZ());
}

void stark::models::RigidBodies::set_damping(const double damping)
{
	this->damping = damping;
}
void stark::models::RigidBodies::set_displacement(const int body_id, const Eigen::Vector3d& t)
{
	this->t1[body_id] = t;
	this->t0[body_id] = t;
}
void stark::models::RigidBodies::add_displacement(const int body_id, const Eigen::Vector3d& t)
{
	this->t1[body_id] += t;
	this->t0[body_id] += t;
}
void stark::models::RigidBodies::set_rotation(const int body_id, const Eigen::Quaterniond& q)
{
	this->q0[body_id] = q.normalized();
	this->q1[body_id] = q.normalized();
	this->R0[body_id] = this->q0[body_id].toRotationMatrix();
	this->R1[body_id] = this->q1[body_id].toRotationMatrix();
}
void stark::models::RigidBodies::set_rotation(const int body_id, const double& angle_deg, const Eigen::Vector3d& axis)
{
	this->set_rotation(body_id, Eigen::Quaterniond(Eigen::AngleAxis<double>(utils::deg2rad(angle_deg), axis.normalized())));
}
void stark::models::RigidBodies::add_rotation(const int body_id, const Eigen::Quaterniond& q)
{
	const Eigen::Quaterniond q_ = q.normalized();
	const Eigen::Matrix3d R = q_.toRotationMatrix();
	this->t0[body_id] = R*this->t0[body_id];
	this->t1[body_id] = R*this->t1[body_id];
	this->set_rotation(body_id, this->q0[body_id]*q_);
}
void stark::models::RigidBodies::add_rotation(const int body_id, const double& angle_deg, const Eigen::Vector3d& axis)
{
	this->add_rotation(body_id, Eigen::Quaterniond(Eigen::AngleAxis<double>(utils::deg2rad(angle_deg), axis.normalized())));
}
void stark::models::RigidBodies::add_force_at(const int body_id, const Eigen::Vector3d& force_glob_coords, const Eigen::Vector3d& application_point_glob_coords)
{
	this->force[body_id] = force_glob_coords;
	this->torque[body_id] = (application_point_glob_coords - this->t1[body_id]).cross(force_glob_coords);
}
void stark::models::RigidBodies::set_force_at_centroid(const int body_id, const Eigen::Vector3d& force_glob_coords)
{
	this->force[body_id] = force_glob_coords;
}
void stark::models::RigidBodies::add_force_at_centroid(const int body_id, const Eigen::Vector3d& force_glob_coords)
{
	this->force[body_id] += force_glob_coords;
}
void stark::models::RigidBodies::set_torque(const int body_id, const Eigen::Vector3d& torque_glob_coords)
{
	this->torque[body_id] = torque_glob_coords;
}
void stark::models::RigidBodies::add_torque(const int body_id, const Eigen::Vector3d& torque_glob_coords)
{
	this->torque[body_id] += torque_glob_coords;
}
void stark::models::RigidBodies::set_velocity(const int body_id, const Eigen::Vector3d& vel_glob_coords)
{
	this->v1[body_id] = vel_glob_coords;
}
void stark::models::RigidBodies::add_velocity(const int body_id, const Eigen::Vector3d& vel_glob_coords)
{
	this->v1[body_id] += vel_glob_coords;
}
void stark::models::RigidBodies::set_angular_velocity(const int body_id, const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->w1[body_id] = angular_vel_glob_coords;
}
void stark::models::RigidBodies::add_angular_velocity(const int body_id, const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->w1[body_id] += angular_vel_glob_coords;
}
void stark::models::RigidBodies::set_acceleration(const int body_id, const Eigen::Vector3d& acc_glob_coords)
{
	this->a[body_id] = acc_glob_coords;
}
void stark::models::RigidBodies::add_acceleration(const int body_id, const Eigen::Vector3d& acc_glob_coords)
{
	this->a[body_id] += acc_glob_coords;
}
void stark::models::RigidBodies::set_angular_acceleration(const int body_id, const Eigen::Vector3d& ang_acc_glob_coords)
{
	this->aa[body_id] = ang_acc_glob_coords;
}
void stark::models::RigidBodies::add_angular_acceleration(const int body_id, const Eigen::Vector3d& ang_acc_glob_coords)
{
	this->aa[body_id] += ang_acc_glob_coords;
}
Eigen::Vector3d stark::models::RigidBodies::get_point_in_global_coordinates(const int body_id, const Eigen::Vector3d& p)
{
	return local_to_global_point(p, this->R1[body_id], this->t1[body_id]);
}
Eigen::Vector3d stark::models::RigidBodies::get_point_in_global_coordinates(const int body_id, const int point_i)
{
	const int global_point_idx = this->mesh.get_global_vertex_idx(body_id, point_i);
	return this->get_point_in_global_coordinates(body_id, this->mesh.vertices[global_point_idx]);
}
int stark::models::RigidBodies::get_n_bodies() const
{
	return (int)this->t0.size();
}
bool stark::models::RigidBodies::is_empty() const
{
	return this->get_n_bodies() == 0;
}
bool stark::models::RigidBodies::is_body_declared(const int body_id) const
{
	return body_id < this->get_n_bodies();
}

void stark::models::RigidBodies::_before_time_step(Stark& sim)
{
	if (this->is_empty()) { return; }

	// Set next time velocities estimation to zero to avoid invalid state outside of the minimzer
	std::fill(this->v1.begin(), this->v1.end(), Eigen::Vector3d::Zero());
	std::fill(this->w1.begin(), this->w1.end(), Eigen::Vector3d::Zero());

	// Inertia
	const int n = this->get_n_bodies();
	this->conn_inertia.resize(n);
	this->J0_glob.resize(n);
	this->J0_inv_glob.resize(n);
	for (int i = 0; i < n; i++) {
		this->conn_inertia[i] = { i };
		const Eigen::Matrix3d J = local_to_global_matrix(this->J_loc[i], this->R1[i]);
		const Eigen::Matrix3d J_inv = J.inverse();
		this->J0_glob[i] = {
			J(0, 0), J(0, 1), J(0, 2),
			J(1, 0), J(1, 1), J(1, 2),
			J(2, 0), J(2, 1), J(2, 2),
		};
		this->J0_inv_glob[i] = {
			J_inv(0, 0), J_inv(0, 1), J_inv(0, 2),
			J_inv(1, 0), J_inv(1, 1), J_inv(1, 2),
			J_inv(2, 0), J_inv(2, 1), J_inv(2, 2),
		};
	}

	// Quaternions
	this->q0_.resize(n);
	for (int i = 0; i < n; i++) {
		Eigen::Quaterniond& q = this->q0[i];
		this->q0_[i] = { q.w(), q.x(), q.y(), q.z() };
	}
}
void stark::models::RigidBodies::_after_time_step(Stark& sim)
{
	if (this->is_empty()) { return; }

	const double dt = sim.settings.simulation.adaptive_time_step.value;

	// Set final positions with solved velocities
	for (int i = 0; i < this->get_n_bodies(); i++) {
		this->t1[i] = this->t0[i] + dt * this->v1[i];
		this->q1[i] = quat_time_integration(this->q0[i], this->w1[i], dt);
		this->R1[i] = this->q1[i].toRotationMatrix();
	}

	// x0 <- x1
	this->t0 = this->t1;
	this->q0 = this->q1;
	this->R0 = this->R1;
	this->v0 = this->v1;
	this->w0 = this->w1;
}
void stark::models::RigidBodies::_write_frame(Stark& sim)
{
	if (this->is_empty()) { return; }

	std::vector<Eigen::Vector3d> glob_vertices(this->mesh.get_n_vertices());
	for (int rb_i = 0; rb_i < this->mesh.get_n_meshes(); rb_i++) {
		const Eigen::Matrix3d& R = this->R1[rb_i];
		const Eigen::Vector3d& t = this->t1[rb_i];

		const std::array<int, 2> range = this->mesh.get_vertices_range(rb_i);
		for (int vertex_i = range[0]; vertex_i < range[1]; vertex_i++) {
			const Eigen::Vector3d p = local_to_global_point(this->mesh.vertices[vertex_i], R, t);
			glob_vertices[vertex_i] = p;
		}
	}

	if (this->write_VTK) {
		utils::write_VTK(sim.get_vtk_path("rb"), glob_vertices, this->mesh.connectivity, false);
	}
}

void stark::models::RigidBodies::_energies_mechanical(Stark& sim)
{
	// Common symbol creation and manipulation functions ------------------------
	auto global_x1 = [&](const symx::Vector& x_loc, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
		symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
		symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
		symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);
		return integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
	};
	auto global_d1 = [&](const symx::Vector& d_loc, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
		symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);
		return integrate_loc_direction(d_loc, q0, w1, dt);
	};
	auto global_x1_d1 = [&](const symx::Vector& x_loc, const symx::Vector& d_loc, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
		symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
		symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
		symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

		symx::Vector x1 = integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
		symx::Vector d1 = integrate_loc_direction(d_loc, q0, w1, dt);
		return std::make_pair(x1, d1);
	};
	auto global_x0_x1_d1 = [&](const symx::Vector& x_loc, const symx::Vector& d_loc, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
		symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
		symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
		symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

		symx::Vector x0 = local_to_global_point(x_loc, t0, q0);
		symx::Vector x1 = integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
		symx::Vector d1 = integrate_loc_direction(d_loc, q0, w1, dt);
		return std::make_tuple(x0, x1, d1);
	};
	auto global_x0_x1 = [&](const symx::Vector& x_loc, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
		symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
		symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
		symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

		symx::Vector x0 = local_to_global_point(x_loc, t0, q0);
		symx::Vector x1 = integrate_loc_point(x_loc, t0, q0, v1, w1, dt);
		return std::make_tuple(x0, x1);
	};
	// --------------------------------------------------------------------------


	// Linear inertia
	sim.global_energy.add_energy("rb_linear_inertia", this->conn_inertia,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, conn[0]);
			symx::Vector v0 = energy.make_vector(this->v0, conn[0]);
			symx::Vector a = energy.make_vector(this->a, conn[0]);
			symx::Vector f = energy.make_vector(this->force, conn[0]);
			symx::Scalar m = energy.make_scalar(this->mass, conn[0]);
			symx::Scalar damping = energy.make_scalar(this->damping);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Vector gravity = energy.make_vector(sim.settings.simulation.gravity);

			symx::Vector vhat = v0 + dt*(a + gravity + f/m);
			symx::Vector dev = v1 - vhat;
			symx::Scalar E = 0.5*m*(dev.dot(dev) + v1.dot(v1)*damping*dt);
			energy.set(E);
		}
	);

	// Angular inertia
	sim.global_energy.add_energy("rb_angular_inertia", this->conn_inertia,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, conn[0]);
			symx::Vector w0 = energy.make_vector(this->w0, conn[0]);
			symx::Vector aa = energy.make_vector(this->aa, conn[0]);
			symx::Vector t = energy.make_vector(this->torque, conn[0]);
			symx::Matrix J0_glob = energy.make_matrix(this->J0_glob, { 3, 3 }, conn[0]);
			symx::Matrix J0_inv_glob = energy.make_matrix(this->J0_inv_glob, { 3, 3 }, conn[0]);
			symx::Scalar damping = energy.make_scalar(this->damping);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			symx::Vector what = w0 + dt*(aa + J0_inv_glob*t);
			symx::Vector dev = w1 - what;
			symx::Scalar E = 0.5*(dev.transpose()*J0_glob*dev + w1.transpose()*J0_glob*w1*damping*dt);
			energy.set(E);
		}
	);

	// Anchor point
	sim.global_energy.add_energy("rb_constraint_anchor_point", this->constraints.anchor_points.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "idx", "a" });

			symx::Vector loc = energy.make_vector(this->constraints.anchor_points.loc, conn["idx"]);
			symx::Vector target = energy.make_vector(this->constraints.anchor_points.target, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Scalar k = energy.make_scalar(this->constraint_stiffness);

			symx::Vector glob = global_x1(loc, conn["a"], dt, energy);
			symx::Scalar E = 0.5 * k * (target - glob).squared_norm();
			energy.set(E);
		}
	);

	// Ball joint
	sim.global_energy.add_energy("rb_constraint_ball_joints", this->constraints.ball_joints.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "idx", "a", "b" });

			symx::Vector a_loc = energy.make_vector(this->constraints.ball_joints.loc_a, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(this->constraints.ball_joints.loc_b, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Scalar k = energy.make_scalar(this->constraint_stiffness);

			symx::Vector a = global_x1(a_loc, conn["a"], dt, energy);
			symx::Vector b = global_x1(b_loc, conn["b"], dt, energy);
			symx::Scalar E = 0.5 * k * (a - b).squared_norm();
			energy.set(E);
		}
	);

	// Lock Relative Direction
	sim.global_energy.add_energy("rb_constraint_lock_relative_direction", this->constraints.relative_direction_lock.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "idx", "a", "b" });

			symx::Vector da_loc = energy.make_vector(this->constraints.relative_direction_lock.loc_da, conn["idx"]);
			symx::Vector db_loc = energy.make_vector(this->constraints.relative_direction_lock.loc_db, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Scalar k = energy.make_scalar(this->constraint_stiffness);

			symx::Vector da = global_d1(da_loc, conn["a"], dt, energy);
			symx::Vector db = global_d1(db_loc, conn["b"], dt, energy);
			symx::Scalar E = 0.5 * k * (da - db).squared_norm();
			energy.set(E);
		}
	);

	// Slider
	sim.global_energy.add_energy("rb_constraint_sliders", this->constraints.sliders.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "idx", "a", "b" });

			// Inputs
			symx::Vector a_loc = energy.make_vector(this->constraints.sliders.loc_a, conn["idx"]);
			symx::Vector b_loc = energy.make_vector(this->constraints.sliders.loc_b, conn["idx"]);
			symx::Vector da_loc = energy.make_vector(this->constraints.sliders.loc_da, conn["idx"]);
			symx::Scalar l_rest = energy.make_scalar(this->constraints.sliders.rest_length, conn["idx"]);
			symx::Scalar spring_stiffness = energy.make_scalar(this->constraints.sliders.spring_stiffness, conn["idx"]);
			symx::Scalar spring_damping = energy.make_scalar(this->constraints.sliders.spring_damping, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Scalar k = energy.make_scalar(this->constraint_stiffness);

			// Transformations
			auto [a0, a1, da1] = global_x0_x1_d1(a_loc, da_loc, conn["a"], dt, energy);
			auto [b0, b1] = global_x0_x1(b_loc, conn["b"], dt, energy);

			// Constraint
			symx::Vector r1 = b1 - a1;
			symx::Vector r0 = b0 - a0;
			symx::Scalar l1 = r1.norm();
			symx::Scalar l0 = r0.norm();

			symx::Scalar E_slider = 0.5 * k * sq_distance_point_line(b1, a1, a1 + da1);
			symx::Scalar E_spring = 0.5 * spring_stiffness * (l1/l_rest - 1.0).powN(2);
			symx::Scalar E_damper = 0.5 * spring_damping * ((l1 - l0)/dt).powN(2);
			symx::Scalar E = E_slider + E_spring + E_damper;
			energy.set(E);
		}
	);
}
