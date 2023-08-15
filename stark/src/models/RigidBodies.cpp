#include "RigidBodies.h"

#include <symx>
#include <vtkio>

#include "../utils/mesh_utils.h"
#include "distances.h"
#include "time_integration.h"
#include "friction_geometry.h"
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
	sim.callbacks.before_energy_evaluation.push_back([&]() { this->_update_contacts(sim); });
	sim.callbacks.is_state_valid.push_back([&]() { return this->_is_valid_configuration(sim); });

	// Energy declarations
	this->_energies_mechanical(sim);
	this->_energies_contact(sim);
	this->_energies_friction(sim);
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
	this->mu.push_back(0.0);
	this->motor_torque.push_back(Eigen::Vector3d::Zero());

	this->mesh.add_mesh(vertices, triangles);

	this->edges.clear();
	utils::find_edges(this->edges, this->mesh.connectivity, this->mesh.get_n_vertices());

	return this->get_n_bodies() - 1;
}
int stark::models::RigidBodies::add_and_transform(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double mass, const Eigen::Matrix3d& inertia_loc, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
{
	const int id = this->add(vertices, triangles, mass, inertia_loc);
	this->add_rotation(id, rotate_deg, rotation_axis);
	this->add_displacement(id, displacement);
	return id;
}
int stark::models::RigidBodies::add_sphere(const double mass, const double radius, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis, const int subdivisions)
{
	const utils::Mesh m = utils::make_sphere(radius, subdivisions);
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_sphere(mass, radius);
	return this->add_and_transform(m.vertices, m.triangles, mass, inertia_local, displacement, rotate_deg, rotation_axis);
}
int stark::models::RigidBodies::add_box(const double mass, const Eigen::Vector3d& size, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
{
	const utils::Mesh m = utils::make_box(size);
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_box(mass, size);
	return this->add_and_transform(m.vertices, m.triangles, mass, inertia_local, displacement, rotate_deg, rotation_axis);
}
int stark::models::RigidBodies::add_cylinder(const double mass, const double radius, const double full_height, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis, const int slices, const int stacks)
{
	const utils::Mesh m = utils::make_cylinder(radius, full_height, slices, stacks);
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_cylinder(mass, radius, full_height);
	return this->add_and_transform(m.vertices, m.triangles, mass, inertia_local, displacement, rotate_deg, rotation_axis);
}
int stark::models::RigidBodies::add_torus(const double mass, const double outer_radius, const double inner_radius, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis, const int slices, const int stacks)
{
	const utils::Mesh m = utils::make_torus(outer_radius, inner_radius, slices, stacks);
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_torus(mass, outer_radius, inner_radius);
	return this->add_and_transform(m.vertices, m.triangles, mass, inertia_local, displacement, rotate_deg, rotation_axis);
}
int stark::models::RigidBodies::add_sphere(const double mass, const double radius, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
{
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_sphere(mass, radius);
	return this->add_and_transform(vertices, triangles, mass, inertia_local, displacement, rotate_deg, rotation_axis);
}
int stark::models::RigidBodies::add_box(const double mass, const Eigen::Vector3d& size, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
{
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_box(mass, size);
	return this->add_and_transform(vertices, triangles, mass, inertia_local, displacement, rotate_deg, rotation_axis);
}
int stark::models::RigidBodies::add_cylinder(const double mass, const double radius, const double full_height, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
{
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_cylinder(mass, radius, full_height);
	return this->add_and_transform(vertices, triangles, mass, inertia_local, displacement, rotate_deg, rotation_axis);
}
int stark::models::RigidBodies::add_torus(const double mass, const double outer_radius, const double inner_radius, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
{
	const Eigen::Matrix3d inertia_local = utils::inertia_tensor_torus(mass, outer_radius, inner_radius);
	return this->add_and_transform(vertices, triangles, mass, inertia_local, displacement, rotate_deg, rotation_axis);
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
void stark::models::RigidBodies::add_constraint_motor(const int body_0, const int body_1, const Eigen::Vector3d& c_global, const Eigen::Vector3d& d_global, const double max_torque, const double target_w, const double delay)
{
	// Careful: delay directly determines the transition between torque application and max torque in rad/s.
	// For smooth operation, set a large value (when very far from target w, apply max_torque)
	// Small values will have a more aggresive max torque application but make the problem harder to solve.

	this->add_constraint_hinge_joint(body_0, body_1, c_global, d_global);

	const Eigen::Vector3d d = d_global.normalized();
	const int constraint_id = (int)this->constraints.motors.conn.size();
	this->constraints.motors.conn.push_back({ constraint_id, body_0, body_1 });
	this->constraints.motors.loc_da.push_back(global_to_local_direction(d, this->R1[body_0]));
	this->constraints.motors.max_torque.push_back(max_torque);
	this->constraints.motors.target_w.push_back(target_w);
	this->constraints.motors.delay.push_back(delay);
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
	this->v0[body_id] = vel_glob_coords;
	this->v1[body_id] = vel_glob_coords;
}
void stark::models::RigidBodies::add_velocity(const int body_id, const Eigen::Vector3d& vel_glob_coords)
{
	this->v0[body_id] += vel_glob_coords;
	this->v1[body_id] += vel_glob_coords;
}
void stark::models::RigidBodies::set_angular_velocity(const int body_id, const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->w0[body_id] = angular_vel_glob_coords;
	this->w1[body_id] = angular_vel_glob_coords;
}
void stark::models::RigidBodies::add_angular_velocity(const int body_id, const Eigen::Vector3d& angular_vel_glob_coords)
{
	this->w0[body_id] += angular_vel_glob_coords;
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
void stark::models::RigidBodies::set_friction(const int body_id, const double coulombs_mu)
{
	this->mu[body_id] = coulombs_mu;
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

void stark::models::RigidBodies::_update_collision_x1(Stark& sim, const double dt)
{
	this->collision_x1.resize(this->mesh.vertices.size());
	for (int rb_i = 0; rb_i < this->mesh.get_n_meshes(); rb_i++) {
		const Eigen::Vector3d t1 = time_integration(this->t0[rb_i], this->v1[rb_i], dt);
		const Eigen::Quaterniond q1 = quat_time_integration(this->q0[rb_i], this->w1[rb_i], dt);
		const Eigen::Matrix3d R1 = q1.toRotationMatrix();

		const std::array<int, 2> range = this->mesh.get_vertices_range(rb_i);
		for (int vertex_i = range[0]; vertex_i < range[1]; vertex_i++) {
			const Eigen::Vector3d p = local_to_global_point(this->mesh.vertices[vertex_i], R1, t1);
			this->collision_x1[vertex_i] = p;
		}
	}
}
const tmcd::ProximityResults& stark::models::RigidBodies::_run_proximity_detection(const std::vector<Eigen::Vector3d>& x, Stark& sim)
{
	this->pd.clear();
	this->pd.set_n_threads(sim.settings.execution.n_threads);
	this->pd.set_edge_edge_parallel_cutoff(sim.settings.contact.edge_edge_cross_norm_sq_cutoff);
	this->pd.add_mesh(&x[0][0], (int)x.size(), &this->mesh.connectivity[0][0], this->mesh.get_n_elements(), &this->edges[0][0], (int)this->edges.size());
	this->pd.activate_point_triangle(sim.settings.contact.triangle_point_enabled);
	this->pd.activate_edge_edge(sim.settings.contact.edge_edge_enabled);
	const tmcd::ProximityResults& proximity = this->pd.run(sim.settings.contact.dhat, tmcd::BroadPhaseStrategy::OctreeSIMD);  // DEBUG
	return proximity;
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
		const Eigen::Matrix3d J = local_to_global_matrix(this->J_loc[i], this->R0[i]);
		this->J0_glob[i] = {
			J(0, 0), J(0, 1), J(0, 2),
			J(1, 0), J(1, 1), J(1, 2),
			J(2, 0), J(2, 1), J(2, 2),
		};
		const Eigen::Matrix3d J_inv = J.inverse();
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

	// Active friction points at x0 for this time step
	this->_update_friction_contacts(sim);
}
void stark::models::RigidBodies::_after_time_step(Stark& sim)
{
	if (this->is_empty()) { return; }

	const double dt = sim.settings.simulation.adaptive_time_step.value;

	// Set final positions with solved velocities
	for (int i = 0; i < this->get_n_bodies(); i++) {
		this->t1[i] = time_integration(this->t0[i], this->v1[i], dt);
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
bool stark::models::RigidBodies::_is_valid_configuration(Stark& sim)
{
	if (this->is_empty()) { return true; }
	if (!sim.settings.contact.collisions_enabled) { return true; }
	if (!sim.settings.contact.enable_intersection_test) { return true; }

	this->_update_collision_x1(sim, sim.settings.simulation.adaptive_time_step.value);
	this->id.clear();
	this->id.set_n_threads(sim.settings.execution.n_threads);
	this->id.add_mesh(&this->collision_x1[0][0], (int)this->collision_x1.size(), &this->mesh.connectivity[0][0], this->mesh.get_n_elements(), &this->edges[0][0], (int)this->edges.size());
	const tmcd::IntersectionResults& intersections = this->id.run();
	return intersections.edge_triangle.size() == 0;
}
void stark::models::RigidBodies::_update_contacts(Stark& sim)
{
	if (this->is_empty()) { return; }
	if (!sim.settings.contact.collisions_enabled) { return; }

	// Proximity detection
	this->_update_collision_x1(sim, sim.settings.simulation.adaptive_time_step.value);
	const tmcd::ProximityResults& proximity = this->_run_proximity_detection(this->collision_x1, sim);

	// Fill connectivities
	this->contacts.clear();

	//// Point - Triangle
	for (const auto& pair : proximity.point_triangle.point_point) {
		const tmcd::Point& p = pair.first;
		const tmcd::TrianglePoint& tp = pair.second;
		const tmcd::Point& q = tp.point;

		const int rb_a_idx = this->mesh.get_mesh_containing_vertex(p.idx);
		const int rb_b_idx = this->mesh.get_mesh_containing_vertex(q.idx);
		if (rb_a_idx != rb_b_idx) {
			this->contacts.point_triangle.point_point.push_back({ rb_a_idx, rb_b_idx, p.idx, q.idx });
		}
	}
	for (const auto& pair : proximity.point_triangle.point_edge) {
		const tmcd::Point& point = pair.first;
		const tmcd::TriangleEdge& te = pair.second;
		const tmcd::TriangleEdge::Edge& edge = te.edge;

		const int rb_a_idx = this->mesh.get_mesh_containing_vertex(point.idx);
		const int rb_b_idx = this->mesh.get_mesh_containing_vertex(edge.vertices[0]);
		if (rb_a_idx != rb_b_idx) {
			this->contacts.point_triangle.point_edge.push_back({ rb_a_idx, rb_b_idx, point.idx, edge.vertices[0], edge.vertices[1] });
		}
	}
	for (const auto& pair : proximity.point_triangle.point_triangle) {
		const tmcd::Point& point = pair.first;
		const tmcd::Triangle& triangle = pair.second;

		const int rb_a_idx = this->mesh.get_mesh_containing_vertex(point.idx);
		const int rb_b_idx = this->mesh.get_mesh_containing_vertex(triangle.vertices[0]);
		if (rb_a_idx != rb_b_idx) {
			this->contacts.point_triangle.point_triangle.push_back({ rb_a_idx, rb_b_idx, point.idx, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2] });
		}
	}

	//// Edge - Edge
	for (const auto& pair : proximity.edge_edge.point_point) {
		const tmcd::EdgePoint& ep_a = pair.first;
		const tmcd::EdgePoint& ep_b = pair.second;

		const int rb_a_idx = this->mesh.get_mesh_containing_vertex(ep_a.point.idx);
		const int rb_b_idx = this->mesh.get_mesh_containing_vertex(ep_b.point.idx);
		if (rb_a_idx != rb_b_idx) {
			this->contacts.edge_edge.point_point.push_back({ rb_a_idx, rb_b_idx, ep_a.edge.vertices[0], ep_a.edge.vertices[1], ep_a.point.idx, ep_b.edge.vertices[0], ep_b.edge.vertices[1], ep_b.point.idx });
		}
	}
	for (const auto& pair : proximity.edge_edge.point_edge) {
		const tmcd::EdgePoint& ep_a = pair.first;
		const tmcd::Edge& e_b = pair.second;

		const int rb_a_idx = this->mesh.get_mesh_containing_vertex(ep_a.point.idx);
		const int rb_b_idx = this->mesh.get_mesh_containing_vertex(e_b.vertices[0]);
		if (rb_a_idx != rb_b_idx) {
			this->contacts.edge_edge.point_edge.push_back({ rb_a_idx, rb_b_idx, ep_a.edge.vertices[0], ep_a.edge.vertices[1], ep_a.point.idx, e_b.vertices[0], e_b.vertices[1] });
		}
	}
	for (const auto& pair : proximity.edge_edge.edge_edge) {
		const int rb_a_idx = this->mesh.get_mesh_containing_vertex(pair.first.vertices[0]);
		const int rb_b_idx = this->mesh.get_mesh_containing_vertex(pair.second.vertices[0]);
		if (rb_a_idx != rb_b_idx) {
			this->contacts.edge_edge.edge_edge.push_back({ rb_a_idx, rb_b_idx, pair.first.vertices[0], pair.first.vertices[1], pair.second.vertices[0], pair.second.vertices[1] });
		}
	}
}
void stark::models::RigidBodies::_update_friction_contacts(Stark& sim)
{
	if (!sim.settings.contact.collisions_enabled) { return; }
	if (!sim.settings.contact.friction_enabled) { return; }

	// Proximity detection
	this->_update_collision_x1(sim, /* dt = */0.0);
	const std::vector<Eigen::Vector3d>& x = this->collision_x1;
	const tmcd::ProximityResults& proximity = this->_run_proximity_detection(x, sim);

	// Friction force
	const double k = sim.settings.contact.adaptive_contact_stiffness.value;
	const double dhat = sim.settings.contact.dhat;
	auto force = [&](double d) { return k * 3.0 * std::pow(dhat - d, 2); };

	// Fill friction data
	this->friction.clear();

	// Point - Triangle
	//// Point - Point
	for (const auto& pair : proximity.point_triangle.point_point) {
		const tmcd::Point& p = pair.first;
		const tmcd::TrianglePoint& tp = pair.second;
		const tmcd::Point& q = tp.point;

		const int rb_a_idx = this->mesh.get_mesh_containing_vertex(p.idx);
		const int rb_b_idx = this->mesh.get_mesh_containing_vertex(q.idx);
		if (rb_a_idx != rb_b_idx) {
			const double mu = 0.5 * (this->mu[rb_a_idx] + this->mu[rb_b_idx]);
			if (mu > 0.0) {
				this->friction.point_point.conn.push_back({ (int)this->friction.point_point.conn.size(), rb_a_idx, rb_b_idx, p.idx, q.idx });
				this->friction.point_point.contact.T.push_back(projection_matrix_point_point(x[p.idx], x[q.idx]));
				this->friction.point_point.contact.mu.push_back(mu);
				this->friction.point_point.contact.fn.push_back(force(pair.distance));
			}
		}
	}
	//// Point - Edge
	for (const auto& pair : proximity.point_triangle.point_edge) {
		const tmcd::Point& p = pair.first;
		const tmcd::TriangleEdge& te = pair.second;
		const tmcd::TriangleEdge::Edge& edge = te.edge;

		const int rb_a_idx = this->mesh.get_mesh_containing_vertex(p.idx);
		const int rb_b_idx = this->mesh.get_mesh_containing_vertex(edge.vertices[0]);

		if (rb_a_idx != rb_b_idx) {
			const double mu = 0.5 * (this->mu[rb_a_idx] + this->mu[rb_b_idx]);
			if (mu > 0.0) {
				this->friction.point_edge.conn.push_back({ (int)this->friction.point_edge.conn.size(), rb_a_idx, rb_b_idx, p.idx, edge.vertices[0], edge.vertices[1] });
				this->friction.point_edge.bary.push_back(barycentric_point_edge(x[p.idx], x[edge.vertices[0]], x[edge.vertices[1]]));
				this->friction.point_edge.contact.T.push_back(projection_matrix_point_edge(x[p.idx], x[edge.vertices[0]], x[edge.vertices[1]]));
				this->friction.point_edge.contact.mu.push_back(mu);
				this->friction.point_edge.contact.fn.push_back(force(pair.distance));
			}
		}
	}
	//// Point - Triangle
	for (const auto& pair : proximity.point_triangle.point_triangle) {
		const tmcd::Point& p = pair.first;
		const tmcd::Triangle& t = pair.second;

		const int rb_a_idx = this->mesh.get_mesh_containing_vertex(p.idx);
		const int rb_b_idx = this->mesh.get_mesh_containing_vertex(t.vertices[0]);

		if (rb_a_idx != rb_b_idx) {
			const double mu = 0.5 * (this->mu[rb_a_idx] + this->mu[rb_b_idx]);
			if (mu > 0.0) {
				this->friction.point_triangle.conn.push_back({ (int)this->friction.point_triangle.conn.size(), rb_a_idx, rb_b_idx, p.idx, t.vertices[0], t.vertices[1], t.vertices[2] });
				this->friction.point_triangle.bary.push_back(barycentric_point_triangle(x[p.idx], x[t.vertices[0]], x[t.vertices[1]], x[t.vertices[2]]));
				this->friction.point_triangle.contact.T.push_back(projection_matrix_triangle(x[t.vertices[0]], x[t.vertices[1]], x[t.vertices[2]]));
				this->friction.point_triangle.contact.mu.push_back(mu);
				this->friction.point_triangle.contact.fn.push_back(force(pair.distance));
			}
		}
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
			symx::Scalar E = 0.5*m*dev.dot(dev) + 0.5*m*v1.dot(v1)*damping*dt;
			// Actual force: f = -dE/dx = -1/dt*(v1 - vhat)*m
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
			symx::Vector mt = energy.make_vector(this->motor_torque, conn[0]);
			symx::Matrix J0_glob = energy.make_matrix(this->J0_glob, { 3, 3 }, conn[0]);
			symx::Matrix J0_inv_glob = energy.make_matrix(this->J0_inv_glob, { 3, 3 }, conn[0]);
			symx::Scalar damping = energy.make_scalar(this->damping);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			symx::Vector what = w0 + dt*(aa + J0_inv_glob*(t + mt));
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

	// Motor
	sim.global_energy.add_energy("rb_constraint_motor", this->constraints.motors.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "idx", "a", "b" });

			// Inputs
			symx::Vector da_loc = energy.make_vector(this->constraints.motors.loc_da, conn["idx"]);
			symx::Scalar max_torque = energy.make_scalar(this->constraints.motors.max_torque, conn["idx"]);
			symx::Scalar target_w = energy.make_scalar(this->constraints.motors.target_w, conn["idx"]);
			symx::Scalar h = energy.make_scalar(this->constraints.motors.delay, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Vector w1a = energy.make_dof_vector(this->dof_w, this->w1, conn["a"]);
			symx::Vector w1b = energy.make_dof_vector(this->dof_w, this->w1, conn["b"]);

			// Transformations
			symx::Vector q0a = energy.make_vector(this->q0_, conn["a"]);
			symx::Vector da = integrate_loc_direction(da_loc, q0a, w1a, dt);

			// Constraint (Analogous to C1 friction)
			// Important: derivatives wrt "positions", therefore needed chain rule and resulted in added product by dt
			symx::Scalar k = max_torque/h;
			symx::Scalar eps = max_torque/(2.0*k);
			symx::Scalar dw = target_w - da.dot(w1b - w1a);
			symx::Scalar E_l = 0.5*k*dw.powN(2)*dt;
			symx::Scalar E_r = max_torque*(dw - eps)*dt;
			symx::Scalar E = symx::branch(dw < h, E_l, E_r);
			energy.set(E);
		}
	);
}
void stark::models::RigidBodies::_energies_contact(Stark& sim)
{
	// Common symbol creation and manipulation functions ------------------------
	auto get_x1 = [&](const std::vector<symx::Index>& indices, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
		symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
		symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
		symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

		symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
		symx::Vector t1 = time_integration(t0, v1, dt);

		std::vector<symx::Vector> x_loc = energy.make_vectors(this->mesh.vertices, indices);

		std::vector<symx::Vector> x1;
		for (const symx::Vector& x_loc_a : x_loc) {
			x1.push_back(local_to_global_point(x_loc_a, t1, R1));
		}
		return x1;
	};
	auto get_X = [&](const std::vector<symx::Index>& indices, symx::Energy& energy)
	{
		return energy.make_vectors(this->mesh.vertices, indices);
	};
	auto get_edge_point = [&](const std::vector<symx::Index>& conn, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		// conn = { conn["a_e0"], conn["a_e1"], conn["a_p"] }
		std::vector<symx::Vector> A = get_x1(conn, rb_idx, dt, energy);
		std::vector<symx::Vector> EA = { A[0], A[1] };
		symx::Vector P = A[2];
		std::vector<symx::Vector> EA_REST = get_X({ conn[0], conn[1] }, energy);
		return std::make_tuple(EA_REST, EA, P);
	};
	auto get_edge = [&](const std::vector<symx::Index>& conn, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		// conn = { conn["a_e0"], conn["a_e1"] }
		std::vector<symx::Vector> EA = get_x1(conn, rb_idx, dt, energy);
		std::vector<symx::Vector> EA_REST = get_X(conn, energy);
		return std::make_tuple(EA_REST, EA);
	};
	auto barrier_energy = [&](const symx::Scalar& d, const symx::Scalar& dhat, const symx::Scalar& k)
	{
		return k * (dhat - d).powN(3);
	};
	auto set_barrier_energy = [&](const symx::Scalar& d, symx::Energy& energy)
	{
		symx::Scalar k = energy.make_scalar(sim.settings.contact.adaptive_contact_stiffness.value);
		symx::Scalar dhat = energy.make_scalar(sim.settings.contact.dhat);
		symx::Scalar E = barrier_energy(d, dhat, k);
		energy.set(E);
		energy.activate(sim.settings.contact.collisions_enabled);
	};
	auto edge_edge_mollifier = [&](const std::vector<symx::Vector>& EA, const std::vector<symx::Vector>& EB, const std::vector<symx::Vector>& EA_REST, const std::vector<symx::Vector>& EB_REST)
	{
		symx::Scalar eps_x = 1e-3 * (EA_REST[0] - EA_REST[1]).squared_norm() * (EB_REST[0] - EB_REST[1]).squared_norm();
		symx::Scalar x = (EA[1] - EA[0]).cross3(EB[1] - EB[0]).squared_norm();
		symx::Scalar x_div_eps_x = x / eps_x;
		symx::Scalar f = (-x_div_eps_x + 2.0) * x_div_eps_x;
		symx::Scalar mollifier = symx::branch(x > eps_x, 1.0, f);
		return mollifier;
	};
	auto set_edge_dge_mollified_barrier_energy = [&](const std::vector<symx::Vector>& EA, const std::vector<symx::Vector>& EB, const std::vector<symx::Vector>& EA_REST, const std::vector<symx::Vector>& EB_REST, const symx::Scalar& d, symx::Energy& energy)
	{
		symx::Scalar k = energy.make_scalar(sim.settings.contact.adaptive_contact_stiffness.value);
		symx::Scalar dhat = energy.make_scalar(sim.settings.contact.dhat);
		symx::Scalar E = edge_edge_mollifier(EA, EB, EA_REST, EB_REST) * barrier_energy(d, dhat, k);
		energy.set(E);
		energy.activate(sim.settings.contact.collisions_enabled);
	};
	// -----------------------------------------------------------------------------

	// Point - Triangle
	//// Point - Point
	sim.global_energy.add_energy("collision_rb_rb_pt_point_point", this->contacts.point_triangle.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "a", "b", "a_p", "b_q" });

			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> P = get_x1({ conn["a_p"] }, conn["a"], dt, energy);
			std::vector<symx::Vector> Q = get_x1({ conn["b_q"] }, conn["b"], dt, energy);
			symx::Scalar d = distance_point_point(P[0], Q[0]);
			set_barrier_energy(d, energy);
		}
	);

	//// Point - Edge
	sim.global_energy.add_energy("collision_rb_rb_pt_point_edge", this->contacts.point_triangle.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "a", "b", "a_p", "b_e0", "b_e1" });

			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> P = get_x1({ conn["a_p"] }, conn["a"], dt, energy);
			std::vector<symx::Vector> Q = get_x1({ conn["b_e0"], conn["b_e1"] }, conn["b"], dt, energy);
			symx::Scalar d = distance_point_line(P[0], Q[0], Q[1]);
			set_barrier_energy(d, energy);
		}
	);

	//// Point - Triangle
	sim.global_energy.add_energy("collision_rb_rb_pt_point_triangle", this->contacts.point_triangle.point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "a", "b", "a_p", "b_t0", "b_t1", "b_t2" });

			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> P = get_x1({ conn["a_p"] }, conn["a"], dt, energy);
			std::vector<symx::Vector> Q = get_x1({ conn["b_t0"], conn["b_t1"], conn["b_t2"]}, conn["b"], dt, energy);
			symx::Scalar d = distance_point_plane(P[0], Q[0], Q[1], Q[2]);
			set_barrier_energy(d, energy);
		}
	);

	// Edge - Edge
	//// Point - Point
	sim.global_energy.add_energy("collision_rb_rb_ee_point_point", this->contacts.edge_edge.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "a", "b", "a_e0", "a_e1", "a_p", "b_e0", "b_e1", "b_q" });

			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			auto [EA_REST, EA, P] = get_edge_point({ conn["a_e0"], conn["a_e1"], conn["a_p"] }, conn["a"], dt, energy);
			auto [EB_REST, EB, Q] = get_edge_point({ conn["b_e0"], conn["b_e1"], conn["b_q"] }, conn["b"], dt, energy);
			symx::Scalar d = distance_point_point(P, Q);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);

	//// Point - Edge
	sim.global_energy.add_energy("collision_rb_rb_ee_point_edge", this->contacts.edge_edge.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "a", "b", "a_e0", "a_e1", "a_p", "b_e0", "b_e1" });

			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			auto [EA_REST, EA, P] = get_edge_point({ conn["a_e0"], conn["a_e1"], conn["a_p"] }, conn["a"], dt, energy);
			auto [EB_REST, EB] = get_edge({ conn["b_e0"], conn["b_e1"] }, conn["b"], dt, energy);
			symx::Scalar d = distance_point_line(P, EB[0], EB[1]);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);

	//// Edge - Edge
	sim.global_energy.add_energy("collision_rb_rb_ee_edge_edge", this->contacts.edge_edge.edge_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "a", "b", "a_e0", "a_e1", "b_e0", "b_e1" });

			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			auto [EA_REST, EA] = get_edge({ conn["a_e0"], conn["a_e1"] }, conn["a"], dt, energy);
			auto [EB_REST, EB] = get_edge({ conn["b_e0"], conn["b_e1"] }, conn["b"], dt, energy);
			symx::Scalar d = distance_line_line(EA[0], EA[1], EB[0], EB[1]);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);
}
void stark::models::RigidBodies::_energies_friction(Stark& sim)
{
	// Common symbol creation and manipulation functions --------------------------------------------------------------------------------------------------
	auto get_v1 = [&](const std::vector<symx::Index>& indices, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Vector v1 = energy.make_dof_vector(this->dof_v, this->v1, rb_idx);
		symx::Vector w1 = energy.make_dof_vector(this->dof_w, this->w1, rb_idx);
		symx::Vector t0 = energy.make_vector(this->t0, rb_idx);
		symx::Vector q0 = energy.make_vector(this->q0_, rb_idx);

		symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
		symx::Vector t1 = time_integration(t0, v1, dt);

		std::vector<symx::Vector> x_loc = energy.make_vectors(this->mesh.vertices, indices);

		std::vector<symx::Vector> v1_glob;
		for (const symx::Vector& x_loc_a : x_loc) {
			symx::Vector x1a = local_to_global_point(x_loc_a, t1, R1);
			symx::Vector r1a = x1a - t1;
			symx::Vector v1a = global_point_velocity_in_rigib_body(v1, w1, r1a);
			v1_glob.push_back(v1a);
		}
		return v1_glob;
	};
	auto set_friction_energy = [&](const symx::Vector& v, const symx::Index& contact_idx, const RigidBodyFriction::Contact& contact, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Matrix T  = energy.make_matrix(contact.T, { 2, 3 }, contact_idx);
		symx::Scalar mu = energy.make_scalar(contact.mu, contact_idx);
		symx::Scalar fn = energy.make_scalar(contact.fn, contact_idx);
		symx::Scalar epsv = energy.make_scalar(sim.settings.contact.friction_stick_slide_threshold);
		symx::Scalar perturbation = energy.make_scalar(sim.settings.contact.friction_displacement_perturbation);

		symx::Vector vt = T*v;
		symx::Vector ut = vt*dt;
		ut[0] += 1.13*perturbation;
		ut[1] -= 1.07*perturbation;
		symx::Scalar u = ut.norm();

		symx::Scalar epsu = dt*epsv;
		if (sim.settings.contact.better_friction_mode) {
			symx::Scalar k = mu*fn/epsu;
			symx::Scalar eps = mu*fn/(2.0*k);

			symx::Scalar E_stick = 0.5*k*u.powN(2);
			symx::Scalar E_slide = mu*fn*(u - eps);
			symx::Scalar E = symx::branch(u < epsu, E_stick, E_slide);
			energy.set(E);
		}
		else {
			symx::Scalar E_stick = mu*fn*(-u*u*u/(3.0*epsu.powN(2)) + u*u/epsu + epsu/3.0);
			symx::Scalar E_slide = mu*fn*u;
			symx::Scalar E = symx::branch(u < epsu, E_stick, E_slide);
			energy.set(E);
		}

		energy.activate(sim.settings.contact.collisions_enabled && sim.settings.contact.friction_enabled);
	};
	// ----------------------------------------------------------------------------------------------------------------------------------------------------


	// Point - Point
	sim.global_energy.add_energy("friction_rb_rb_point_point", this->friction.point_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "idx", "a", "b", "a_p", "b_q" });

			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> VP = get_v1({ conn["a_p"] }, conn["a"], dt, energy);
			std::vector<symx::Vector> VQ = get_v1({ conn["b_q"] }, conn["b"], dt, energy);

			symx::Vector v = VQ[0] - VP[0];
			set_friction_energy(v, conn["idx"], this->friction.point_point.contact, dt, energy);
		}
	);
	// Point - Edge
	sim.global_energy.add_energy("friction_rb_rb_point_edge", this->friction.point_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "idx", "a", "b", "a_p", "b_e0", "b_e1" });

			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> VP = get_v1({ conn["a_p"] }, conn["a"], dt, energy);
			std::vector<symx::Vector> VE = get_v1({ conn["b_e0"], conn["b_e1"] }, conn["b"], dt, energy);
			symx::Vector bary = energy.make_vector(this->friction.point_edge.bary, conn["idx"]);

			symx::Vector vp = VP[0];
			symx::Vector vq = bary[0]*VE[0] + bary[1]*VE[1];
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn["idx"], this->friction.point_edge.contact, dt, energy);
		}
	);
	// Point - Triangle
	sim.global_energy.add_energy("friction_rb_rb_point_triangle", this->friction.point_triangle.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			conn.set_labels({ "idx", "a", "b", "a_p", "b_t0", "b_t1", "b_t2" });

			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> VP = get_v1({ conn["a_p"] }, conn["a"], dt, energy);
			std::vector<symx::Vector> VT = get_v1({ conn["b_t0"], conn["b_t1"], conn["b_t2"] }, conn["b"], dt, energy);
			symx::Vector bary = energy.make_vector(this->friction.point_triangle.bary, conn["idx"]);

			symx::Vector vp = VP[0];
			symx::Vector vq = bary[0]*VT[0] + bary[1]*VT[1] + bary[2]*VT[2];  
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn["idx"], this->friction.point_triangle.contact, dt, energy);
		}
	);
	//// Edge - Edge
	//sim.global_energy.add_energy("collision_cloth_cloth_friction_edge_edge", this->friction.edge_edge.conn,
	//	[&](symx::Energy& energy, symx::Element& conn)
	//	{
	//		std::vector<symx::Vector> VEA = get_v1({ conn[1], conn[2] }, energy);
	//		std::vector<symx::Vector> VEB = get_v1({ conn[3], conn[4] }, energy);
	//		symx::Vector bary = energy.make_vector(this->friction.edge_edge.bary, conn[0]);

	//		symx::Vector vp = VEA[0] + bary[0]*(VEA[1] - VEA[0]);
	//		symx::Vector vq = VEB[0] + bary[1]*(VEB[1] - VEB[0]);
	//		symx::Vector v = vq - vp;
	//		set_friction_energy(v, conn[0], this->friction.edge_edge.contact, energy);
	//	}
	//);
}
