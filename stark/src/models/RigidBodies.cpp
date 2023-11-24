#include "RigidBodies.h"

#include <symx>
#include <vtkio>

#include "../utils/mesh_utils.h"
#include "distances.h"
#include "time_integration.h"
#include "friction_geometry.h"
#include "rigidbody_transformations.h"

stark::models::RigidBodies::RigidBodies(Stark& stark, spRigidBodyDynamics dyn)
	: dyn(dyn)
{
	this->inertia = std::make_shared<EnergyRigidBodyInertia>(stark, dyn);
	this->constraints = std::make_shared<EnergyRigidBodyConstraints>(stark, dyn);
}

int stark::models::RigidBodies::add(const double mass, const Eigen::Matrix3d& inertia_loc, const Eigen::Vector3d& displacement, const double rotate_deg, const Eigen::Vector3d& rotation_axis)
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

void stark::models::RigidBodies::add_constraint_anchor_point(const int body_id, const Eigen::Vector3d& p_glob)
{
	const int constraint_id = (int)this->constraints.anchor_points.loc.size();
	this->constraints.anchor_points.conn.push_back({ constraint_id, body_id });
	this->constraints.anchor_points.target.push_back(p_glob);
	this->constraints.anchor_points.loc.push_back(global_to_local_point(p_glob, this->R1[body_id], this->t1[body_id]));
}
stark::models::AnchorPointHandler stark::models::RigidBodies::add_constraint_anchor_point(const RigidBodyHandler& body, const Eigen::Vector3d& p_glob, double stiffness_per_kg)
{
	const int idx = this->constraints->anchor_points->add(
		id.get_global_idx(), 
		this->dyn->global_to_local_point(id, p_glob),
		p_glob, 
		stiffness_per_kg * this->inertia->mass[id.get_global_idx()]
	);
	return AnchorPointHandler(this->constraints->anchor_points, idx);
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
void stark::models::RigidBodies::add_constraint_relative_direction_lock(const int body_0, const int body_1)
{
	this->add_constraint_relative_direction_lock(body_0, body_1, Eigen::Vector3d::UnitX());
	this->add_constraint_relative_direction_lock(body_0, body_1, Eigen::Vector3d::UnitY());
	this->add_constraint_relative_direction_lock(body_0, body_1, Eigen::Vector3d::UnitZ());
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
void stark::models::RigidBodies::add_constraint_parallel_gripper(const int base, const int right_finger, const int left_finger, const Eigen::Vector3d& d_global, const double max_force, const double closing_velocity, const double delay)
{
	const Eigen::Vector3d d = d_global.normalized();

	this->add_constraint_relative_direction_lock(base, right_finger);
	this->add_constraint_relative_direction_lock(base, left_finger);
	this->add_constraint_slider(base, right_finger, this->t1[right_finger] + d, this->t1[right_finger]);
	this->add_constraint_slider(base, left_finger, this->t1[left_finger] + d, this->t1[left_finger]);
	
	const int constraint_id = (int)this->constraints.parallel_gripper.conn.size();
	this->constraints.parallel_gripper.conn.push_back({ constraint_id, right_finger, left_finger });
	this->constraints.parallel_gripper.loc_da.push_back(global_to_local_direction(d, this->R1[right_finger]));
	this->constraints.parallel_gripper.max_force.push_back(max_force);
	this->constraints.parallel_gripper.target_v.push_back(closing_velocity);
	this->constraints.parallel_gripper.delay.push_back(delay);
}

void stark::models::RigidBodies::add_to_output_label(const std::string label, Id& id)
{
	this->output_groups.add_to_group(label, id.get_global_idx());
}

void stark::models::RigidBodies::_write_frame(Stark& sim)
{
	if (this->is_empty()) { return; }
	this->constraint_logger.save_to_disk();
	if (!this->write_VTK) { return; }

	if (this->output_labeled_groups.size() > 0) {
		for (const auto& pair : this->output_labeled_groups) {
			const std::string& label = pair.first;
			const std::vector<int>& rb_indices = pair.second;

			std::vector<Eigen::Vector3d> glob_vertices;
			std::vector<std::array<int, 3>> triangles;
			for (const int rb_i : rb_indices) {
				const Eigen::Matrix3d& R = this->R1[rb_i];
				const Eigen::Vector3d& t = this->t1[rb_i];

				const std::array<int, 2> v_range = this->collision_mesh.get_vertices_range(rb_i);
				const int idx_offset =  (int)glob_vertices.size() - v_range[0];
				for (int vertex_i = v_range[0]; vertex_i < v_range[1]; vertex_i++) {
					const Eigen::Vector3d p = local_to_global_point(this->collision_mesh.vertices[vertex_i], R, t);
					glob_vertices.push_back(p);
				}
			
				const std::array<int, 2> e_range = this->collision_mesh.get_elements_range(rb_i);
				for (int tri_i = e_range[0]; tri_i < e_range[1]; tri_i++) {
					const std::array<int, 3>& tri = this->collision_mesh.connectivity[tri_i];
					triangles.push_back({ tri[0] + idx_offset, tri[1] + idx_offset, tri[2] + idx_offset });
				}
			}
			if (label.size() == 0) {
				utils::write_VTK(sim.get_vtk_path("rb"), glob_vertices, triangles, false);
			}
			else {
				utils::write_VTK(sim.get_vtk_path("rb_" + label), glob_vertices, triangles, false);
			}
		}
	}
	else {
		std::vector<Eigen::Vector3d> glob_vertices(this->collision_mesh.get_n_vertices());
		for (int rb_i = 0; rb_i < this->collision_mesh.get_n_meshes(); rb_i++) {
			const Eigen::Matrix3d& R = this->R1[rb_i];
			const Eigen::Vector3d& t = this->t1[rb_i];

			const std::array<int, 2> range = this->collision_mesh.get_vertices_range(rb_i);
			for (int vertex_i = range[0]; vertex_i < range[1]; vertex_i++) {
				const Eigen::Vector3d p = local_to_global_point(this->collision_mesh.vertices[vertex_i], R, t);
				glob_vertices[vertex_i] = p;
			}
		}
		utils::write_VTK(sim.get_vtk_path("rb"), glob_vertices, this->collision_mesh.connectivity, false);
	}
}
