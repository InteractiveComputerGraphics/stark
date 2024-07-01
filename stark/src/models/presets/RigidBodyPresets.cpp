#include "RigidBodyPresets.h"

#include "../../utils/mesh_generators.h"
#include "../rigidbodies/inertia_tensors.h"


stark::RigidBodyPresets::RigidBodyPresets(core::Stark& stark, std::shared_ptr<RigidBodies> rigidbodies, std::shared_ptr<Interactions> interactions)
	: rigidbodies(rigidbodies), interactions(interactions)
{
}

stark::RigidBody::Handler stark::RigidBodyPresets::add(
	const std::string& output_label, double mass, const Eigen::Matrix3d& inertia_local, 
	const std::vector<Eigen::Vector3d>& collision_vertices, const std::vector<std::array<int, 3>>& collision_triangles, 
	const std::vector<Eigen::Vector3d>& render_vertices, const std::vector<std::array<int, 3>>& render_triangles, 
	const ContactParams& contact_params)
{
	RigidBodyHandler rb = this->rigidbodies->add(mass, inertia_local);
	ContactHandler contact = this->interactions->contact->add_triangles(rb, collision_vertices, collision_triangles, contact_params);

	if (!output_label.empty()) {
		this->rigidbodies->output.add_triangle_mesh(output_label, rb, render_vertices, render_triangles);
	}

	return { rb, contact };
}

stark::RigidBody::Handler stark::RigidBodyPresets::add(
	const std::string& output_label, double mass, const Eigen::Matrix3d& inertia_local, 
	const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles,
	const ContactParams& contact_params)
{
	return this->add(output_label, mass, inertia_local, vertices, triangles, vertices, triangles, contact_params);
}

stark::RigidBody::VCH stark::RigidBodyPresets::add_sphere(const std::string& output_label, double mass, double radius, int subdivisions, const ContactParams& contact_params)
{
	const Eigen::Matrix3d inertia_local = inertia_tensor_sphere(mass, radius);
	const Mesh<3> mesh = make_sphere(radius, subdivisions);
	RigidBody::Handler handler = this->add(output_label, mass, inertia_local, mesh.vertices, mesh.conn, contact_params);
	return { mesh.vertices, mesh.conn, handler };
}
stark::RigidBody::VCH stark::RigidBodyPresets::add_box(const std::string& output_label, double mass, const Eigen::Vector3d& size, const ContactParams& contact_params)
{
	const Eigen::Matrix3d inertia_local = inertia_tensor_box(mass, size);
	const Mesh<3> mesh = make_box(size);
	RigidBody::Handler handler = this->add(output_label, mass, inertia_local, mesh.vertices, mesh.conn, contact_params);
	return { mesh.vertices, mesh.conn, handler };
}
stark::RigidBody::VCH stark::RigidBodyPresets::add_box(const std::string& output_label, double mass, double size, const ContactParams& contact_params)
{
	return this->add_box(output_label, mass, { size, size, size }, contact_params);
}
stark::RigidBody::VCH stark::RigidBodyPresets::add_cylinder(const std::string& output_label, double mass, double radius, double full_height, int slices, int stacks, const ContactParams& contact_params)
{
	const Eigen::Matrix3d inertia_local = inertia_tensor_cylinder(mass, radius, full_height);
	const Mesh<3> mesh = make_cylinder(radius, full_height, slices, stacks);
	RigidBody::Handler handler = this->add(output_label, mass, inertia_local, mesh.vertices, mesh.conn, contact_params);
	return { mesh.vertices, mesh.conn, handler };
}
stark::RigidBody::VCH stark::RigidBodyPresets::add_torus(const std::string& output_label, double mass, double outer_radius, double inner_radius, int slices, int stacks, const ContactParams& contact_params)
{
	const Eigen::Matrix3d inertia_local = inertia_tensor_torus(mass, outer_radius, inner_radius);
	const Mesh<3> mesh = make_torus(outer_radius, inner_radius, slices, stacks);
	RigidBody::Handler handler = this->add(output_label, mass, inertia_local, mesh.vertices, mesh.conn, contact_params);
	return { mesh.vertices, mesh.conn, handler };
}
