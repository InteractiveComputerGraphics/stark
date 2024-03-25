#pragma once
#include "../rigidbodies/RigidBodies.h"
#include "../interactions/Interactions.h"


namespace stark
{
	namespace RigidBody
	{
		struct Handler
		{
			RigidBodyHandler rigidbody;
			ContactHandler contact;
		};

		/**
		* @brief Vertex-Connectivity-Handler
		*/
		struct VCH
		{
			std::vector<Eigen::Vector3d> vertices;
			std::vector<std::array<int, 3>> triangles;
			Handler handler;
		};
	}

	class RigidBodyPresets
	{
	public:
		/* Methods */
		RigidBodyPresets(core::Stark& stark, std::shared_ptr<RigidBodies> rigidbodies, std::shared_ptr<Interactions> interactions);

		RigidBody::Handler add(const std::string& output_label, double mass, const Eigen::Matrix3d& inertia_local,
			const std::vector<Eigen::Vector3d>& collision_vertices, const std::vector<std::array<int, 3>>& collision_triangles,
			const std::vector<Eigen::Vector3d>& render_vertices, const std::vector<std::array<int, 3>>& render_triangles,
			const ContactParams& contact_params = ContactParams());
		RigidBody::Handler add(const std::string& output_label, double mass, const Eigen::Matrix3d& inertia_local,
			const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const ContactParams& contact_params = ContactParams());

		RigidBody::VCH add_sphere(const std::string& output_label, double mass, double radius, int subdivisions = 2, const ContactParams& contact_params = ContactParams());
		RigidBody::VCH add_box(const std::string& output_label, double mass, const Eigen::Vector3d& size, const ContactParams& contact_params = ContactParams());
		RigidBody::VCH add_box(const std::string& output_label, double mass, double size, const ContactParams& contact_params = ContactParams());
		RigidBody::VCH add_cylinder(const std::string& output_label, double mass, double radius, double full_height, int slices = 16, int stacks = 1, const ContactParams& contact_params = ContactParams());
		RigidBody::VCH add_torus(const std::string& output_label, double mass, double outer_radius, double inner_radius, int slices = 16, int stacks = 32, const ContactParams& contact_params = ContactParams());

	private:
		/* Fields */
		std::shared_ptr<RigidBodies> rigidbodies;
		std::shared_ptr<Interactions> interactions;
	};
}
