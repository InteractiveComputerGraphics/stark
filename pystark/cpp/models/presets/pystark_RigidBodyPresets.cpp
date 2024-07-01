#include "../../nanobind_stark_include_all.h"

void pystark_RigidBodyPresets(nb::module_& m)
{
	auto rb_module = m.def_submodule("RigidBody");
	{
		nb::class_<RigidBody::Handler>(rb_module, "Handler")
			.def_rw("rigidbody", &RigidBody::Handler::rigidbody)
			.def_rw("contact", &RigidBody::Handler::contact)
			;
	}

    nb::class_<RigidBodyPresets>(m, "RigidBodyPresets")
		.def("add", [](RigidBodyPresets& self, const std::string& output_label, double mass, const Eigen::Matrix3d& inertia_local, MatX3d& collision_vertices, MatX3i& collision_triangles, MatX3d& render_vertices, MatX3i& render_triangles, const ContactParams& contact_params)
			{ return self.add(output_label, mass, inertia_local,  nb_to_stark(collision_vertices), nb_to_stark(collision_triangles),  nb_to_stark(render_vertices), nb_to_stark(render_triangles), contact_params); },
			"output_label"_a, "mass"_a, "inertia_local"_a, "collision_vertices"_a, "collision_triangles"_a, "render_vertices"_a, "render_triangles"_a, "contact_params"_a = ContactParams())
		.def("add", [](RigidBodyPresets& self, const std::string& output_label, double mass, const Eigen::Matrix3d& inertia_local, MatX3d& vertices, MatX3i& triangles, const ContactParams& contact_params)
			{ return self.add(output_label, mass, inertia_local,  nb_to_stark(vertices), nb_to_stark(triangles), contact_params); },
			"output_label"_a, "mass"_a, "inertia_local"_a, "vertices"_a, "triangles"_a, "contact_params"_a = ContactParams())
		.def("add_sphere", [](RigidBodyPresets& self, const std::string& output_label, double mass, double radius, int subdivisions, const ContactParams& contact_params)
			{ 
				auto [V, C, H] = self.add_sphere(output_label, mass, radius, subdivisions, contact_params);
				nb::tuple out = nb::make_tuple(stark_to_nb(V), stark_to_nb(C), H);
				return create_named_tuple("RigidBodyOutput", { "vertices", "segments", "handler" }, out);
			},
			"output_label"_a, "mass"_a, "radius"_a, "subdivisions"_a = 2, "contact_params"_a = ContactParams())
		.def("add_box", [](RigidBodyPresets& self, const std::string& output_label, double mass, const Eigen::Vector3d& size, const ContactParams& contact_params)
			{ 
				auto [V, C, H] = self.add_box(output_label, mass, size, contact_params);
				nb::tuple out = nb::make_tuple(stark_to_nb(V), stark_to_nb(C), H);
				return create_named_tuple("RigidBodyOutput", { "vertices", "segments", "handler" }, out);
			},
			"output_label"_a, "mass"_a, "size"_a, "contact_params"_a = ContactParams())
		.def("add_box", [](RigidBodyPresets& self, const std::string& output_label, double mass, double size, const ContactParams& contact_params)
			{ 
				auto [V, C, H] = self.add_box(output_label, mass, size, contact_params);
				nb::tuple out = nb::make_tuple(stark_to_nb(V), stark_to_nb(C), H);
				return create_named_tuple("RigidBodyOutput", { "vertices", "segments", "handler" }, out);
			},
			"output_label"_a, "mass"_a, "size"_a, "contact_params"_a = ContactParams())
		.def("add_cylinder", [](RigidBodyPresets& self, const std::string& output_label, double mass, double radius, double full_height, int slices, int stacks, const ContactParams& contact_params)
			{ 
				auto [V, C, H] = self.add_cylinder(output_label, mass, radius, full_height, slices, stacks, contact_params);
				nb::tuple out = nb::make_tuple(stark_to_nb(V), stark_to_nb(C), H);
				return create_named_tuple("RigidBodyOutput", { "vertices", "segments", "handler" }, out);
			},
			"output_label"_a, "mass"_a, "radius"_a, "full_height"_a, "slices"_a = 16, "stacks"_a = 1, "contact_params"_a = ContactParams())
		.def("add_torus", [](RigidBodyPresets& self, const std::string& output_label, double mass, double outer_radius, double inner_radius, int slices, int stacks, const ContactParams& contact_params)
			{ 
				auto [V, C, H] = self.add_torus(output_label, mass, outer_radius, inner_radius, slices, stacks, contact_params);
				nb::tuple out = nb::make_tuple(stark_to_nb(V), stark_to_nb(C), H);
				return create_named_tuple("RigidBodyOutput", { "vertices", "segments", "handler" }, out);
			},
			"output_label"_a, "mass"_a, "radius"_a, "full_height"_a, "slices"_a = 16, "stacks"_a = 32, "contact_params"_a = ContactParams())
		;
}
