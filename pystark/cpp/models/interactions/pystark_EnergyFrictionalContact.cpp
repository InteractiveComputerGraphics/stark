#include "../../nanobind_stark_include_all.h"

void pystark_EnergyFrictionalContact(nb::module_& m)
{
    using Self = EnergyFrictionalContact;
    using Params = Self::Params;
    using GlobalParams = Self::GlobalParams;
    using Handler = Self::Handler;

    auto model = nb::class_<EnergyFrictionalContact>(m, "EnergyFrictionalContact")
        .def("add_triangles", [](Self& self, PH& set, MatX3i& triangles, const Params& params)
            { return self.add_triangles(set, nb_to_stark(triangles), params); },
            "set"_a, "triangles"_a, "params"_a)
        .def("add_triangles", [](Self& self, PH& set, MatX3i& triangles, VecXi& point_set_map, const Params& params)
            { return self.add_triangles(set, nb_to_stark(triangles), nb_to_stark(point_set_map), params); },
            "set"_a, "triangles"_a, "point_set_map"_a, "params"_a)
        .def("add_edges", [](Self& self, PH& set, MatX2i& edges, const Params& params)
            { return self.add_edges(set, nb_to_stark(edges), params); },
            "set"_a, "edges"_a, "params"_a)
        .def("add_edges", [](Self& self, PH& set, MatX2i& edges, VecXi& point_set_map, const Params& params)
            { return self.add_edges(set, nb_to_stark(edges), nb_to_stark(point_set_map), params); },
            "set"_a, "edges"_a, "point_set_map"_a, "params"_a)

        .def("add_triangles", [](Self& self, RBH& rb, MatX3d& vertices, MatX3i& triangles, const Params& params)
            { return self.add_triangles(rb, nb_to_stark(vertices), nb_to_stark(triangles), params); },
            "rb"_a, "vertices"_a, "triangles"_a, "params"_a)
        .def("add_edges", [](Self& self, RBH& rb, MatX3d& vertices, MatX2i& edges, const Params& params)
            { return self.add_edges(rb, nb_to_stark(vertices), nb_to_stark(edges), params); },
            "rb"_a, "vertices"_a, "edges"_a, "params"_a)

        .def("get_global_params", &Self::get_global_params)
        .def("set_global_params", &Self::set_global_params)
        .def("get_contact_stiffness", &Self::get_contact_stiffness)
        .def("set_contact_thickness", &Self::set_contact_thickness)
        .def("set_friction", &Self::set_friction)
        .def("disable_collision", &Self::disable_collision)
        .def("is_empty", &Self::is_empty)
        ;

    BIND_PARAMS(Params) 
        PARAM(contact_thickness)
        ;
    BIND_PARAMS(GlobalParams) 
        PARAM_(GlobalParams, default_contact_thickness) 
        PARAM_(GlobalParams, min_contact_stiffness)
        PARAM_(GlobalParams, friction_stick_slide_threshold)
        PARAM_(GlobalParams, collisions_enabled)
        PARAM_(GlobalParams, friction_enabled)
        PARAM_(GlobalParams, triangle_point_enabled)
        PARAM_(GlobalParams, edge_edge_enabled)
        PARAM_(GlobalParams, intersection_test_enabled)
        ;
    nb::class_<Handler>(model, "Handler")
        .def("get_idx", &Self::Handler::get_idx)
        .def("set_contact_thickness", &Self::Handler::set_contact_thickness)
        .def("set_friction", &Self::Handler::set_friction)
        .def("disable_collision", &Self::Handler::disable_collision)
        .def("is_valid", &Self::Handler::is_valid)
        .def("exit_if_not_valid", &Self::Handler::exit_if_not_valid)
        ;
}
