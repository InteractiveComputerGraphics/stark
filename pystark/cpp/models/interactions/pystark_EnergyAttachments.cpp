#include "../../nanobind_stark_include_all.h"

void pystark_EnergyAttachments(nb::module_& m)
{
    using Self = EnergyAttachments;
    using Params = Self::Params;
    using Handler = Self::Handler;

    auto model = nb::class_<EnergyAttachments>(m, "EnergyAttachments")
        .def("add", [](Self& self, PH& set_0, PH& set_1, VecXi& points_0, VecXi& points_1, const Params& params)
            { return self.add(set_0, set_1, nb_to_stark(points_0), nb_to_stark(points_1), params); },
            "set_0"_a, "set_1"_a, "points_0"_a, "points_1"_a, "params"_a)
        .def("add", [](Self& self, PH& set_0, PH& set_1, VecXi& points, MatX2i& edges, MatX2d& bary, const Params& params)
            { return self.add(set_0, set_1, nb_to_stark(points), nb_to_stark(edges), nb_to_stark(bary), params); },
            "set_0"_a, "set_1"_a, "points"_a, "edges"_a, "bary"_a, "params"_a)
        .def("add", [](Self& self, PH& set_0, PH& set_1, VecXi& points, MatX3i& triangles, MatX3d& bary, const Params& params)
            { return self.add(set_0, set_1, nb_to_stark(points), nb_to_stark(triangles), nb_to_stark_no_eigen(bary), params); },
            "set_0"_a, "set_1"_a, "points"_a, "triangles"_a, "bary"_a, "params"_a)
        .def("add", [](Self& self, PH& set_0, PH& set_1, MatX2i& edges_0, MatX2i& edges_1, MatX2d& bary_0, MatX2d& bary_1, const Params& params)
            { return self.add(set_0, set_1, nb_to_stark(edges_0), nb_to_stark(edges_1), nb_to_stark(bary_0), nb_to_stark(bary_1), params); },
            "set_0"_a, "set_1"_a, "edges_0"_a, "edges_1"_a, "bary_0"_a, "bary_1"_a, "params"_a)
        .def("add_by_distance", [](Self& self, PH& set_0, PH& set_1, VecXi& points, MatX3i& triangles, const double distance, const Params& params)
            { return self.add_by_distance(set_0, set_1, nb_to_stark(points), nb_to_stark(triangles), distance, params); },
            "set_0"_a, "set_1"_a, "points"_a, "triangles"_a, "distance"_a, "params"_a)

        .def("add", [](Self& self, RBH& rb, PH& set, MatX3d& rb_points_loc, VecXi& set_points, const Params& params)
            { return self.add(rb, set, nb_to_stark(rb_points_loc), nb_to_stark(set_points), params); },
            "rb"_a, "set"_a, "rb_points_loc"_a, "set_points"_a, "params"_a)
        .def("add", [](Self& self, RBH& rb, PH& set, VecXi& set_points, const Params& params)
            { return self.add(rb, set, nb_to_stark(set_points), params); },
            "rb"_a, "set"_a, "set_points"_a, "params"_a)
        .def("add_by_distance", [](Self& self, RBH& rb, PH& set, MatX3d& rb_points_loc, MatX3i& rb_triangles, VecXi& points, const double distance, const Params& params)
            { return self.add_by_distance(rb, set, nb_to_stark(rb_points_loc), nb_to_stark(rb_triangles), nb_to_stark(points), distance, params); },
            "rb"_a, "set"_a, "rb_points_loc"_a, "rb_triangles"_a, "points"_a, "distance"_a, "params"_a)

        .def("get_params", &Self::get_params)
        .def("set_params", &Self::set_params)
        ;

    BIND_PARAMS(Params) PARAM(stiffness) PARAM(tolerance);
    BIND_HANDLER(Handler);
    nb::class_<Self::MultiHandler>(model, "MultiHandler")
        .def("get_params", &Self::MultiHandler::get_params)
        .def("set_params", &Self::MultiHandler::set_params)
        ;
}
