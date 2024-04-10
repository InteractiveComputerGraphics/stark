#include "../../../nanobind_stark_include_all.h"

void pystark_EnergyLumpedInertia(nb::module_& m)
{
    using Self = EnergyLumpedInertia;
    using Params = Self::Params;
    using Handler = Self::Handler;

    auto model = nb::class_<EnergyLumpedInertia>(m, "EnergyLumpedInertia")
        .def("add", [](Self& self, PH& set, VecXi& points, VecXd& lumped_volume, const Params& params)
            { return self.add(set, nb_to_stark(points), nb_to_stark(lumped_volume), params); },
            "set"_a, "points"_a, "lumped_volume"_a, "params"_a)
        .def("add", [](Self& self, PH& set, VecXd& lumped_volume, const Params& params)
            { return self.add(set, nb_to_stark(lumped_volume), params); },
            "set"_a, "lumped_volume"_a, "params"_a)
        .def("add", [](Self& self, PH& set, MatX2i& edges, const Params& params)
            { return self.add(set, nb_to_stark(edges), params); },
            "set"_a, "edges"_a, "params"_a)
        .def("add", [](Self& self, PH& set, MatX3i& triangles, const Params& params)
            { return self.add(set, nb_to_stark(triangles), params); },
            "set"_a, "triangles"_a, "params"_a)
        .def("add", [](Self& self, PH& set, MatX4i& tets, const Params& params)
            { return self.add(set, nb_to_stark(tets), params); },
            "set"_a, "tets"_a, "params"_a)
        .def("get_params", &Self::get_params)
        .def("set_params", &Self::set_params)
        ;

    BIND_PARAMS(Params) PARAM(density) PARAM(damping);
    BIND_HANDLER(Handler)
		.def("get_mass", &Handler::get_mass)
		;
}
