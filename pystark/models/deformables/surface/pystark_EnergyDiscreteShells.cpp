#include "../../../nanobind_stark_include_all.h"

void pystark_EnergyDiscreteShells(nb::module_& m)
{
    using Self = EnergyDiscreteShells;
    using Params = Self::Params;
    using Handler = Self::Handler;

    auto model = nb::class_<EnergyDiscreteShells>(m, "EnergyDiscreteShells")
        .def("add", [](Self& self, PH& set, MatX3i& triangles, const Params& params)
            { return self.add(set, nb_to_stark(triangles), params); },
            "set"_a, "triangles"_a, "params"_a)
        .def("get_params", &Self::get_params)
        .def("set_params", &Self::set_params)
        ;

    BIND_PARAMS(Params) PARAM(elasticity_only) PARAM(scale) PARAM(flat_rest_angle) PARAM(stiffness) PARAM(damping);
    BIND_HANDLER(Handler);
}
