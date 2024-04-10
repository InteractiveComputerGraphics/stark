#include "../../../nanobind_stark_include_all.h"

void pystark_EnergyTriangleStrain(nb::module_& m)
{
    using Self = EnergyTriangleStrain;
    using Params = Self::Params;
    using Handler = Self::Handler;

    auto model = nb::class_<EnergyTriangleStrain>(m, "EnergyTriangleStrain")
        .def("add", [](Self& self, PH& set, MatX3i& triangles, const Params& params)
            { return self.add(set, nb_to_stark(triangles), params); },
            "set"_a, "triangles"_a, "params"_a)
        .def("get_params", &Self::get_params)
        .def("set_params", &Self::set_params)
        ;

    BIND_PARAMS(Params) PARAM(elasticity_only) PARAM(scale) PARAM(thickness) PARAM(youngs_modulus) PARAM(poissons_ratio) PARAM(damping) PARAM_STRAIN_LIMITING() PARAM(inflation);
    BIND_HANDLER(Handler);
}
