#include "../../../nanobind_stark_include_all.h"

void pystark_EnergyTetStrain(nb::module_& m)
{
    using Self = EnergyTetStrain;
    using Params = Self::Params;
    using Handler = Self::Handler;

    auto model = nb::class_<EnergyTetStrain>(m, "EnergyTetStrain")
        .def("add", [](Self& self, PH& set, MatX4i& tets, const Params& params)
            { return self.add(set, nb_to_stark(tets), params); },
            "set"_a, "tets"_a, "params"_a)
        .def("get_params", &Self::get_params)
        .def("set_params", &Self::set_params)
        ;

    BIND_PARAMS(Params) PARAM(elasticity_only) PARAM(scale) PARAM(youngs_modulus) PARAM(poissons_ratio) PARAM(damping) PARAM_STRAIN_LIMITING();
    BIND_HANDLER(Handler);
}
