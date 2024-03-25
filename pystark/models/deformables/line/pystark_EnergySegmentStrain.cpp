#include "../../../nanobind_stark_include_all.h"

void pystark_EnergySegmentStrain(nb::module_& m)
{
    using Self = EnergySegmentStrain;
    using Params = Self::Params;
    using Handler = Self::Handler;

    auto model = nb::class_<EnergySegmentStrain>(m, "EnergySegmentStrain")
        .def("add", [](Self& self, PH& set, MatX2i& segments, const Params& params)
            { return self.add(set, nb_to_stark(segments), params); },
            "set"_a, "tets"_a, "params"_a)
        .def("get_params", &Self::get_params)
        .def("set_params", &Self::set_params)
        ;

    BIND_PARAMS(Params) PARAM(elasticity_only) PARAM(scale) PARAM(section_radius) PARAM(youngs_modulus) PARAM(damping) PARAM_STRAIN_LIMITING();
    BIND_HANDLER(Handler);
}
