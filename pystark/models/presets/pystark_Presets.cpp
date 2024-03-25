#include "../../nanobind_stark_include_all.h"

void pystark_Presets(nb::module_& m)
{
    nb::class_<Presets>(m, "Presets")
        .def("deformables", [](Presets& self) { return self.deformables.get(); }, nb::rv_policy::reference_internal)
        .def("rigidbodies", [](Presets& self) { return self.rigidbodies.get(); }, nb::rv_policy::reference_internal)
        ;
}
