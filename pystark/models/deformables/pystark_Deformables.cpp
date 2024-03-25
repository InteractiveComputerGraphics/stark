#include "../../nanobind_stark_include_all.h"

void pystark_Deformables(nb::module_& m)
{
    nb::class_<Deformables>(m, "Deformables")
        .def("output", [](Deformables& self) { return self.output.get(); }, nb::rv_policy::reference_internal)
        .def("point_sets", [](Deformables& self) { return self.point_sets.get(); }, nb::rv_policy::reference_internal)
        .def("lumped_inertia", [](Deformables& self) { return self.lumped_inertia.get(); }, nb::rv_policy::reference_internal)
        .def("prescribed_positions", [](Deformables& self) { return self.prescribed_positions.get(); }, nb::rv_policy::reference_internal)
        .def("segment_strain", [](Deformables& self) { return self.segment_strain.get(); }, nb::rv_policy::reference_internal)
        .def("triangle_strain", [](Deformables& self) { return self.triangle_strain.get(); }, nb::rv_policy::reference_internal)
        .def("discrete_shells", [](Deformables& self) { return self.discrete_shells.get(); }, nb::rv_policy::reference_internal)
        .def("tet_strain", [](Deformables& self) { return self.tet_strain.get(); }, nb::rv_policy::reference_internal)
        ;
}
