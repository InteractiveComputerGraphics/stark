#include "../../nanobind_stark_include_all.h"

void pystark_Interactions(nb::module_& m)
{
    nb::class_<Interactions>(m, "Interactions")
        .def("attachments", [](Interactions& self) { return self.attachments.get(); }, nb::rv_policy::reference_internal)
        .def("contact", [](Interactions& self) { return self.contact.get(); }, nb::rv_policy::reference_internal)
        ;
}
