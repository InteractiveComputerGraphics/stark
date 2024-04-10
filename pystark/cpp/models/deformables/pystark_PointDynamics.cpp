#include "../../nanobind_stark_include_all.h"

void pystark_PointDynamics(nb::module_& m)
{
    nb::class_<PointDynamics>(m, "PointDynamics")
        .def("add", [](PointDynamics& self, MatX3d& x, const std::string& label) { return self.add(nb_to_stark(x), label); },
            "x"_a, "label"_a = "")
        .def("size", &PointDynamics::size)
        .def("get_set_size", &PointDynamics::get_set_size)
        .def("get_begin", &PointDynamics::get_begin)
        .def("get_end", &PointDynamics::get_end)
        .def("get_global_index", &PointDynamics::get_global_index, "set_idx"_a, "local_index"_a);
}
