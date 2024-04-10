#include "../../nanobind_stark_include_all.h"

void pystark_RigidBodyDynamics(nb::module_& m)
{
    nb::class_<RigidBodyDynamics>(m, "RigidBodyDynamics")
        .def("add", &RigidBodyDynamics::add, "label"_a)
        .def("get_n_bodies", &RigidBodyDynamics::get_n_bodies)
        ;
    ;
}