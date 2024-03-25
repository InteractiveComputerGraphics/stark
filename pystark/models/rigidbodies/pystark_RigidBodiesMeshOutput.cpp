#include "../../nanobind_stark_include_all.h"

void pystark_RigidBodiesMeshOutput(nb::module_& m)
{
    nb::class_<RigidBodiesMeshOutput>(m, "RigidBodiesMeshOutput")
        .def("add_point_mesh", [](RigidBodiesMeshOutput& self, const std::string& label, RBH& rb, MatX3d& vertices)
            { return self.add_point_mesh(label, rb, nb_to_stark(vertices)); },
            "label"_a, "rb"_a, "vertices"_a)
        .def("add_segment_mesh", [](RigidBodiesMeshOutput& self, const std::string& label, RBH& rb, MatX3d& vertices, MatX2i& conn)
            { return self.add_segment_mesh(label, rb, nb_to_stark(vertices), nb_to_stark(conn)); },
            "label"_a, "rb"_a, "vertices"_a, "conn"_a)
        .def("add_triangle_mesh", [](RigidBodiesMeshOutput& self, const std::string& label, RBH& rb, MatX3d& vertices, MatX3i& conn)
            { return self.add_triangle_mesh(label, rb, nb_to_stark(vertices), nb_to_stark(conn)); },
            "label"_a, "rb"_a, "vertices"_a, "conn"_a)
        .def("add_tet_mesh", [](RigidBodiesMeshOutput& self, const std::string& label, RBH& rb, MatX3d& vertices, MatX4i& conn)
            { return self.add_tet_mesh(label, rb, nb_to_stark(vertices), nb_to_stark(conn)); },
            "label"_a, "rb"_a, "vertices"_a, "conn"_a)
        ;
}