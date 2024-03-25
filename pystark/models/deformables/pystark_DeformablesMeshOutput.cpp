#include "../../nanobind_stark_include_all.h"

void pystark_DeformablesMeshOutput(nb::module_& m)
{
    nb::class_<DeformablesMeshOutput>(m, "DeformablesMeshOutput")
        .def("add_point_set", [](DeformablesMeshOutput& self, const std::string& label, PH& set)
            { return self.add_point_set(label, set); },
            "label"_a, "set"_a)
        .def("add_point_set", [](DeformablesMeshOutput& self, const std::string& label, PH& set, VecXi& points)
            { return self.add_point_set(label, set, nb_to_stark(points)); },
            "label"_a, "set"_a, "points"_a)
        .def("add_segment_mesh", [](DeformablesMeshOutput& self, const std::string& label, PH& set, MatX2i& segments)
            { return self.add_segment_mesh(label, set, nb_to_stark(segments)); },
            "label"_a, "set"_a, "segments"_a)
        .def("add_segment_mesh", [](DeformablesMeshOutput& self, const std::string& label, PH& set, MatX2i& segments, VecXi& point_set_map)
            { return self.add_segment_mesh(label, set, nb_to_stark(segments), nb_to_stark(point_set_map)); },
            "label"_a, "set"_a, "segments"_a, "point_set_map"_a)
        .def("add_triangle_mesh", [](DeformablesMeshOutput& self, const std::string& label, PH& set, MatX3i& triangles)
            { return self.add_triangle_mesh(label, set, nb_to_stark(triangles)); },
            "label"_a, "set"_a, "triangles"_a)
        .def("add_triangle_mesh", [](DeformablesMeshOutput& self, const std::string& label, PH& set, MatX3i& triangles, VecXi& point_set_map)
            { return self.add_triangle_mesh(label, set, nb_to_stark(triangles), nb_to_stark(point_set_map)); },
            "label"_a, "set"_a, "triangles"_a, "point_set_map"_a)
        .def("add_tet_mesh", [](DeformablesMeshOutput& self, const std::string& label, PH& set, MatX4i& tets)
            { return self.add_tet_mesh(label, set, nb_to_stark(tets)); },
            "label"_a, "set"_a, "tets"_a)
        .def("add_tet_mesh", [](DeformablesMeshOutput& self, const std::string& label, PH& set, MatX4i& tets, VecXi& point_set_map)
            { return self.add_tet_mesh(label, set, nb_to_stark(tets), nb_to_stark(point_set_map)); },
            "label"_a, "set"_a, "tets"_a, "point_set_map"_a)
        ;
}