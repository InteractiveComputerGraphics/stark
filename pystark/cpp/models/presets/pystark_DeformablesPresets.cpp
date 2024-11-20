#include "../../nanobind_stark_include_all.h"

void pystark_DeformablesPresets(nb::module_& m)
{
    nb::class_<DeformablesPresets>(m, "DeformablesPresets")

        /* Methods */
        // Line
        .def("add_line", [](DeformablesPresets& self, const std::string& output_label, MatX3d& vertices, MatX2i& segments, const Line::Params& params)
            { return self.add_line(output_label, nb_to_stark(vertices), nb_to_stark(segments), params); },
            "output_label"_a, "vertices"_a, "segments"_a, "params"_a)
        .def("add_line_as_segments", [](DeformablesPresets& self, const std::string& output_label, const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments, const Line::Params& params)
            {
                auto [V, C, H] = self.add_line_as_segments(output_label, begin, end, n_segments, params);
                nb::tuple out = nb::make_tuple(stark_to_nb(V), stark_to_nb(C), H);
                return create_named_tuple("LineOutput", { "vertices", "segments", "handler" }, out);
            },
            "output_label"_a, "begin"_a, "end"_a, "n_segments"_a, "params"_a)

        // Surface
        .def("add_surface", [](DeformablesPresets& self, const std::string& output_label, MatX3d& vertices, MatX3i& triangles, const Surface::Params& params)
            { return self.add_surface(output_label, nb_to_stark(vertices), nb_to_stark(triangles), params); },
            "output_label"_a, "vertices"_a, "triangles"_a, "params"_a)
        .def("add_surface_grid", [](DeformablesPresets& self, const std::string& output_label, const Eigen::Vector2d& dim, const Eigen::Vector2i& subdivisions, const Surface::Params& params)
            {
                auto [V, C, H] = self.add_surface_grid(output_label, dim, { subdivisions[0], subdivisions[1] }, params);
                nb::tuple out = nb::make_tuple(stark_to_nb(V), stark_to_nb(C), H);
                return create_named_tuple("SurfaceOutput", { "vertices", "triangles", "handler" }, out);
            },
            "output_label"_a, "size"_a, "subdivisions"_a, "params"_a)
        .def("add_prescribed_surface", [](DeformablesPresets& self, const std::string& output_label, MatX3d& vertices, MatX3i& triangles, const PrescribedSurface::Params& params)
            { return self.add_prescribed_surface(output_label, nb_to_stark(vertices), nb_to_stark(triangles), params); },
            "output_label"_a, "vertices"_a, "triangles"_a, "params"_a)

        // Volume
        .def("add_volume", [](DeformablesPresets& self, const std::string& output_label, MatX3d& vertices, MatX4i& tets, const Volume::Params& params)
            { return self.add_volume(output_label, nb_to_stark(vertices), nb_to_stark(tets), params); },
            "output_label"_a, "vertices"_a, "tets"_a, "params"_a)
        .def("add_volume_grid", [](DeformablesPresets& self, const std::string& output_label, const Eigen::Vector3d& dim, const Eigen::Vector3i& sub, const Volume::Params& params)
            {
                auto [V, C, H] = self.add_volume_grid(output_label, dim, { sub[0], sub[1], sub[2] }, params);
                nb::tuple out = nb::make_tuple(stark_to_nb(V), stark_to_nb(C), H);
                return create_named_tuple("VolumeOutput", { "vertices", "tets", "handler" }, out);
            },
            "output_label"_a, "size"_a, "subdivisions"_a, "params"_a)


        /* Fields */
        .def("point_sets", [](Deformables& self) { return self.point_sets.get(); }, nb::rv_policy::reference_internal)
        .def("lumped_inertia", [](Deformables& self) { return self.lumped_inertia.get(); }, nb::rv_policy::reference_internal)
        .def("prescribed_positions", [](Deformables& self) { return self.prescribed_positions.get(); }, nb::rv_policy::reference_internal)
        .def("segment_strain", [](Deformables& self) { return self.segment_strain.get(); }, nb::rv_policy::reference_internal)
        .def("triangle_strain", [](Deformables& self) { return self.triangle_strain.get(); }, nb::rv_policy::reference_internal)
        .def("discrete_shells", [](Deformables& self) { return self.discrete_shells.get(); }, nb::rv_policy::reference_internal)
        .def("tet_strain", [](Deformables& self) { return self.tet_strain.get(); }, nb::rv_policy::reference_internal)
        ;
}
