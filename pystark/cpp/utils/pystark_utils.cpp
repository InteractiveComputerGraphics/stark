#include "../nanobind_stark_include_all.h"

auto return_segment_line(const Mesh<2>& m)
{
    nb::tuple out = nb::make_tuple(stark_to_nb(m.vertices), stark_to_nb(m.conn));
    return create_named_tuple("SegmentLine", { "vertices", "segments" }, out);
}
auto return_triangle_mesh(const Mesh<3>& m)
{
    nb::tuple out = nb::make_tuple(stark_to_nb(m.vertices), stark_to_nb(m.conn));
    return create_named_tuple("TriangleMesh", { "vertices", "triangles" }, out);
}
auto return_tet_mesh(const Mesh<4>& m)
{
    nb::tuple out = nb::make_tuple(stark_to_nb(m.vertices), stark_to_nb(m.conn));
    return create_named_tuple("TetMesh", { "vertices", "tets" }, out);
}

void pystark_utils_impl(nb::module_& m)
{
    // blends.h
    nb::enum_<stark::BlendType>(m, "BlendType")
        .value("Instant", BlendType::Instant)
        .value("Linear", BlendType::Linear)
        .value("EaseIn", BlendType::EaseIn)
        .value("EaseOut", BlendType::EaseOut)
        .value("EaseInOut", BlendType::EaseInOut)
        ;
    m.def("blend", &blend);

    // mesh_generators.h
    m.def("make_sphere", [](const double radius, const int subdivisions) { return return_triangle_mesh(make_sphere(radius, subdivisions)); }, "radius"_a, "subdivisions"_a = 2);
    m.def("make_box", [](const Eigen::Vector3d& size) { return return_triangle_mesh(make_box(size)); }, "size"_a);
    m.def("make_box", [](const double size) { return return_triangle_mesh(make_box(size)); }, "size"_a);
    m.def("make_cylinder", [](const double radius, const double full_height, const int slices, const int stacks) { return return_triangle_mesh(make_cylinder(radius, full_height, slices, stacks)); }, "radius"_a, "full_height"_a, "slices"_a = 16, "stacks"_a = 1);
    m.def("make_torus", [](const double outer_radius, const double inner_radius, const int slices, const int stacks) { return return_triangle_mesh(make_torus(outer_radius, inner_radius, slices, stacks)); }, "radius"_a, "full_height"_a, "slices"_a = 16, "stacks"_a = 8);
    m.def("make_knot", [](const double size, const double inner_radius, const int slices, const int stacks) { return return_triangle_mesh(make_knot(size, inner_radius, slices, stacks)); }, "size"_a, "full_height"_a, "slices"_a = 16, "stacks"_a = 8);
    m.def("generate_triangle_grid", [](const Eigen::Vector2d& center, const Eigen::Vector2d& dimensions, const std::array<int, 2>& n_quads_per_dim, const double z) { return return_triangle_mesh(generate_triangle_grid(center, dimensions, n_quads_per_dim, z)); }, "center"_a, "dimensions"_a, "n_quads_per_dim"_a, "z"_a = 0.0);
    m.def("generate_tet_grid", [](const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const std::array<int, 3>& n_quads_per_dim) { return return_tet_mesh(generate_tet_grid(center, dimensions, n_quads_per_dim)); }, "center"_a, "dimensions"_a, "n_quads_per_dim"_a);
    m.def("generate_segment_line", [](const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments) { return return_segment_line(generate_segment_line(begin, end, n_segments)); }, "begin"_a, "end"_a, "n_segments"_a);

    // mesh_utils.h
    m.def("deg2rad", &deg2rad);
    m.def("rad2deg", &rad2deg);
    m.def("triangle_normal", &triangle_normal);
    m.def("triangle_area", &triangle_area);
    m.def("unsigned_tetra_volume", &unsigned_tetra_volume);
    m.def("signed_tetra_volume", &signed_tetra_volume);

    m.def("find_edges_from_triangles", [](MatX3i& triangles, const int n_vertices) { return stark_to_nb(find_edges_from_simplices(nb_to_stark(triangles), n_vertices)); }, "triangles"_a, "n_vertices"_a);
    m.def("find_perimeter_edges", [](MatX3i& triangles, const int n_vertices)
        { 
            auto [edges, edge_to_triangle_map] = find_perimeter_edges(nb_to_stark(triangles), n_vertices);
            nb::tuple out = nb::make_tuple(stark_to_nb(edges), stark_to_nb(edge_to_triangle_map));
            return create_named_tuple("FindPerimeterEdgesOutput", { "edges", "edge_to_triangle_map" }, out);
        }, 
        "triangles"_a, "n_vertices"_a);
    m.def("find_sharp_edges", [](MatX3d& vertices, MatX3i& triangles, double angle_deg_threshold)
        { 
            auto [edges, edge_to_triangle_map] = find_sharp_edges(nb_to_stark(vertices), nb_to_stark(triangles), nb_to_stark(vertices).size());
            nb::tuple out = nb::make_tuple(stark_to_nb(edges), stark_to_nb(edge_to_triangle_map));
            return create_named_tuple("FindSharpEdgesOutput", { "edges", "edge_to_triangle_map" }, out);
        }, 
        "vertices"_a, "triangles"_a, "angle_deg_threshold"_a);
    m.def("find_surface", [](MatX3d& vertices, MatX4i& tets)
        { 
            auto [triangles, tri_to_tet_map] = find_surface(nb_to_stark(vertices), nb_to_stark(tets));
            nb::tuple out = nb::make_tuple(stark_to_nb(triangles), stark_to_nb(tri_to_tet_map));
            return create_named_tuple("FindSurfaceOutput", { "triangles", "tri_to_tet_map" }, out);
        }, 
        "vertices"_a, "tets"_a);
    m.def("clean_triangle_mesh", [](MatX3d& vertices, MatX3i& triangles, double merge_by_distance)
        { 
            auto [vertices_out, triangles_out] = clean_triangle_mesh(nb_to_stark(vertices), nb_to_stark(triangles), merge_by_distance);
            nb::tuple out = nb::make_tuple(stark_to_nb(vertices_out), stark_to_nb(triangles_out));
            return create_named_tuple("TriangleMesh", { "vertices", "triangles" }, out);
        }, 
        "vertices"_a, "triangles"_a, "merge_by_distance"_a = 0.0);


    // Transformations
    m.def("move", [](MatX3d& vertices, const Eigen::Vector3d& translation) 
        { 
            auto vertices_ = nb_to_stark(vertices);
            move(vertices_, translation);
            return stark_to_nb(vertices_);
        }, 
        "vertices"_a, "translation"_a);
    m.def("rotate_deg", [](MatX3d& vertices, const double angle_deg, const Eigen::Vector3d& axis)
        { 
            auto vertices_ = nb_to_stark(vertices);
            rotate_deg(vertices_, angle_deg, axis);
            return stark_to_nb(vertices_);
        }, 
        "vertices"_a, "angle_deg"_a, "axis"_a);
    m.def("rotate_deg_with_pivot", [](MatX3d& vertices, const double angle_deg, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot)
        { 
            auto vertices_ = nb_to_stark(vertices);
            rotate_deg(vertices_, angle_deg, axis, pivot);
            return stark_to_nb(vertices_);
        }, 
        "vertices"_a, "angle_deg"_a, "axis"_a, "pivot"_a);
    m.def("scale", [](MatX3d& vertices, const Eigen::Vector3d& scale_factor)
        { 
            auto vertices_ = nb_to_stark(vertices);
            scale(vertices_, scale_factor);
            return stark_to_nb(vertices_);
        }, 
        "vertices"_a, "scale_factor"_a);
    m.def("scale", [](MatX3d& vertices, const double scale_factor)
        { 
            auto vertices_ = nb_to_stark(vertices);
            scale(vertices_, scale_factor);
            return stark_to_nb(vertices_);
        }, 
        "vertices"_a, "scale_factor"_a);
    m.def("mirror", [](MatX3d& vertices, const int dim, const double pivot)
        { 
            auto vertices_ = nb_to_stark(vertices);
            mirror(vertices_, dim, pivot);
            return stark_to_nb(vertices_);
        }, 
        "vertices"_a, "dim"_a, "pivot"_a = 0.0);
}
