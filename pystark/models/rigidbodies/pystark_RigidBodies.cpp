#include "../../nanobind_stark_include_all.h"

void pystark_RigidBodies(nb::module_& m)
{
    nb::class_<RigidBodies>(m, "RigidBodies")
        .def("add", &RigidBodies::add)

        .def("set_default_constraint_stiffness", &RigidBodies::set_default_constraint_stiffness)
        .def("set_default_constraint_distance_tolerance", &RigidBodies::set_default_constraint_distance_tolerance)
        .def("set_default_constraint_angle_tolerance", &RigidBodies::set_default_constraint_angle_tolerance)
        .def("get_default_constraint_stiffness", &RigidBodies::get_default_constraint_stiffness)
        .def("get_default_constraint_distance_tolerance", &RigidBodies::get_default_constraint_distance_tolerance)
        .def("get_default_constraint_angle_tolerance", &RigidBodies::get_default_constraint_angle_tolerance)

        .def("add_constraint_global_point", &RigidBodies::add_constraint_global_point)
        .def("add_constraint_global_direction", &RigidBodies::add_constraint_global_direction)
        .def("add_constraint_point", &RigidBodies::add_constraint_point)
        .def("add_constraint_point_on_axis", &RigidBodies::add_constraint_point_on_axis)
        .def("add_constraint_distance", &RigidBodies::add_constraint_distance)
        .def("add_constraint_distance_limits", &RigidBodies::add_constraint_distance_limits)
        .def("add_constraint_direction", &RigidBodies::add_constraint_direction)
        .def("add_constraint_angle_limit", &RigidBodies::add_constraint_angle_limit)
        .def("add_constraint_spring", &RigidBodies::add_constraint_spring)
        .def("add_constraint_linear_velocity", &RigidBodies::add_constraint_linear_velocity)
        .def("add_constraint_angular_velocity", &RigidBodies::add_constraint_angular_velocity)

        .def("add_constraint_fix", &RigidBodies::add_constraint_fix)
        .def("add_constraint_attachment", &RigidBodies::add_constraint_attachment)
        .def("add_constraint_point_with_angle_limit", &RigidBodies::add_constraint_point_with_angle_limit)
        .def("add_constraint_hinge", &RigidBodies::add_constraint_hinge)
        .def("add_constraint_hinge_with_angle_limit", &RigidBodies::add_constraint_hinge_with_angle_limit)
        .def("add_constraint_spring_with_limits", &RigidBodies::add_constraint_spring_with_limits)
        .def("add_constraint_slider", &RigidBodies::add_constraint_slider)
        .def("add_constraint_prismatic_slider", &RigidBodies::add_constraint_prismatic_slider)
        .def("add_constraint_prismatic_press", &RigidBodies::add_constraint_prismatic_press)
        .def("add_constraint_motor", &RigidBodies::add_constraint_motor)
        ;
}