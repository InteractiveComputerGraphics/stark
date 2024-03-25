#include "../../nanobind_stark_include_all.h"

void pystark_PointSetHandler(nb::module_& m)
{
    nb::class_<PointSetHandler>(m, "PointSetHandler")
        .def("get_idx", &PointSetHandler::get_idx)
        .def("is_valid", &PointSetHandler::is_valid)
        .def("exit_if_not_valid", &PointSetHandler::exit_if_not_valid)
        .def("set_label", &PointSetHandler::set_label)
        .def("get_label", &PointSetHandler::get_label)
        .def("get_begin", &PointSetHandler::get_begin)
        .def("get_end", &PointSetHandler::get_end)
        .def("size", &PointSetHandler::size)
        .def("get_global_index", &PointSetHandler::get_global_index)
        .def("all", [](PointSetHandler& self) { return stark_to_nb(self.all()); })

        .def("get_position", &PointSetHandler::get_position)
        .def("get_rest_position", &PointSetHandler::get_rest_position)
        .def("get_velocity", &PointSetHandler::get_velocity)
        .def("get_acceleration", &PointSetHandler::get_acceleration)
        .def("get_force", &PointSetHandler::get_force)

        .def("set_position", &PointSetHandler::set_position)
        .def("set_rest_position", &PointSetHandler::set_rest_position)
        .def("set_velocity", &PointSetHandler::set_velocity)
        .def("set_acceleration", &PointSetHandler::set_acceleration)
        .def("set_force", &PointSetHandler::set_force)

        .def("add_translation", &PointSetHandler::add_displacement, "displacement"_a, "also_at_rest_pose"_a = true, nb::rv_policy::reference_internal)
        .def("add_rotation", &PointSetHandler::add_rotation, "angle_deg"_a, "axis"_a, "pivot"_a = Eigen::Vector3d(0.0, 0.0, 0.0), "also_at_rest_pose"_a = true, nb::rv_policy::reference_internal)
        ;
}
