#include "../../nanobind_stark_include_all.h"

void pystark_RigidBodyHandler(nb::module_& m)
{
    nb::class_<RigidBodyHandler>(m, "RigidBodyHandler")
        .def("get_idx", &RigidBodyHandler::get_idx)
        .def("is_valid", &RigidBodyHandler::is_valid)
        .def("exit_if_not_valid", &RigidBodyHandler::exit_if_not_valid)
        .def("get_label", &RigidBodyHandler::get_label)
        .def("set_label", &RigidBodyHandler::set_label)
        .def("get_translation", &RigidBodyHandler::get_translation)
        .def("set_translation", &RigidBodyHandler::set_translation)
        .def("add_translation", &RigidBodyHandler::add_translation)
        .def("get_quaternion", &RigidBodyHandler::get_quaternion)
        .def("get_rotation_matrix", &RigidBodyHandler::get_rotation_matrix)
        .def("set_rotation", nb::overload_cast<const Eigen::Quaterniond&>(&RigidBodyHandler::set_rotation), "quaternion"_a)
        .def("set_rotation", nb::overload_cast<double, const Eigen::Vector3d&>(&RigidBodyHandler::set_rotation), "angle_deg"_a, "axis"_a)
        .def("add_rotation", nb::overload_cast<const Eigen::Quaterniond&>(&RigidBodyHandler::add_rotation), "quaternion"_a)
        .def("add_rotation", nb::overload_cast<double, const Eigen::Vector3d&, const Eigen::Vector3d&>(&RigidBodyHandler::add_rotation), "angle_deg"_a, "axis"_a, "pivot"_a = Eigen::Vector3d::Zero())
        .def("get_velocity", &RigidBodyHandler::get_velocity)
        .def("get_velocity_at", &RigidBodyHandler::get_velocity_at)
        .def("set_velocity", &RigidBodyHandler::set_velocity)
        .def("add_velocity", &RigidBodyHandler::add_velocity)
        .def("get_angular_velocity", &RigidBodyHandler::get_angular_velocity)
        .def("set_angular_velocity", &RigidBodyHandler::set_angular_velocity)
        .def("add_angular_velocity", &RigidBodyHandler::add_angular_velocity)
        .def("get_force", &RigidBodyHandler::get_force)
        .def("add_force_at", &RigidBodyHandler::add_force_at, "force_glob"_a, "application_point_glob"_a)
        .def("set_force_at_centroid", &RigidBodyHandler::set_force_at_centroid)
        .def("add_force_at_centroid", &RigidBodyHandler::add_force_at_centroid)
        .def("get_torque", &RigidBodyHandler::get_torque)
        .def("set_torque", &RigidBodyHandler::set_torque)
        .def("add_torque", &RigidBodyHandler::add_torque)
        .def("get_acceleration", &RigidBodyHandler::get_acceleration)
        .def("set_acceleration", &RigidBodyHandler::set_acceleration)
        .def("add_acceleration", &RigidBodyHandler::add_acceleration)
        .def("get_angular_acceleration", &RigidBodyHandler::get_angular_acceleration)
        .def("set_angular_acceleration", &RigidBodyHandler::set_angular_acceleration)
        .def("add_angular_acceleration", &RigidBodyHandler::add_angular_acceleration)

        .def("transform_local_to_global_point", &RigidBodyHandler::transform_local_to_global_point)
        .def("transform_local_to_global_direction", &RigidBodyHandler::transform_local_to_global_direction)
        .def("transform_local_to_global_matrix", &RigidBodyHandler::transform_local_to_global_matrix)
        .def("transform_global_to_local_point", &RigidBodyHandler::transform_global_to_local_point)
        .def("transform_global_to_local_direction", &RigidBodyHandler::transform_global_to_local_direction)
        .def("transform_global_to_local_matrix", &RigidBodyHandler::transform_global_to_local_matrix)

        .def("get_mass", &RigidBodyHandler::get_mass)
        .def("set_mass", &RigidBodyHandler::set_mass)
        .def("get_local_inertia_tensor", &RigidBodyHandler::get_local_inertia_tensor)
        .def("get_global_inertia_tensor", &RigidBodyHandler::get_global_inertia_tensor)
        .def("set_local_inertia_tensor", &RigidBodyHandler::set_local_inertia_tensor)
        .def("get_linear_damping", &RigidBodyHandler::get_linear_damping)
        .def("set_linear_damping", &RigidBodyHandler::set_linear_damping)
        .def("get_angular_damping", &RigidBodyHandler::get_angular_damping)
        .def("set_angular_damping", &RigidBodyHandler::set_angular_damping)
        ;
}