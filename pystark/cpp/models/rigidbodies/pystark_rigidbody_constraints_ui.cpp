#include "../../nanobind_stark_include_all.h"


#define DEFINE_HEAD() \
    .def("is_valid", &Self::is_valid) \
    .def("get_label", &Self::get_label) \
    .def("set_label", &Self::set_label) \
    .def("enable", &Self::enable) \

#define DEFINE_HEAD_ONE_RB() \
    DEFINE_HEAD() \
    .def("get_body", &Self::get_body) \

#define DEFINE_HEAD_TWO_RB() \
    DEFINE_HEAD() \
    .def("get_body_a", &Self::get_body_a) \
    .def("get_body_b", &Self::get_body_b) \

#define DEFINE(NAME) \
    .def(#NAME, &Self::NAME) \
	
#define DEFINE_GETTER_SETTER(NAME) \
    .def("get_##NAME", &Self::get_##NAME) \
    .def("set_##NAME", &Self::set_##NAME) \

void pystark_rigidbody_constraints_ui(nb::module_& m)
{
    /* =======================================================  BASE CONSTRAINTS  ======================================================= */
    {
        using Self = RBCGlobalPointHandler;
        nb::class_<Self>(m, "RBCGlobalPointHandler")
            DEFINE_HEAD_ONE_RB()
            DEFINE_GETTER_SETTER(stiffness)
            DEFINE_GETTER_SETTER(tolerance_in_m)
            DEFINE_GETTER_SETTER(global_target_point)
            DEFINE_GETTER_SETTER(local_point)
            .def("get_violation_in_m_and_force", &Self::get_violation_in_m_and_force);
    }
    {
        using Self = RBCGlobalDirectionHandler;
        nb::class_<Self>(m, "RBCGlobalDirectionHandler")
            DEFINE_HEAD_ONE_RB()
            DEFINE_GETTER_SETTER(stiffness)
            DEFINE_GETTER_SETTER(tolerance_in_deg)
            DEFINE_GETTER_SETTER(global_target_direction)
            DEFINE_GETTER_SETTER(local_direction)
            .def("get_violation_in_deg_and_torque", &Self::get_violation_in_deg_and_torque)
            .def("set_rotation", nb::overload_cast<const Eigen::Matrix3d&>(&Self::set_rotation))
            .def("set_rotation", nb::overload_cast<const double, const Eigen::Vector3d&>(&Self::set_rotation));
    }
    {
        using Self = RBCPointHandler;
        nb::class_<Self>(m, "RBCPointHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE_GETTER_SETTER(stiffness)
            DEFINE_GETTER_SETTER(tolerance_in_m)
            DEFINE_GETTER_SETTER(local_point_body_a)
            DEFINE_GETTER_SETTER(local_point_body_b)
            .def("get_violation_in_m_and_force", &Self::get_violation_in_m_and_force);
    }
    {
        using Self = RBCPointOnAxisHandler;
        nb::class_<Self>(m, "RBCPointOnAxisHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE_GETTER_SETTER(stiffness)
            DEFINE_GETTER_SETTER(tolerance_in_m)
            DEFINE_GETTER_SETTER(local_point_body_a)
            DEFINE_GETTER_SETTER(local_direction_body_a)
            DEFINE_GETTER_SETTER(local_point_body_b)
            .def("get_violation_in_m_and_force", &Self::get_violation_in_m_and_force);
    }
    {
        using Self = RBCDistanceHandler;
        nb::class_<Self>(m, "RBCDistanceHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE_GETTER_SETTER(stiffness)
            DEFINE_GETTER_SETTER(tolerance_in_m)
            DEFINE_GETTER_SETTER(local_point_body_a)
            DEFINE_GETTER_SETTER(local_point_body_b)
            DEFINE_GETTER_SETTER(target_distance)
            .def("get_violation_in_m_and_force", &Self::get_signed_violation_in_m_and_force);
    }
    {
        using Self = RBCDistanceLimitHandler;
        nb::class_<Self>(m, "RBCDistanceLimitHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE_GETTER_SETTER(stiffness)
            DEFINE_GETTER_SETTER(tolerance_in_m)
            DEFINE_GETTER_SETTER(local_point_body_a)
            DEFINE_GETTER_SETTER(local_point_body_b)
            DEFINE_GETTER_SETTER(min_distance)
            DEFINE_GETTER_SETTER(max_distance)
            .def("get_signed_violation_in_m_and_force", &Self::get_signed_violation_in_m_and_force);
    }
    {
        using Self = RBCDirectionHandler;
        nb::class_<Self>(m, "RBCDirectionHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE_GETTER_SETTER(stiffness)
            DEFINE_GETTER_SETTER(tolerance_in_deg)
            DEFINE_GETTER_SETTER(local_direction_body_a)
            DEFINE_GETTER_SETTER(local_direction_body_b)
            .def("get_violation_in_deg_and_torque", &Self::get_violation_in_deg_and_torque)
            .def("set_opening_angle_rad", &Self::set_opening_angle_rad)
            .def("set_opening_angle_deg", &Self::set_opening_angle_deg);
    }
    {
        using Self = RBCAngleLimitHandler;
        nb::class_<Self>(m, "RBCAngleLimitHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE_GETTER_SETTER(stiffness)
            DEFINE_GETTER_SETTER(tolerance_in_deg)
            DEFINE_GETTER_SETTER(local_direction_body_a)
            DEFINE_GETTER_SETTER(local_direction_body_b)
            DEFINE_GETTER_SETTER(limit_angle_in_deg)
            .def("get_violation_in_deg_and_torque", &Self::get_violation_in_deg_and_torque);
    }
    {
        using Self = RBCDampedSpringHandler;
        nb::class_<Self>(m, "RBCDampedSpringHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE_GETTER_SETTER(stiffness)
            DEFINE_GETTER_SETTER(local_point_body_a)
            DEFINE_GETTER_SETTER(local_point_body_b)
            DEFINE_GETTER_SETTER(rest_length)
            DEFINE_GETTER_SETTER(damping)
            .def("get_signed_spring_displacement_in_m_and_force", &Self::get_signed_spring_displacement_in_m_and_force)
            .def("get_signed_damper_velocity_and_force", &Self::get_signed_damper_velocity_and_force);
    }
    {
        using Self = RBCLinearVelocityHandler;
        nb::class_<Self>(m, "RBCLinearVelocityHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE_GETTER_SETTER(local_direction_body_a)
            DEFINE_GETTER_SETTER(target_velocity)
            DEFINE_GETTER_SETTER(max_force)
            DEFINE_GETTER_SETTER(delay)
            .def("get_signed_velocity_violation_and_force", &Self::get_signed_velocity_violation_and_force);
    }
    {
        using Self = RBCAngularVelocityHandler;
        nb::class_<Self>(m, "RBCAngularVelocityHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE_GETTER_SETTER(local_direction_body_a)
            DEFINE_GETTER_SETTER(target_angular_velocity_in_rad_per_s)
            DEFINE_GETTER_SETTER(target_angular_velocity_in_deg_per_s)
            DEFINE_GETTER_SETTER(max_torque)
            DEFINE_GETTER_SETTER(delay)
            .def("get_signed_angular_velocity_violation_in_deg_per_s_and_torque", &Self::get_signed_angular_velocity_violation_in_deg_per_s_and_torque);
    }


    /* =======================================================  DERIVED CONSTRAINTS  ======================================================= */
    {
        using Self = RBCFixHandler;
        nb::class_<Self>(m, "RBCFixHandler")
            DEFINE_HEAD()
            DEFINE(get_body)
            DEFINE(get_anchor_point)
            DEFINE(get_z_lock)
            DEFINE(get_x_lock)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m)
            DEFINE(set_tolerance_in_deg)
            DEFINE(set_tolerance_in_deg)
            .def("set_transformation", nb::overload_cast<const Eigen::Vector3d&, const Eigen::Matrix3d&>(&Self::set_transformation),
                "translation"_a, "rotation"_a = Eigen::Matrix3d::Identity())
            .def("set_transformation", nb::overload_cast<const Eigen::Vector3d&, const double, const Eigen::Vector3d&>(&Self::set_transformation),
                "translation"_a, "angle_deg"_a, "axis"_a);
    }
    {
        using Self = RBCAttachmentHandler;
        nb::class_<Self>(m, "RBCAttachmentHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE(get_point_constraint)
            DEFINE(get_z_lock)
            DEFINE(get_x_lock)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m)
            DEFINE(set_tolerance_in_deg);
    }
    {
        using Self = RBCPointWithAngleLimitHandler;
        nb::class_<Self>(m, "RBCPointWithAngleLimitHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE(get_point_constraint)
            DEFINE(get_angle_limit)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m)
            DEFINE(set_tolerance_in_deg);
    }
    {
        using Self = RBCHingeJointHandler;
        nb::class_<Self>(m, "RBCHingeJointHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE(get_point_constraint)
            DEFINE(get_direction_constraint)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m)
            DEFINE(set_tolerance_in_deg);
    }
    {
        using Self = RBCHingeJointWithAngleLimitHandler;
        nb::class_<Self>(m, "RBCHingeJointWithAngleLimitHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE(get_hinge_joint)
            DEFINE(get_angle_limit)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m)
            DEFINE(set_tolerance_in_deg);
    }
    {
        using Self = RBCSpringWithLimitsHandler;
        nb::class_<Self>(m, "RBCSpringWithLimitsHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE(get_spring)
            DEFINE(get_distance_limit)
            DEFINE(set_spring_stiffness)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m);
    }
    {
        using Self = RBCSliderHandler;
        nb::class_<Self>(m, "RBCSliderHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE(get_point_on_axis)
            DEFINE(get_direction_lock)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m)
            DEFINE(set_tolerance_in_deg);
    }
    {
        using Self = RBCPrismaticSliderHandler;
        nb::class_<Self>(m, "RBCPrismaticSliderHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE(get_slider)
            DEFINE(get_direction_lock)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m)
            DEFINE(set_tolerance_in_deg);
    }
    {
        using Self = RBCPrismaticPressHandler;
        nb::class_<Self>(m, "RBCPrismaticPressHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE(get_prismatic_slider)
            DEFINE(get_linear_velocity_constraint)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m)
            DEFINE(set_tolerance_in_deg);
    }
    {
        using Self = RBCMotorHandler;
        nb::class_<Self>(m, "RBCMotorHandler")
            DEFINE_HEAD_TWO_RB()
            DEFINE(get_hinge_joint)
            DEFINE(get_angular_velocity_constraint)
            DEFINE(set_stiffness)
            DEFINE(set_tolerance_in_m)
            DEFINE(set_tolerance_in_deg);
    }
}