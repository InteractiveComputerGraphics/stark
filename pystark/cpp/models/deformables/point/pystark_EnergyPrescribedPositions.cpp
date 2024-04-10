#include "../../../nanobind_stark_include_all.h"

void pystark_EnergyPrescribedPositions(nb::module_& m)
{
    using Self = EnergyPrescribedPositions;
    using Params = Self::Params;
    using Handler = Self::Handler;

    auto model = nb::class_<EnergyPrescribedPositions>(m, "EnergyPrescribedPositions")
        .def("add", [](Self& self, PH& set, VecXi& points, const Params& params)
            { return self.add(set, nb_to_stark(points), params); },
            "set"_a, "points"_a, "params"_a)
        .def("add_inside_aabb", &Self::add_inside_aabb, "set"_a, "aabb_center"_a, "aabb_dim"_a, "params"_a)
        .def("add_outside_aabb", &Self::add_outside_aabb, "set"_a, "aabb_center"_a, "aabb_dim"_a, "params"_a)
        .def("get_params", &Self::get_params)
        .def("set_params", &Self::set_params)
        .def("set_transformation", nb::overload_cast<const Handler&, const Eigen::Vector3d&, const Eigen::Matrix3d&>(&Self::set_transformation)
            , "handler"_a, "t"_a, "R"_a)
        .def("set_transformation", nb::overload_cast<const Handler&, const Eigen::Vector3d&, const double, const Eigen::Vector3d&>(&Self::set_transformation)
			, "handler"_a, "t"_a, "angle_deg"_a, "axis"_a)
        ;

    BIND_PARAMS(Params) PARAM(stiffness) PARAM(tolerance);
    BIND_HANDLER(Handler)
        .def("set_transformation", nb::overload_cast<const Eigen::Vector3d&, const Eigen::Matrix3d&>(&Handler::set_transformation),
            "t"_a, "R"_a = Eigen::Matrix3d::Identity())
        .def("set_transformation", nb::overload_cast<const Eigen::Vector3d&, const double, const Eigen::Vector3d&>(&Handler::set_transformation),
            "t"_a, "angle_deg"_a, "axis"_a)
        .def("set_target_position", &Handler::set_target_position, "prescribed_idx"_a, "position"_a)
        ;
}
