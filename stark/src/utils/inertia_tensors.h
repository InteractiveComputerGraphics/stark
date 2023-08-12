#pragma once

#include <Eigen/Dense>

namespace stark::utils {

    Eigen::Matrix3d inertia_tensor_sphere(double mass, double radius);
    Eigen::Matrix3d inertia_tensor_cylinder(double mass, double radius, double height);
    Eigen::Matrix3d inertia_tensor_box(double mass, double side);
    Eigen::Matrix3d inertia_tensor_box(double mass, const Eigen::Vector3d& size);
    Eigen::Matrix3d inertia_tensor_torus(double mass, double outer_radius, double inner_radius);
}
