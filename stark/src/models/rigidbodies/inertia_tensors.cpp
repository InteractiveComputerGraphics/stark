#include "inertia_tensors.h"

Eigen::Matrix3d stark::inertia_tensor_sphere(double mass, double radius)
{
    double I = (2.0 / 5.0) * mass * radius * radius;
    Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity() * I;
    return inertia_tensor;
}
Eigen::Matrix3d stark::inertia_tensor_cylinder(double mass, double radius, double full_height)
{
    const double height = full_height;
    double Ix = (1.0 / 12.0) * mass * (3 * radius * radius + height * height);
    double Iy = (1.0 / 12.0) * mass * (3 * radius * radius + height * height);
    double Iz = (1.0 / 2.0) * mass * radius * radius;

    Eigen::Matrix3d inertia_tensor;
    inertia_tensor << Ix, 0, 0,
        0, Iy, 0,
        0, 0, Iz;
    return inertia_tensor;
}
Eigen::Matrix3d stark::inertia_tensor_box(double mass, double size)
{
    double I = (1.0 / 6.0) * mass * size * size;
    Eigen::Matrix3d inertia_tensor;
    inertia_tensor << I, 0, 0,
        0, I, 0,
        0, 0, I;
    return inertia_tensor;
}
Eigen::Matrix3d stark::inertia_tensor_box(double mass, const Eigen::Vector3d& size)
{
    const double length = size.x();
    const double width = size.y();
    const double height = size.z();

    double Ix = (1.0 / 12.0) * mass * (width * width + height * height);
    double Iy = (1.0 / 12.0) * mass * (length * length + height * height);
    double Iz = (1.0 / 12.0) * mass * (length * length + width * width);

    Eigen::Matrix3d inertia_tensor;
    inertia_tensor << Ix, 0, 0,
        0, Iy, 0,
        0, 0, Iz;
    return inertia_tensor;
}
Eigen::Matrix3d stark::inertia_tensor_torus(double mass, double b, double a)
{
    double a2 = a*a;
    double b2 = b*b;
    double Ix = (1.0 / 8.0) * mass * (5.0*a2 + 4.0*b2);
    double Iy = (1.0 / 8.0) * mass * (5.0*a2 + 4.0*b2);
    double Iz = (1.0 / 4.0) * mass * (4.0*b2 + 3.0*a2);

    Eigen::Matrix3d inertia_tensor;
    inertia_tensor << Ix, 0, 0,
        0, Iy, 0,
        0, 0, Iz;
    return inertia_tensor;
}