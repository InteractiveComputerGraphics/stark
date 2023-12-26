#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>

#include "../../core/Stark.h"
#include "Id.h"
#include "PointDynamics.h"

namespace stark::models
{
class EnergyTriangleBendingGrinspun03
{
public:
    /* Fields */
    const spPointDynamics dyn;
    symx::LabelledConnectivity<6> conn{ {"idx", "group", "v_edge_0", "v_edge_1", "v_opp_0", "v_opp_1"} };
    std::vector<double> stiffness; // per group (k_B)
    std::vector<double> rest_angle; // per hinge
    std::vector<double> rest_edge_length; // per hinge
    std::vector<double> rest_height; // per hinge

    /* Methods */
    EnergyTriangleBendingGrinspun03(stark::core::Stark& stark, spPointDynamics dyn);
    void add(Id& id, const std::vector<std::array<int, 3>>& triangles, double stiffness);
    void set_parameters(Id& id, double stiffness);
};
using spEnergyTriangleBendingGrinspun03 = std::shared_ptr<EnergyTriangleBendingGrinspun03>;
}
