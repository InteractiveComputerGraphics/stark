#pragma once

#include <Eigen/Dense>
#include <stark>

void towel_parametrization();
void folding_towel();
void rolling_towel();
void kobuki_test();
void kobuki_v_towel(const std::string output_directory, const std::string name, const std::string mesh_path, const std::string kobuki_collision_path, const double floor_friction, const double rotation);
void kobuki_v_towel_suite();