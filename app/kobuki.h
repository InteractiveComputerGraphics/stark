#pragma once

#include <Eigen/Dense>
#include <stark>

struct Kobuki {};

Kobuki make_kobuki(stark::models::Simulation& sim);

void kobuki_test();
