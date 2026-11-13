#pragma once
#include "acfly_perception_common/cloud_window.hpp"
namespace ring_detector {
bool fit_circle(const acfly::perception::Points& pts, Eigen::Vector3d& center, double& radius,
                Eigen::Vector3d& e1, Eigen::Vector3d& e2);
}
