#pragma once
#include "acfly_perception_common/cloud_window.hpp"
namespace multi_pole_detector {
using acfly::perception::Points;
std::vector<std::vector<int>> cluster_xy(const Points& pts, double tolerance);
}  // namespace multi_pole_detector
