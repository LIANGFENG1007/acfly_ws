#pragma once

#include <optional>

#include "acfly_perception_common/cloud_window.hpp"

namespace acfly::perception {

struct Pole {
  double x, y, z, radius;
  int id = -1;
};

// Solve the horizontal circle and take the median elevation of the observed points.
std::optional<Pole> fit_pole(const Points& points);
std::optional<Pole> fit_pole(const Points& points, const std::vector<int>& indices);

}  // namespace acfly::perception
