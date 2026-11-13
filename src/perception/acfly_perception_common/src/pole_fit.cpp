#include "acfly_perception_common/pole_fit.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>

namespace acfly::perception {
namespace {
template <typename PointAt>
std::optional<Pole> fit(size_t size, PointAt at) {
  if (size < 3) return std::nullopt;
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < size; ++i) mean += at(i).template head<2>().template cast<double>();
  mean /= static_cast<double>(size);

  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  std::vector<float> elevations;
  elevations.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    const auto& p = at(i);
    const double x = p.x() - mean.x(), y = p.y() - mean.y();
    const Eigen::Vector3d row(2.0 * x, 2.0 * y, 1.0);
    matrix += row * row.transpose();
    rhs += row * (x * x + y * y);
    elevations.push_back(p.z());
  }
  const Eigen::Vector3d solution = matrix.ldlt().solve(rhs);
  const double radius_squared = solution.z() + solution.head<2>().squaredNorm();
  if (!(radius_squared > 0) || !solution.allFinite()) return std::nullopt;
  const auto middle = elevations.begin() + elevations.size() / 2;
  std::nth_element(elevations.begin(), middle, elevations.end());
  return Pole{mean.x() + solution.x(), mean.y() + solution.y(), *middle, std::sqrt(radius_squared)};
}
}  // namespace

std::optional<Pole> fit_pole(const Points& points) {
  return fit(
      points.size(), [&](size_t i) -> const auto& { return points[i]; });
}
std::optional<Pole> fit_pole(const Points& points, const std::vector<int>& indices) {
  return fit(
      indices.size(), [&](size_t i) -> const auto& { return points[indices[i]]; });
}
}  // namespace acfly::perception
