#pragma once

#include <Eigen/Core>
#include <deque>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

namespace acfly::perception {

using Points = std::vector<Eigen::Vector3f>;
struct Region {
  double min_x, max_x, min_y, max_y, min_z, max_z;
};

class CloudWindow {
 public:
  CloudWindow(Region region, int capacity);
  // Throws std::invalid_argument for malformed clouds. The previous window remains intact.
  Points update(const sensor_msgs::msg::PointCloud2& cloud);

 private:
  Region region_;
  size_t capacity_;
  std::deque<Points> frames_;
};

sensor_msgs::msg::PointCloud2 make_cloud(const Points& points, const std_msgs::msg::Header& header);

}  // namespace acfly::perception
