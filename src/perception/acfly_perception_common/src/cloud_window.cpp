#include "acfly_perception_common/cloud_window.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>

namespace acfly::perception {
namespace {

size_t offset(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name && field.count == 1 &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        cloud.point_step >= sizeof(float) && field.offset <= cloud.point_step - sizeof(float)) {
      return field.offset;
    }
  }
  throw std::invalid_argument("Missing or invalid FLOAT32 field: " + name);
}

float read_float(const uint8_t* data, bool swap) {
  std::array<uint8_t, sizeof(float)> bytes;
  std::copy_n(data, bytes.size(), bytes.begin());
  if (swap) std::reverse(bytes.begin(), bytes.end());
  float value;
  std::memcpy(&value, bytes.data(), sizeof(value));
  return value;
}
}  // namespace

CloudWindow::CloudWindow(Region region, int capacity)
    : region_(region), capacity_(static_cast<size_t>(capacity)) {
  if (capacity < 1 || !(region.min_x < region.max_x && region.min_y < region.max_y &&
                        region.min_z < region.max_z)) {
    throw std::invalid_argument("accum_frames must be positive and each box_min must be < box_max");
  }
}

Points CloudWindow::update(const sensor_msgs::msg::PointCloud2& cloud) {
  if (uint64_t(cloud.point_step) * cloud.width > cloud.row_step ||
      uint64_t(cloud.row_step) * cloud.height != cloud.data.size()) {
    throw std::invalid_argument("Inconsistent PointCloud2 width, height, stride or data size");
  }
  const auto x_offset = offset(cloud, "x");
  const auto y_offset = offset(cloud, "y");
  const auto z_offset = offset(cloud, "z");
  const uint16_t one = 1;
  const bool swap = cloud.is_bigendian != (*reinterpret_cast<const uint8_t*>(&one) == 0);
  Points frame;
  // Rows in an organized cloud can have padding. Fields may be unaligned or reordered.
  for (size_t row = 0; row < cloud.height && cloud.width != 0; ++row) {
    for (size_t column = 0; column < cloud.width; ++column) {
      const auto* point = cloud.data.data() + row * cloud.row_step + column * cloud.point_step;
      const float x = read_float(point + x_offset, swap);
      const float y = read_float(point + y_offset, swap);
      const float z = read_float(point + z_offset, swap);
      if (x >= region_.min_x && x <= region_.max_x && y >= region_.min_y && y <= region_.max_y &&
          z >= region_.min_z && z <= region_.max_z) {
        frame.emplace_back(x, y, z);
      }
    }
  }
  frames_.push_back(std::move(frame));
  if (frames_.size() > capacity_) frames_.pop_front();
  size_t size = 0;
  for (const auto& stored : frames_) size += stored.size();
  Points combined;
  combined.reserve(size);
  for (const auto& stored : frames_) combined.insert(combined.end(), stored.begin(), stored.end());
  return combined;
}

sensor_msgs::msg::PointCloud2 make_cloud(const Points& points,
                                         const std_msgs::msg::Header& header) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  if (points.empty()) return cloud;
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& point : points) {
    *x = point.x();
    *y = point.y();
    *z = point.z();
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}
}  // namespace acfly::perception
