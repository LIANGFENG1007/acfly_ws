#include "acfly_perception_common/cloud_window.hpp"

#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace acfly::perception;

void require(bool pass, const char* message) {
  if (!pass) throw std::runtime_error(message);
}

int main() {
  // Build a padded, big-endian organized cloud. Byte offsets are deliberately unaligned.
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 2;
  msg.width = 2;
  msg.point_step = 16;
  msg.row_step = 36;
  msg.is_bigendian = true;
  msg.data.resize(72);
  for (const auto& entry : {std::make_pair("z", 1), {"x", 5}, {"y", 9}}) {
    sensor_msgs::msg::PointField field;
    field.name = entry.first;
    field.offset = entry.second;
    field.count = 1;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields.push_back(field);
  }
  auto set = [&](size_t index, float x, float y, float z) {
    auto* ptr = msg.data.data() + index / 2 * msg.row_step + index % 2 * msg.point_step;
    const float values[] = {z, x, y};
    for (size_t i = 0; i < 3; ++i) {
      uint32_t bits;
      std::memcpy(&bits, &values[i], sizeof(bits));
      for (int b = 0; b < 4; ++b) ptr[msg.fields[i].offset + b] = bits >> (24 - b * 8);
    }
  };
  set(0, 0, 0, 0);
  set(1, -2, 0, 0);
  set(2, 1, 0, -1);
  set(3, std::numeric_limits<float>::quiet_NaN(), 0, 0);
  CloudWindow window({-1, 1, -1, 1, -1, 1}, 2);
  const auto first = window.update(msg);
  require(first.size() == 2 && first.back() == Eigen::Vector3f(1, 0, -1), "layout or crop");

  for (int kind = 0; kind < 4; ++kind) {
    auto bad = msg;
    switch (kind) {
      case 0:
        bad.data.pop_back();
        break;
      case 1:
        bad.fields[0].offset = bad.point_step;
        break;
      case 2:
        bad.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
        break;
      case 3:
        bad.fields.pop_back();
        break;
    }
    bool rejected = false;
    try {
      window.update(bad);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    require(rejected, "malformed cloud accepted");
  }
  const auto empty = make_cloud({}, msg.header);
  require(window.update(empty).size() == 2, "malformed frame changed history");
  require(window.update(empty).empty(), "old frame retained");
  require(window.update(make_cloud(first, msg.header)).size() == 2, "output cloud unreadable");
}
