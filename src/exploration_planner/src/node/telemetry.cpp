#include "exploration_node.hpp"

namespace exploration {

void ExplorationNode::publish_obstacle_cloud() {
  const std::vector<Vec2> pts = obs_map_->occupied_points();
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = now();
  msg.header.frame_id = "camera_init";
  msg.height = 1;
  msg.width = static_cast<uint32_t>(pts.size());
  msg.is_dense = true;
  msg.is_bigendian = false;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());
  sensor_msgs::PointCloud2Iterator<float> ox(msg, "x"), oy(msg, "y"), oz(msg, "z");
  for (const auto &p : pts) {
    *ox = static_cast<float>(p.x);
    *oy = static_cast<float>(p.y);
    *oz = 0.3f; // 抬到 0.3m 高度，rviz 里好看、不被地面盖住
    ++ox;
    ++oy;
    ++oz;
  }
  obs_cloud_pub_->publish(msg);
}

void ExplorationNode::publish_field_boundary() {
  visualization_msgs::msg::Marker m;
  m.header.stamp = now();
  m.header.frame_id = "camera_init";
  m.ns = "field_boundary";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.05; // 线宽 5cm
  m.color.r = 1.0f;
  m.color.g = 0.2f;
  m.color.b = 0.2f;
  m.color.a = 1.0f; // 红框
  m.pose.orientation.w = 1.0;
  const double x0 = gcfg_.min_x, y0 = gcfg_.min_y, x1 = gcfg_.max_x, y1 = gcfg_.max_y;
  auto add = [&](double x, double y) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    m.points.push_back(p);
  };
  add(x0, y0);
  add(x1, y0);
  add(x1, y1);
  add(x0, y1);
  add(x0, y0); // 闭合矩形
  boundary_pub_->publish(m);
}

void ExplorationNode::publish_status(const std::string &phase) {
  std_msgs::msg::Float64 coverage;
  coverage.data = grid_->coverage_ratio();
  nav_msgs::msg::Path active_path;
  active_path.header.stamp = now();
  active_path.header.frame_id = "camera_init";
  for (const auto &point : traj_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = active_path.header;
    pose.pose.position.x = point.p.x;
    pose.pose.position.y = point.p.y;
    pose.pose.position.z = pz_;
    pose.pose.orientation.w = 1.0;
    active_path.poses.push_back(pose);
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header = active_path.header;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "exploration/planner";
  status.hardware_id = "exploration_planner";
  status.message = phase;
  status.level = (phase == "waiting_for_pose" || phase == "waiting_for_cloud")
                     ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                     : diagnostic_msgs::msg::DiagnosticStatus::OK;
  auto value = [&status](const std::string &key, const std::string &text) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = text;
    status.values.push_back(item);
  };
  auto number = [&value](const std::string &key, auto scalar) {
    value(key, std::to_string(scalar));
  };

  value("mode", phase);
  value("recovery", recovery_status_);
  value("last_replan_reason", last_replan_reason_);
  value("blocked_reason", global_block_reason_);
  number("coverage", coverage.data);
  number("astar_searches", astar_searches_);
  number("obstacles", obs_map_->snapshot().size());
  diagnostics.status.push_back(status);
  coverage_pub_->publish(coverage);
  active_path_pub_->publish(active_path);
  diagnostics_pub_->publish(diagnostics);
  publish_obstacle_cloud();
  if (!boundary_sent_) {
    publish_field_boundary();
    boundary_sent_ = true;
  }
}

} // namespace exploration
