#include "exploration_node.hpp"

namespace exploration {

void ExplorationNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Validate before constructing rclcpp::Time, which throws on negative stamps.
  if ((!msg->header.frame_id.empty() && msg->header.frame_id != "camera_init") ||
      msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "[探索定位] 忽略坐标系或时间戳无效的里程计帧");
    return;
  }
  const auto &p = msg->pose.pose.position;
  const auto &q = msg->pose.pose.orientation;
  const double received_at = steady_seconds();

  double yaw = 0.0;
  const double qn = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(qn) ||
      qn <= 1e-9)
    return;
  if (qn > 1e-9) {
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tq);
    double r, pi;
    m.getRPY(r, pi, yaw);
  }

  const rclcpp::Time stamp = msg->header.stamp;

  std::lock_guard<std::mutex> lk(mtx_);
  if (!pose_freshness_.observe(stamp.nanoseconds(), received_at))
    return;
  // 位置差分估速度（local 系），再转机体系给 tracker 当 D 项参考
  //   (2026-09-23 试过卡尔曼常速度滤波：实机级噪声下速度误差只降 20%，无噪声时滞后是 EMA 的
  //    2.6 倍，归航末段实测速度方向滞后→刹停投影偏出末端直线→反复急刹。保留 EMA。)
  if (has_prev_pose_) {
    const double dt = pose_freshness_.source_interval();
    if (dt > 1e-3) {
      const double vx = (p.x - prev_x_) / dt;
      const double vy = (p.y - prev_y_) / dt;
      // local → 机体：前进 = 投影到机头方向，横向 = 投影到左方向
      const double c = std::cos(yaw), s = std::sin(yaw);
      const double vf = c * vx + s * vy;
      const double vl = -s * vx + c * vy;
      v_fwd_est_ = v_est_alpha_ * vf + (1.0 - v_est_alpha_) * v_fwd_est_;
      v_lat_est_ = v_est_alpha_ * vl + (1.0 - v_est_alpha_) * v_lat_est_;
      const double wz = wrap_pi(yaw - prev_yaw_) / dt;
      yaw_rate_diff_ = v_est_alpha_ * wz + (1.0 - v_est_alpha_) * yaw_rate_diff_;
      // 雷达 IMU 自检：与 SLAM 航向差分长期反号，或幅值比偏离
      // [0.5,2](单位/坐标错误)，停用并退回差分。
      if (!gyro_disabled_ && gyro_fresh(received_at) && std::abs(yaw_rate_diff_) > 0.3 &&
          std::abs(gyro_z_) > 0.3) {
        gyro_agreement_ =
            0.97 * gyro_agreement_ + 0.03 * ((yaw_rate_diff_ > 0) == (gyro_z_ > 0) ? 1.0 : 0.0);
        gyro_ratio_ =
            0.97 * gyro_ratio_ + 0.03 * std::clamp(std::abs(gyro_z_ / yaw_rate_diff_), 0.0, 5.0);
        if (++gyro_agreement_samples_ >= 40 &&
            (gyro_agreement_ < 0.3 || gyro_ratio_ < 0.5 || gyro_ratio_ > 2.0)) {
          gyro_disabled_ = true;
          RCLCPP_ERROR(get_logger(),
                       "[雷达IMU] %s 与 SLAM 航向变化不一致(同号率 %.2f，幅值比 "
                       "%.2f)，已停用，退回 SLAM 航向差分",
                       imu_topic_.c_str(), gyro_agreement_, gyro_ratio_);
        }
      }
    } else if (dt == 0.0) {
      v_fwd_est_ = v_lat_est_ = yaw_rate_diff_ = 0.0;
    }
  }
  yaw_rate_est_ = current_yaw_rate(received_at);
  prev_x_ = p.x;
  prev_y_ = p.y;
  prev_yaw_ = yaw;
  has_prev_pose_ = true;

  px_ = p.x;
  py_ = p.y;
  pz_ = p.z;
  yaw_ = yaw;
  const double qn_sqrt = std::sqrt(qn);
  imu_attitude_ = {q.w / qn_sqrt, q.x / qn_sqrt, q.y / qn_sqrt, q.z / qn_sqrt};
  has_pose_ = true;
}

void ExplorationNode::on_mapping_clock(const rosgraph_msgs::msg::Clock &msg) {
  if (msg.clock.sec < 0 || msg.clock.nanosec >= 1000000000u)
    return;
  std::lock_guard<std::mutex> lk(mtx_);
  mapping_clock_ns_ = rclcpp::Time(msg.clock).nanoseconds();
  mapping_clock_received_at_ = steady_seconds();
}

void ExplorationNode::on_flight_state(const mavros_msgs::msg::State &state) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!state.connected || !state.armed || obstacle_mapping_start_ns_)
    return;
  // 首帧到达前无法确定传感器时间域；等下一次已解锁状态再保守开启。
  // 包括节点在飞机已解锁后启动、且 /clock 尚未到达的情况。
  if (!last_obstacle_cloud_stamp_ns_)
    return;
  // 单次启动：重复状态、上锁/再解锁均不清除本次进程已建立的地图。
  const int64_t ros_now = now().nanoseconds();
  // Gazebo/Point-LIO 保留仿真时间（几秒/几百秒），ros2 run 默认使用系统时间。
  // 不得把两个时间域直接比较，也不能去掉门限放入解锁前排队扫描。
  // 同域实机保持原判据；异域必须等新鲜且与点云同域的 /clock，再开始累计。
  const bool different_clock = last_obstacle_cloud_stamp_ns_ &&
                               std::abs(static_cast<double>(*last_obstacle_cloud_stamp_ns_) -
                                        ros_now) > clock_domain_gap_s_ * 1e9;
  const bool fresh_clock =
      mapping_clock_ns_ && steady_seconds() - mapping_clock_received_at_ < clock_timeout_s_;
  const bool matches_cloud = fresh_clock && last_obstacle_cloud_stamp_ns_ &&
                             std::abs(static_cast<double>(*mapping_clock_ns_) -
                                      *last_obstacle_cloud_stamp_ns_) < clock_cloud_gap_s_ * 1e9;
  if (different_clock && !matches_cloud) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "[探索建图] 点云与系统时间不同域，等待同域 /clock；检查仿真时钟桥接");
    return;
  }
  obstacle_mapping_start_ns_ = different_clock ? *mapping_clock_ns_ : ros_now;
  RCLCPP_INFO(get_logger(), "[探索建图] 已检测到飞机解锁，开始累计解锁后的障碍点云（%s时间）",
              different_clock ? "仿真" : "ROS");
}

void ExplorationNode::on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const CloudXYZ xyz(*msg);
  if (!xyz.valid() || (!msg->header.frame_id.empty() && msg->header.frame_id != "camera_init")) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "[探索点云] 忽略 XYZ 结构或坐标系不正确的点云");
    return;
  }
  // An empty packet supplies no free-space evidence for obstacle removal.
  if (msg->width == 0 || msg->height == 0)
    return;
  double px, py, yaw, wz;
  bool ok;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u)
      return;
    last_obstacle_cloud_stamp_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
    // 门限与点云同一时间域：拒绝解锁前扫描，接受解锁后的仿真/实机数据。
    if (!obstacle_mapping_start_ns_ || *last_obstacle_cloud_stamp_ns_ < *obstacle_mapping_start_ns_)
      return;
    px = px_;
    py = py_;
    yaw = yaw_;
    wz = yaw_rate_est_;
    ok = has_pose_ && pose_freshness_.fresh(steady_seconds(), pose_timeout_);
    if (!ok)
      return;
  }

  // ★高角速度门控★：飞机快速旋转时点云拖影(点被甩到障碍外侧)会污染累积、被 latch 钉成虚胖大圆。
  //   整帧丢弃(不累积、不 latch)；旋转停下后正常帧补回真实边界。阈值<=0 关闭。
  if (obs_skip_yaw_rate_ > 0.0 && std::fabs(wz) > obs_skip_yaw_rate_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "旋转过快(|yaw_rate|=%.2f>%.2f rad/s)→丢弃本帧点云防拖影污染",
                         std::fabs(wz), obs_skip_yaw_rate_);
    return;
  }

  const double self_r = robot_radius_ + self_margin_;
  const double self_r2 = self_r * self_r;

  std::vector<Vec2> pts;
  pts.reserve(static_cast<size_t>(msg->width) * msg->height / 2 + 1);

  bool has_finite_return = false;
  xyz.for_each([&](float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      return;
    has_finite_return = true;
    if (z < ground_z_)
      return; // 高度窗口·下限：地面/近地杂物滤掉
    if (z > ceil_z_)
      return; // 高度窗口·上限：天花板/高处墙面/吊挂物滤掉
    if (ok) {
      const double dx = x - px, dy = y - py;
      if (dx * dx + dy * dy < self_r2)
        return; // 自身回波：滤掉
    }
    pts.push_back({static_cast<double>(x), static_cast<double>(y)});
  });

  // 地面回波可能全部被高度窗口滤掉；它仍是有效采集。全是 NaN/Inf 的包则没有任何测量。
  if (!has_finite_return)
    return;
  // The cloud group is mutually exclusive; a duplicate packet is not new occupancy evidence.
  if (!input_cloud_sequence_.observe(rclcpp::Time(msg->header.stamp).nanoseconds(),
                                     steady_seconds()))
    return;
  obs_map_->integrate(pts, px, py, yaw, ok);
  std::lock_guard<std::mutex> lk(mtx_);
  cloud_freshness_.observe(rclcpp::Time(msg->header.stamp).nanoseconds(), steady_seconds());
}

} // namespace exploration
