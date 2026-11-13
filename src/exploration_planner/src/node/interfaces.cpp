#include "exploration_node.hpp"

namespace exploration {

void ExplorationNode::connect_interfaces() {
  flight_state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10),
      [this](const mavros_msgs::msg::State::SharedPtr msg) { on_flight_state(*msg); });
  RCLCPP_INFO(get_logger(), "[探索建图] 等待飞机解锁后开始累计障碍物");
  control_pub_ = create_publisher<acfly_interfaces::msg::ExplorationCommand>("/exploration/control",
                                                                             rclcpp::QoS(1));
  context_sub_ = create_subscription<acfly_interfaces::msg::ExplorationContext>(
      "/exploration/context", rclcpp::QoS(1).transient_local(),
      [this](const acfly_interfaces::msg::ExplorationContext::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!std::isfinite(msg->home_z) || !std::isfinite(msg->cruise_z) ||
            !std::isfinite(msg->height_tolerance) || msg->height_tolerance <= 0 ||
            msg->mission.stamp.sec < 0 || msg->mission.stamp.nanosec >= 1000000000u ||
            msg->mission.frame_id != "camera_init")
          return;
        // Don't replace the active mission's baseline with a late old context.
        if (context_ && rclcpp::Time(msg->mission.stamp) < rclcpp::Time(context_->mission.stamp))
          return;
        context_ = *msg;
      });
  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/exploration/cmd_vel", 10);

  rclcpp::QoS latched(1);
  latched.transient_local();
  finished_pub_ = create_publisher<std_msgs::msg::Bool>("/exploration/finished", latched);
  finished_goal_pub_ =
      create_publisher<std_msgs::msg::Header>("/exploration/finished_goal", latched);
  coverage_pub_ = create_publisher<std_msgs::msg::Float64>("/exploration/coverage", 1);
  diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/exploration/diagnostics", 1);
  active_path_pub_ = create_publisher<nav_msgs::msg::Path>("/exploration/active_path", 1);
  // 处理后障碍点云(rviz 看"算法当障碍的点")+ 设定边界框(rviz 画 FIELD_* 四面墙)。
  obs_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/exploration/obstacle_cloud", 5);
  boundary_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("/exploration/field_boundary", latched);
  if (imu_topic_.find("mavros") != std::string::npos) {
    // 实机 /mavros/imu/data 数据不正确：即便被参数误配也拒绝订阅。
    RCLCPP_ERROR(get_logger(),
                 "[雷达IMU] 拒绝订阅 %s(飞控 IMU 数据不可信)，角速度改用 SLAM 航向差分",
                 imu_topic_.c_str());
    imu_topic_.clear();
  }
  if (!imu_topic_.empty()) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          const auto &w = msg->angular_velocity;
          if (!std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z))
            return;
          std::lock_guard<std::mutex> lk(mtx_);
          if (!has_pose_)
            return;
          // IMU 系角速度经 SLAM 姿态(即 IMU 系在 camera_init 下的姿态)转到世界系，取 z =
          // 航向角速度。
          const auto &q = imu_attitude_;
          const double r20 = 2.0 * (q[1] * q[3] - q[0] * q[2]);
          const double r21 = 2.0 * (q[2] * q[3] + q[0] * q[1]);
          const double r22 = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
          gyro_z_ = r20 * w.x + r21 * w.y + r22 * w.z;
          gyro_at_ = steady_seconds();
        });
    RCLCPP_INFO(get_logger(),
                "[雷达IMU] 航向角速度取自 %s(自检不通过或断流时自动退回 SLAM 航向差分)",
                imu_topic_.c_str());
  }
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
      std::bind(&ExplorationNode::on_odom, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/exploration/goal", latched,
      std::bind(&ExplorationNode::on_goal, this, std::placeholders::_1));

  // 仅供解锁建图时间门限使用；不改变主控/命令消息的 ROS 时钟。
  mapping_clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) { on_mapping_clock(*msg); });

  // 雷达点云订阅：独立 reentrant 回调组，后台聚类不被 20Hz timer 阻塞。
  cloud_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions cloud_opt;
  cloud_opt.callback_group = cloud_cbg_;
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", rclcpp::SensorDataQoS(),
      std::bind(&ExplorationNode::on_cloud, this, std::placeholders::_1), cloud_opt);
}

} // namespace exploration
