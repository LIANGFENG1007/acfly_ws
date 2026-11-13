// Calls callbacks in process; publications are intercepted, no aircraft or simulator.
#include "../src/node/exploration_node.hpp"
#include <iostream>
#include <stdexcept>

static size_t commands = 0, finished = 0;
static acfly_interfaces::msg::ExplorationCommand last_command;
extern "C" rcl_ret_t __wrap_rcl_publish(const rcl_publisher_t *publisher, const void *message,
                                        rmw_publisher_allocation_t *) {
  const std::string topic = rcl_publisher_get_topic_name(publisher);
  if (topic == "/exploration/control") {
    ++commands;
    last_command = *static_cast<const acfly_interfaces::msg::ExplorationCommand *>(message);
  }
  if (topic == "/exploration/finished_goal")
    ++finished;
  return RCL_RET_OK;
}
namespace exploration {
struct PlannerContractTest {
  static void require(bool condition, const char *why) {
    if (!condition)
      throw std::runtime_error(why);
  }
  static void fresh_pose(ExplorationNode &n) {
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = n.now();
    odom->pose.pose.position.x = 1;
    odom->pose.pose.orientation.w = 1;
    n.on_odom(odom);
  }
  static void obstacle_mapping_starts_on_arming() {
    ExplorationNode node;
    node.timer_->cancel();
    fresh_pose(node);
    node.has_pose_ = true;
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = "camera_init";
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    const auto scan = [&](float x, int64_t stamp, int count = 3) {
      cloud->header.stamp = rclcpp::Time(stamp);
      *sensor_msgs::PointCloud2Iterator<float>(*cloud, "x") = x;
      *sensor_msgs::PointCloud2Iterator<float>(*cloud, "y") = 0.f;
      *sensor_msgs::PointCloud2Iterator<float>(*cloud, "z") = .8f;
      for (int i = 0; i < count; ++i) {
        // Repeated acquisitions carry successive source stamps.
        cloud->header.stamp = rclcpp::Time(stamp + i);
        node.on_cloud(cloud);
      }
    };
    const auto sees = [&](double x) {
      for (const auto &p : node.obs_map_->occupied_points())
        if (std::abs(p.x - x) < node.ocfg_.cell)
          return true;
      return false;
    };
    const int64_t before = node.now().nanoseconds();
    scan(2.f, before);
    mavros_msgs::msg::State state;
    state.armed = true;
    state.connected = false;
    node.on_flight_state(state);
    scan(2.f, before);
    state.armed = false;
    state.connected = true;
    node.on_flight_state(state);
    scan(2.f, before);
    require(!node.obstacle_mapping_start_ns_ && node.obs_map_->snapshot().empty() &&
                node.obs_map_->occupied_points().empty(),
            "pre-arm scans entered obstacle map");
    for (int i = 0; i < 2 * node.gcfg_.scan_dwell_ticks + 2; ++i)
      node.on_timer();
    require(node.grid_->coverage_ratio() == 0,
            "pre-arm empty obstacle map marked coverage as scanned");

    state.armed = true;
    state.mode = "MANUAL";
    node.on_flight_state(state);
    require(node.obstacle_mapping_start_ns_.has_value(),
            "arming did not enable mapping outside OFFBOARD");
    const int64_t start = *node.obstacle_mapping_start_ns_;
    scan(2.f, start - 10);
    require(node.obs_map_->occupied_points().empty(),
            "queued pre-arm scan entered map after arming");
    cloud->header.stamp = rclcpp::Time(start);
    for (const char *field : {"x", "y", "z"})
      *sensor_msgs::PointCloud2Iterator<float>(*cloud, field) =
          std::numeric_limits<float>::quiet_NaN();
    node.on_cloud(cloud);
    require(!node.cloud_freshness_.fresh(node.steady_seconds(), node.cloud_timeout_),
            "all-NaN scans renewed the sensor watchdog");
    *sensor_msgs::PointCloud2Iterator<float>(*cloud, "x") = 2.f;
    *sensor_msgs::PointCloud2Iterator<float>(*cloud, "y") = 0.f;
    *sensor_msgs::PointCloud2Iterator<float>(*cloud, "z") = static_cast<float>(node.ground_z_ - 1.);
    node.on_cloud(cloud);
    require(node.cloud_freshness_.fresh(node.steady_seconds(), node.cloud_timeout_) &&
                node.obs_map_->occupied_points().empty(),
            "ground-only scan failed validity or occupied a cell");
    scan(2.f, start + 1, 1);
    require(node.obs_map_->occupied_points().empty(),
            "pre-arm hits leaked into post-arm accumulation");
    scan(3.f, start + 1000000);
    require(!sees(2) && sees(3) && !node.obs_map_->snapshot().empty(),
            "moved obstacle did not map only at its post-arm position");
    scan(4.f, start + 2000000);
    require(sees(3) && sees(4), "map froze instead of continuing to accumulate after arming");
    fresh_pose(node);
    for (int i = 0; i < 2 * node.gcfg_.scan_dwell_ticks + 2; ++i)
      node.on_timer();
    require(node.grid_->coverage_ratio() > 0, "coverage did not start after arming");
    node.on_flight_state(state);
    state.armed = false;
    node.on_flight_state(state);
    state.connected = false;
    node.on_flight_state(state);
    state.armed = state.connected = true;
    node.on_flight_state(state);
    require(*node.obstacle_mapping_start_ns_ == start && sees(3) && sees(4),
            "repeated state/disarm/rearm cleared map or moved its start time");

    ExplorationNode already_armed;
    already_armed.timer_->cancel();
    already_armed.on_flight_state(state);
    require(!already_armed.obstacle_mapping_start_ns_, "selected clock before first scan");
    cloud->header.stamp = already_armed.now();
    already_armed.on_cloud(cloud);
    already_armed.on_flight_state(state);
    require(already_armed.obstacle_mapping_start_ns_.has_value(),
            "node started on an armed aircraft incorrectly required a disarmed transition");

    ExplorationNode sim;
    sim.timer_->cancel();
    fresh_pose(sim);
    sim.has_pose_ = true;
    const auto sim_scan = [&](float x, int64_t ns) {
      cloud->header.stamp = rclcpp::Time(ns);
      *sensor_msgs::PointCloud2Iterator<float>(*cloud, "x") = x;
      for (int i = 0; i < 3; ++i) {
        cloud->header.stamp = rclcpp::Time(ns + i);
        sim.on_cloud(cloud);
      }
    };
    // State can precede both the bridge and Point-LIO startup.
    sim.on_flight_state(state);
    require(!sim.obstacle_mapping_start_ns_, "early state locked simulator to wall time");
    sim_scan(2.f, 100000000000LL);
    sim.on_flight_state(state);
    require(!sim.obstacle_mapping_start_ns_,
            "mismatched timestamps silently accepted without clock");
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(100500000000LL);
    sim.on_mapping_clock(clock);
    sim.mapping_clock_received_at_ -= 2;
    sim.on_flight_state(state);
    require(!sim.obstacle_mapping_start_ns_, "stale simulation clock accepted");
    clock.clock = sim.now();
    sim.on_mapping_clock(clock);
    sim.on_flight_state(state);
    require(!sim.obstacle_mapping_start_ns_, "unrelated clock accepted");
    clock.clock = rclcpp::Time(100500000000LL);
    sim.on_mapping_clock(clock);
    sim.on_flight_state(state);
    require(sim.obstacle_mapping_start_ns_ == 100500000000LL,
            "simulator arming cutoff was not in sensor clock domain");
    sim_scan(2.f, 100499999990LL);
    require(sim.obs_map_->occupied_points().empty(), "queued simulator scan mapped before arming");
    sim_scan(3.f, 100550000000LL);
    require(!sim.obs_map_->occupied_points().empty(), "post-arm simulator obstacle invisible");
    for (const auto &p : sim.obs_map_->occupied_points())
      require(std::abs(p.x - 2) > sim.ocfg_.cell, "pre-arm simulator obstacle leaked into map");
    sim.on_flight_state(state);
    require(sim.obstacle_mapping_start_ns_ == 100500000000LL,
            "simulator cutoff moved on repeated state");
    std::cout << "simulation mapping: early state, missing/stale/unrelated clock, queued scan "
                 "rejection and post-arm obstacle passed\n";
    std::cout << "armed mapping: pre-arm/moved/queued scans, continued mapping and repeated states "
                 "passed\n";
  }
  static void control_contract() {
    ExplorationNode n;
    auto goal = std::make_shared<geometry_msgs::msg::PointStamped>();
    goal->header.frame_id = "camera_init";
    goal->header.stamp = n.now();
    goal->point.x = 1;
    n.on_goal(goal);
    fresh_pose(n);
    n.on_timer();
    require(commands == 0, "issued control without matching mission context");
    n.context_.emplace();
    n.context_->mission = goal->header;
    n.context_->cruise_z = 1.2;
    n.on_timer();
    require(commands == 1 && last_command.velocity.linear.x == 0 && !n.homing_,
            "missing cloud must hold instead of exploring or completing");
    n.obstacle_mapping_start_ns_ = 1;
    n.cloud_freshness_.observe(2, n.steady_seconds());
    n.on_timer();
    require(commands == 2 && !n.homing_ && finished == 0 && n.astar_searches_ > 0,
            "valid inputs must select coverage before returning");
    require(last_command.header.frame_id == "base_link" && last_command.mission == goal->header &&
                last_command.altitude == 1.2,
            "control header, mission id, or cruise height diverged");
    n.on_goal(goal);
    require(n.astar_searches_ > 0 && n.has_goal_, "retained goal replaced active mission");
    for (int i = 0; i < n.gcfg_.scan_dwell_ticks * 2; ++i)
      n.grid_->mark_scan(1, 0, 0, {});
    n.search_coverage_ = n.grid_->coverage_ratio();
    require(n.search_coverage_ > 0, "fixture did not cover any cells");
    n.on_timer();
    require(n.homing_ && n.finished_ && finished == 1, "settled exact endpoint not handed back");
    n.on_timer();
    require(finished == 1, "completion must be emitted once per mission");
  }
};
} // namespace exploration
int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    exploration::PlannerContractTest::obstacle_mapping_starts_on_arming();
    exploration::PlannerContractTest::control_contract();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
