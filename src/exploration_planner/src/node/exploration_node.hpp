#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "exploration_planner/bezier.hpp"
#include "exploration_planner/cloud_xyz.hpp"
#include "exploration_planner/coverage_planner.hpp"
#include "exploration_planner/global_planner.hpp"
#include "exploration_planner/grid_map.hpp"
#include "exploration_planner/home_approach.hpp"
#include "exploration_planner/obstacle_map.hpp"
#include "exploration_planner/route_rejoin.hpp"
#include "exploration_planner/sensor_freshness.hpp"
#include "exploration_planner/trajectory_tracker.hpp"
#include "exploration_planner/types.hpp"
#include "exploration_planner/velocity_slew.hpp"
#include <acfly_config/defaults.hpp>
#include <acfly_interfaces/msg/exploration_command.hpp>
#include <acfly_interfaces/msg/exploration_context.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

namespace exploration {
class ExplorationNode final : public rclcpp::Node {
public:
  ExplorationNode();

private:
  friend struct PlannerContractTest;
  acfly_config::Defaults defaults_{
      "exploration_planner",
      {"mapping.yaml", "coverage.yaml", "motion.yaml", "return.yaml", "runtime.yaml"}};
  int timer_period_ms_{}, diagnostics_every_{};
  double pose_timeout_{}, cloud_timeout_{}, cruise_vertical_limit_{};
  template <class T> T read(const std::string &name) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const T result = defaults_.declare<T>(*this, name, descriptor);
    if constexpr (std::is_floating_point_v<T>)
      if (!std::isfinite(result))
        throw std::invalid_argument(name + " must be finite");
    return result;
  }
  static double steady_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_mapping_clock(const rosgraph_msgs::msg::Clock &msg);
  void on_flight_state(const mavros_msgs::msg::State &state);
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool adopt_explore_path(const GlobalResult &gr, const Vec2 &scan_target,
                          const FrontierSelection *observation = nullptr);
  void cancel_handoff_brake();
  bool path_inside_safe_field(const Path2 &path) const;
  GlobalConfig explore_exec_config() const;
  bool turn_correction_clear(Vec2 correction, const Obstacles &obstacles) const;
  void begin_hold_from_motion(PositionHold &hold);
  Vec2 turn_position_correction(PositionHold &hold, const Obstacles &obstacles);
  void finish_observation(VelCmd &command, const Obstacles &obstacles);
  Trajectory rounded_fallback_trajectory(const Path2 &path, const GlobalConfig &cfg,
                                         const Obstacles &obs) const;
  Trajectory make_polyline_trajectory(const Path2 &path) const;
  Path2 clean_explore_guides(const Path2 &path, const Obstacles &obs) const;
  Path2 blend_explore_transition(const Path2 &path, const Obstacles &obs);
  double cone_hook_angle(const Vec2 &cur, const Path2 &path) const;
  double start_curvature(const Path2 &path, const Obstacles &obs) const;
  void enter_turn_for_solution(const Vec2 &cur, const Vec2 &scan_target, const GlobalResult &probe,
                               int candidate_band, const FrontierSelection *observation = nullptr);
  int preferred_turn_direction(double error) const;
  bool step_turn_for_solution(const Obstacles &obs, geometry_msgs::msg::TwistStamped &cmd);
  void replan_locked(const Vec2 &cur);
  GlobalResult search_global(const Vec2 &start, const Vec2 &goal, const Obstacles &obstacles,
                             const GlobalConfig &config, double yaw = NAN);
  GlobalResult search_explore_global(const Vec2 &start, const Vec2 &goal,
                                     const Obstacles &obstacles, double yaw = NAN);
  Vec2 pd_core(const Vec2 &heading_pt, double dist_for_speed);
  Vec2 pd_to_point(const Vec2 &target);
  void hold_required_path(const std::string &reason);
  GlobalConfig required_config() const;
  GlobalConfig exec_config() const;
  bool required_motion_clear(const Vec2 &stop, const Obstacles &obs) const;
  bool required_velocity_clear(const Vec2 &world_velocity, const Obstacles &obs) const;
  Vec2 pd_to_point_avoid(const Vec2 &target, const Obstacles &obs,
                         double travel_speed_limit = std::numeric_limits<double>::infinity());
  Vec2 guard_required_command(VelCmd command, const Obstacles &obs);
  bool step_route_rejoin(const Obstacles &obs, geometry_msgs::msg::TwistStamped &cmd);
  Vec2 recovery_velocity(const Obstacles &obs, bool required_geometry);
  VelCmd guard_explore_velocity(VelCmd command, const Obstacles &obs);
  double nearest_obstacle_dist(const Obstacles &obs) const;
  Vec2 do_retreat(const Obstacles &obs);
  bool retreat_available();
  bool try_retreat(const Obstacles &obs, geometry_msgs::msg::TwistStamped &cmd);
  bool required_retreat_allowed(const Obstacles &obs, geometry_msgs::msg::TwistStamped &cmd);
  void record_trail();
  void finish_backtrack(const char *reason);
  bool update_backtrack(const Obstacles &obs, geometry_msgs::msg::TwistStamped &cmd);
  void remember_yaw_command(double rate);
  double pending_yaw_rotation() const;
  double approach_yaw_rate(double heading) const;
  bool escape_velocity_clear(Vec2 velocity, const Obstacles &obs) const;
  void reset_home_approach();
  GlobalConfig home_transit_config() const;
  void straighten_home_entry(Vec2 start, const Obstacles &obs, const GlobalConfig &transit);
  bool approach_curve_clear(const Path2 &route, const Obstacles &obs,
                            const GlobalConfig &cfg) const;
  void prepare_home_curve(const Obstacles &obs);
  Vec2 approach_curve_velocity(const Obstacles &obs);
  bool select_home_entry(const Obstacles &obs);
  Vec2 home_target_velocity(const Obstacles &obs);
  void publish_obstacle_cloud();
  void publish_field_boundary();
  void on_goal(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void coverage_step(const Obstacles &obstacles, geometry_msgs::msg::TwistStamped &cmd);
  bool home_step(const Obstacles &obstacles, geometry_msgs::msg::TwistStamped &cmd);
  void on_timer();
  void publish_status(const std::string &phase);
  void configure();
  void connect_interfaces();

  GridConfig gcfg_;
  std::optional<acfly_interfaces::msg::ExplorationContext> context_;
  rclcpp::Publisher<acfly_interfaces::msg::ExplorationCommand>::SharedPtr control_pub_;
  rclcpp::Subscription<acfly_interfaces::msg::ExplorationContext>::SharedPtr context_sub_;
  builtin_interfaces::msg::Time goal_stamp_;
  std_msgs::msg::Header goal_header_;
  SensorFreshness pose_freshness_, cloud_freshness_;
  bool carlike_mode_{};
  double turn_blend_m_{};
  double turn_round_min_m_{};
  double turn_round_max_curvature_{};
  double turn_blend_min_angle_rad_{};
  double turn_blend_max_angle_rad_{};
  int turn_blend_samples_{};
  TrackerGains gains_;
  FrontierConfig fcfg_;
  bool frontier_observation_enabled_{};
  bool have_observation_target_ = false;
  FrontierSelection observation_target_, turn_observation_;
  double observation_yaw_rate_ = 0.0;
  PositionHold observation_hold_, turn_hold_;
  PositionHold required_goal_hold_;
  bool required_goal_holding_ = false;
  double required_goal_yaw_ = 0.0;
  double required_brake_speed_ = 0.0;
  bool observation_turn_active_ = false;
  double frontier_continuity_lookahead_{};
  int active_target_gain_ = -1;
  std::string last_replan_reason_ = "none";
  uint64_t kept_after_candidate_failure_ = 0;
  bool explore_handoff_deferred_ = false;
  bool observation_arrived_ = false;
  double guide_min_segment_{};
  double guide_max_deviation_{};
  double explore_plan_reserve_{};
  double required_plan_reserve_{};
  double arc_ds_;
  double goal_tol_, v_est_alpha_;
  double search_coverage_{};
  double home_slow_radius_{}, home_slow_speed_{};
  std::deque<Vec2> trail_;
  bool backtracking_ = false;
  Path2 backtrack_path_;
  size_t backtrack_next_ = 0;
  double backtrack_started_ = -1.0, no_route_since_ = -1.0, last_explore_tick_ = -1.0;
  Vec2 backtrack_command_{};
  double backtrack_after_s_{}, backtrack_distance_{};
  uint64_t backtracks_ = 0;
  double goal_stop_v_, kp_goal_, kd_goal_, v_goal_max_;
  double replan_period_{}, replan_dev_{};
  double candidate_period_{};
  double target_stale_r_{};
  ObstacleConfig ocfg_;
  double robot_radius_{};
  double ground_z_{};
  double ceil_z_{};
  double obs_skip_yaw_rate_{};
  double self_margin_{};
  GlobalConfig ggcfg_;
  double explore_guard_inflate_{};
  double home_goal_wall_margin_{};
  double commit_target_tol_{};
  double global_lookahead_{};
  double explore_handoff_m_{};
  double explore_target_min_dist_{};
  double path_switch_safety_gain_{};
  double path_switch_time_gain_{};
  bool path_switch_require_both_{};
  std::unique_ptr<GridMap> grid_;
  std::unique_ptr<TrajectoryTracker> tracker_;
  std::unique_ptr<TrajectoryTracker> global_tracker_;
  TrackerGains global_gains_{};
  double xy_brake_accel_{};
  Vec2 last_command_world_{};
  std::optional<Vec2> route_rejoin_target_;
  Path2 route_rejoin_path_;
  int route_rejoin_owner_ = 0;
  double route_rejoin_bias_ = 0;
  std::deque<double> yaw_command_history_;
  Vec2 recovery_anchor_{};
  bool recovery_holding_ = false, recovery_requested_ = false;
  std::string recovery_status_ = "none";
  std::unique_ptr<ObstacleMap> obs_map_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr finished_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr coverage_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  uint64_t diagnostic_ticks_ = 0, replan_checks_ = 0, astar_searches_ = 0;
  uint64_t candidate_skips_ = 0;
  uint64_t adoptions_ = 0, smooth_fallbacks_ = 0, turn_entries_ = 0, recovery_entries_ = 0;
  uint64_t explore_guard_rejections_ = 0, explore_guard_lateral_trims_ = 0;
  uint64_t cone_detour_rejections_ = 0;
  uint64_t cone_bypass_adoptions_ = 0;
  bool handoff_brake_ = false;
  double handoff_brake_speed_{};
  double handoff_brake_angle_{};
  uint64_t invalid_paths_ = 0;
  std::string smooth_failure_reason_ = "none";
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundary_pub_;
  bool boundary_sent_ = false;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr flight_state_sub_;
  std::optional<int64_t> obstacle_mapping_start_ns_;
  SensorFreshness input_cloud_sequence_;
  std::optional<int64_t> last_obstacle_cloud_stamp_ns_, mapping_clock_ns_;
  double mapping_clock_received_at_ = -1.0;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr mapping_clock_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::CallbackGroup::SharedPtr cloud_cbg_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mtx_;
  double px_ = 0.0, py_ = 0.0, pz_ = 0.0, yaw_ = 0.0;
  bool has_pose_ = false;
  Vec2 goal_;
  bool has_goal_ = false;
  bool plan_pending_ = false;
  Trajectory traj_;
  Vec2 last_look_;
  bool look_valid_ = false;
  bool finished_ = false;
  bool homing_ = false;
  double home_straight_approach_m_{};
  double home_approach_speed_{}, home_approach_yaw_ = 0.0;
  bool home_approach_active_ = false;
  HomeApproach home_entry_plan_;
  bool home_curve_active_ = false;
  Vec2 home_entry_goal_{}, home_leg_anchor_{};
  std::optional<Vec2> home_leg_target_;
  double home_leg_heading_ = 0.0, home_entry_retry_ = -1;
  int cur_band_ = -1;
  rclcpp::Time last_plan_time_;
  rclcpp::Time last_candidate_time_;
  bool candidate_time_valid_ = false;
  bool explore_failed_ = false;
  Path2 explore_raw_;
  bool explore_has_committed_ = false;
  bool explore_goal_projected_ = false;
  Vec2 explore_target_;
  std::vector<Vec2> unreachable_;
  double last_unreach_clear_cov_ = 0.0;
  double unreach_block_r_{};
  double unreach_clear_step_{};
  Trajectory global_traj_;
  Path2 global_raw_;
  Vec2 global_target_;
  bool global_has_ = false;
  bool global_failed_ = false;
  bool global_goal_blocked_ = false;
  bool global_at_goal_ = false;
  bool global_braking_blocked_ = false;
  bool command_projection_blocked_ = false, measured_projection_blocked_ = false;
  double required_motion_inflate_{};
  double required_path_exec_inflate_{};
  double required_blocked_replan_s_{};
  double required_blocked_since_ = 0.0;
  bool required_blocked_time_valid_ = false;
  uint64_t required_blocked_replans_ = 0;
  Vec2 global_connector_start_;
  uint64_t global_path_searches_ = 0, global_smooth_fallbacks_ = 0;
  uint64_t global_invalid_paths_ = 0, global_velocity_brakes_ = 0;
  std::string global_block_reason_ = "none";
  rclcpp::Time last_global_plan_time_;
  bool retreating_ = false;
  bool retreat_exhausted_ = false;
  Vec2 retreat_exhausted_at_;
  Vec2 retreat_origin_;
  double retreat_trigger_{};
  double retreat_step_{};
  double retreat_max_dist_{};
  double retreat_v_max_{};
  double route_rejoin_v_max_{};
  double route_rejoin_min_mps_{};
  rclcpp::Time retreat_start_;
  double retreat_timeout_{};
  bool turning_for_solution_ = false;
  bool turn_heading_valid_ = false;
  Path2 turn_probe_path_;
  Vec2 turn_target_;
  int turn_candidate_band_ = -1;
  int turn_dir_ = 1;
  double turn_prev_yaw_ = 0.0;
  double turn_heading_ = 0.0;
  double turn_command_rate_ = 0.0;
  double turn_settled_s_ = 0.0;
  double turn_accum_ = 0.0;
  rclcpp::Time turn_start_time_;
  int turn_research_tick_ = 0;
  double turn_solve_yaw_rate_{};
  int turn_solve_research_every_{};
  double turn_solve_max_rev_{};
  double turn_solve_timeout_{};
  bool has_unreachable_marker_ = false;
  Vec2 unreachable_pos_;
  static double wrap_pi(double a) {
    while (a > M_PI)
      a -= 2.0 * M_PI;
    while (a <= -M_PI)
      a += 2.0 * M_PI;
    return a;
  }
  double last_yaw_rate_ = 0.0;
  double prev_x_ = 0.0, prev_y_ = 0.0;
  double prev_yaw_ = 0.0;
  bool has_prev_pose_ = false;
  double v_fwd_est_ = 0.0, v_lat_est_ = 0.0;
  double yaw_rate_est_ = 0.0;
  double gyro_z_ = 0.0, gyro_at_ = -1e9, imu_timeout_{}, yaw_rate_diff_ = 0.0;
  std::array<double, 4> imu_attitude_{1.0, 0.0, 0.0, 0.0};
  double gyro_agreement_ = 1.0, gyro_ratio_ = 1.0;
  int gyro_agreement_samples_ = 0;
  bool gyro_disabled_ = false;
  std::string imu_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  bool gyro_fresh(double now) const {
    return now - gyro_at_ >= 0.0 && now - gyro_at_ <= imu_timeout_;
  }
  double current_yaw_rate(double now) const {
    return !gyro_disabled_ && gyro_fresh(now) ? gyro_z_ : yaw_rate_diff_;
  }
  bool explore_tracker_ticked_ = false, global_tracker_ticked_ = false;
  double cone_hook_deg_{}, cone_hook_window_{};
  double cone_hook_kappa_{};
  int max_home_retreats_{}, trail_capacity_{};
  double trail_spacing_{}, backtrack_tolerance_{}, backtrack_timeout_{};
  double clock_domain_gap_s_{}, clock_cloud_gap_s_{}, clock_timeout_s_{};
  int required_retreats_ = 0;
  unsigned long cone_hook_rejections_ = 0;
};
} // namespace exploration
