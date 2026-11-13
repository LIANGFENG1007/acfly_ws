#include "exploration_node.hpp"

namespace exploration {

void ExplorationNode::configure() {
  timer_period_ms_ = read<int>("timer_period_ms");
  diagnostics_every_ = read<int>("diagnostics_period_ticks");
  pose_timeout_ = read<double>("pose_timeout_s");
  cloud_timeout_ = read<double>("cloud_timeout_s");
  imu_timeout_ = read<double>("imu_timeout_s");
  imu_topic_ = read<std::string>("imu_topic");
  cruise_vertical_limit_ = read<double>("cruise_vertical_limit");
  if (timer_period_ms_ <= 0 || diagnostics_every_ <= 0 || pose_timeout_ <= 0 ||
      cloud_timeout_ <= 0 || imu_timeout_ <= 0 || cruise_vertical_limit_ <= 0)
    throw std::invalid_argument("Positive timing and motion limits required");
  clock_domain_gap_s_ = read<double>("clock_domain_gap_s");
  clock_timeout_s_ = read<double>("clock_timeout_s");
  clock_cloud_gap_s_ = read<double>("clock_cloud_gap_s");
  max_home_retreats_ = read<int>("max_home_retreats");
  trail_capacity_ = read<int>("trail_capacity");
  trail_spacing_ = read<double>("trail_spacing_m");
  backtrack_tolerance_ = read<double>("backtrack_tolerance_m");
  backtrack_timeout_ = read<double>("backtrack_timeout_s");
  if (clock_domain_gap_s_ <= 0 || clock_timeout_s_ <= 0 || clock_cloud_gap_s_ <= 0 ||
      max_home_retreats_ < 0 || trail_capacity_ < 1 || trail_spacing_ <= 0 ||
      backtrack_tolerance_ <= 0 || backtrack_timeout_ <= 0)
    throw std::invalid_argument("Invalid clock thresholds, return attempts, or recovery trail");
  carlike_mode_ = read<bool>("exploration_carlike_mode");
  gcfg_.min_x = read<double>("field_min_x");
  gcfg_.min_y = read<double>("field_min_y");
  gcfg_.max_x = read<double>("field_max_x");
  gcfg_.max_y = read<double>("field_max_y");
  gcfg_.big_cell = read<double>("big_cell");
  gcfg_.small_cell = read<double>("small_cell");
  gcfg_.coverage_thresh = read<double>("coverage_thresh");
  gcfg_.fov_deg = read<double>("fov_deg");
  gcfg_.fov_range = read<double>("fov_range");
  if (!std::isfinite(gcfg_.fov_range) || gcfg_.fov_range <= 0)
    throw std::invalid_argument("fov_range must be finite and positive");
  gcfg_.scan_dwell_ticks = static_cast<int>(
      std::clamp(std::round(read<double>("scan_dwell_s") * 1000.0 / timer_period_ms_), 1.0, 127.0));
  gcfg_.scan_near_range = std::max(0.0, read<double>("scan_near_range_m"));
  arc_ds_ = read<double>("arc_sample_ds");

  gains_.v_max = read<double>("v_max");
  gains_.v_min = read<double>("v_min");
  gains_.k_curv = read<double>("k_curv");
  gains_.lookahead = read<double>("lookahead");
  gains_.endpoint_slow_r = read<double>("endpoint_slow_r");
  gains_.endpoint_slow_speed = std::max(0.0, read<double>("endpoint_slow_speed"));
  gains_.lookahead_time = std::max(0.0, read<double>("carlike_lookahead_time"));
  gains_.lookahead_max = std::max(gains_.lookahead, read<double>("carlike_lookahead_max"));
  gains_.kp_yaw = read<double>("kp_yaw");
  gains_.kd_yaw = read<double>("kd_yaw");
  gains_.max_yaw_rate = read<double>("max_yaw_rate");
  gains_.kp_lat = read<double>("kp_lat");
  gains_.kd_lat = read<double>("kd_lat");
  gains_.max_v_lat = read<double>("max_v_lat");
  gains_.heading_gate_rad = read<double>("heading_gate_deg") * M_PI / 180.0;
  gains_.forward_only = carlike_mode_;
  gains_.compensate_lateral_drift = carlike_mode_;
  gains_.drift_velocity_gain = std::max(0.0, read<double>("carlike_drift_kd"));
  xy_brake_accel_ = std::max(0.0, read<double>("xy_brake_accel"));
  gains_.max_accel = read<double>("carlike_max_accel");
  gains_.max_decel = std::max(0.0, read<double>("carlike_max_decel"));
  gains_.curvature_feedforward = std::clamp(read<double>("carlike_curvature_ff"), 0.0, 1.5);
  gains_.creep_speed = std::max(0.0, read<double>("carlike_creep_speed"));
  handoff_brake_speed_ = std::max(0.0, read<double>("carlike_handoff_brake_speed"));
  handoff_brake_angle_ =
      std::clamp(read<double>("carlike_handoff_brake_angle_deg"), 5.0, 180.0) * M_PI / 180.0;
  gains_.max_yaw_accel = read<double>("carlike_max_yaw_accel");
  gains_.yaw_filter_tau = read<double>("carlike_yaw_filter_s");
  gains_.prediction_time = read<double>("carlike_prediction_s");
  gains_.lateral_prediction_time = read<double>("carlike_lateral_prediction_s");
  gains_.align_resume_rad = read<double>("carlike_align_resume_deg") * M_PI / 180.0;
  gains_.align_stop_speed = read<double>("carlike_align_stop_speed");
  gains_.align_stop_yaw_rate = read<double>("carlike_align_stop_yaw_rate");
  gains_.align_settle_s = read<double>("carlike_align_settle_s");
  gains_.stop_align_rad = read<double>("carlike_stop_align_deg") * M_PI / 180.0;
  gains_.corner_stop_rad = read<double>("carlike_corner_stop_deg") * M_PI / 180.0;
  gains_.max_lateral_accel = read<double>("carlike_max_lateral_accel");
  {
    gains_.smith_time = std::max(0.0, read<double>("carlike_smith_s"));
    gains_.smith_yaw_time = std::max(0.0, read<double>("carlike_smith_yaw_s"));
    gains_.command_filter_tau = std::max(0.0, read<double>("carlike_command_filter_s"));
    gains_.lateral_slow_gain = std::max(0.0, read<double>("carlike_lateral_slow_gain"));
    gains_.lateral_slow_deadband = std::max(0.0, read<double>("carlike_lateral_slow_deadband"));
    gains_.lateral_slow_tau = std::max(0.0, read<double>("carlike_lateral_slow_tau"));
    gains_.stop_anchor_latency = std::max(0.0, read<double>("carlike_stop_anchor_latency_s"));
    gains_.stop_anchor_max_m = std::max(0.0, read<double>("carlike_stop_anchor_max_m"));
  }
  gains_.stop_anchor_decel = read<double>("stop_anchor_decel_mps2");
  gains_.yaw_damping_command_weight = read<double>("yaw_damping_command_weight");
  gains_.turn_hold_kp = read<double>("turn_hold_kp");
  gains_.turn_hold_kd = read<double>("turn_hold_kd");
  gains_.turn_hold_speed = read<double>("turn_hold_speed_mps");
  if (gains_.stop_anchor_decel <= 0 || gains_.yaw_damping_command_weight < 0 ||
      gains_.yaw_damping_command_weight > 1 || gains_.turn_hold_kp < 0 || gains_.turn_hold_kd < 0 ||
      gains_.turn_hold_speed <= 0)
    throw std::invalid_argument("Invalid stop-anchor, yaw damping or turn-hold settings");
  if (!carlike_mode_) {
    gains_.lookahead_time = gains_.smith_time = gains_.smith_yaw_time = 0;
    gains_.command_filter_tau = gains_.lateral_slow_deadband = gains_.lateral_slow_tau = 0;
  }
  turn_blend_m_ = read<double>("carlike_turn_blend_m");
  turn_blend_min_angle_rad_ = read<double>("carlike_turn_blend_angle_deg") * M_PI / 180.0;
  turn_blend_max_angle_rad_ =
      std::clamp(read<double>("turn_blend_max_angle_deg"), 0.0, 180.0) * M_PI / 180.0;
  cone_hook_deg_ = std::max(0.0, read<double>("carlike_cone_hook_deg"));
  cone_hook_window_ = std::max(0.0, read<double>("carlike_cone_hook_window_m"));
  cone_hook_kappa_ = std::max(0.0, read<double>("carlike_cone_hook_kappa"));
  turn_blend_samples_ = read<int>("carlike_turn_blend_samples");
  turn_round_min_m_ = std::max(0.0, read<double>("turn_round_min_m"));
  turn_round_max_curvature_ = std::max(0.0, read<double>("turn_round_max_curvature"));
  // D 项差分周期 = 主循环真实周期。★与 timer 同源推导★，勿写死常数(见 TrackerGains::dt)。
  gains_.dt = timer_period_ms_ / 1000.0;

  search_coverage_ = read<double>("search_coverage");
  backtrack_after_s_ = std::max(0.0, read<double>("explore_backtrack_after_s"));
  backtrack_distance_ = std::max(0.3, read<double>("explore_backtrack_m"));
  goal_tol_ = read<double>("goal_tol_xy");
  goal_stop_v_ = read<double>("goal_stop_v");
  kp_goal_ = read<double>("kp_goal");
  kd_goal_ = read<double>("kd_goal");
  v_goal_max_ = read<double>("v_goal_max");
  v_est_alpha_ = read<double>("v_est_alpha");

  // 动态前沿覆盖配置
  fcfg_.min_x = gcfg_.min_x;
  fcfg_.min_y = gcfg_.min_y;
  fcfg_.max_x = gcfg_.max_x;
  fcfg_.max_y = gcfg_.max_y;
  fcfg_.big_cell = gcfg_.big_cell;
  fcfg_.margin = read<double>("wall_margin");
  fcfg_.near_weight = read<double>("frontier_near_w");
  fcfg_.turn_penalty = read<double>("frontier_turn_pen");
  fcfg_.cluster_weight = read<double>("frontier_cluster_w");
  // 孤格饥饿修复：cluster 加成饱和上限（防遮挡小区被排到队尾、走远才回头补扫）
  fcfg_.cluster_cap = read<int>("frontier_cluster_cap");
  fcfg_.horizon = read<double>("explore_horizon_m");
  fcfg_.chain_gap = read<double>("chain_gap_m");
  fcfg_.band_width = read<double>("band_width");
  fcfg_.band_tol = read<double>("band_tol");
  fcfg_.along_bonus = read<double>("along_bonus");
  fcfg_.band_clear_cnt = read<int>("band_clear_cnt");
  frontier_observation_enabled_ = read<bool>("frontier_observation_enabled");
  fcfg_.small_region_cells = read<int>("frontier_small_region_cells");
  fcfg_.small_region_penalty = read<double>("frontier_small_region_penalty");
  fcfg_.observation_standoff = read<double>("frontier_observation_standoff");
  fcfg_.preferred_clearance = read<double>("frontier_preferred_clearance");
  fcfg_.gain_weight = read<double>("frontier_gain_weight");
  fcfg_.clearance_weight = read<double>("frontier_clearance_weight");
  fcfg_.observation_min_distance = read<double>("frontier_observation_min_distance");
  fcfg_.continuity_turn_penalty = read<double>("frontier_continuity_turn_pen");
  fcfg_.goal_detour_weight = std::max(0.0, read<double>("frontier_goal_detour_w"));
  fcfg_.goal_detour_start = read<double>("frontier_goal_detour_start");
  guide_min_segment_ = std::max(0.0, read<double>("explore_guide_min_segment_m"));
  guide_max_deviation_ = std::max(0.0, read<double>("explore_guide_max_deviation_m"));
  explore_plan_reserve_ = std::max(0.0, read<double>("explore_plan_reserve_m"));
  required_plan_reserve_ = std::max(0.0, read<double>("required_plan_reserve_m"));
  frontier_continuity_lookahead_ = std::max(0.10, read<double>("frontier_continuity_lookahead_m"));
  replan_period_ = read<double>("replan_period_s");
  replan_dev_ = read<double>("replan_dev_m");
  candidate_period_ = std::max(0.0, read<double>("replan_candidate_period_s"));
  // 目标失效检测：承诺目标邻域此半径内已无未扫大格 → 放弃承诺改投别处。0=关闭
  target_stale_r_ = read<double>("target_stale_r");

  // ---- 雷达感知：点云聚类(obstacle_map) 配置 ----
  ground_z_ = read<double>("ground_z");
  ceil_z_ = read<double>("obs_z_max");
  obs_skip_yaw_rate_ = read<double>("obs_skip_yaw_rate");
  self_margin_ = read<double>("obs_self_margin");
  ocfg_.min_x = gcfg_.min_x;
  ocfg_.min_y = gcfg_.min_y;
  ocfg_.max_x = gcfg_.max_x;
  ocfg_.max_y = gcfg_.max_y;
  ocfg_.cell = read<double>("obs_cell");
  ocfg_.edge_ignore = read<double>("obs_edge_ignore");
  ocfg_.hit_thresh = read<int>("obs_hit_thresh");
  ocfg_.hit_inc = read<int>("obs_hit_inc");
  ocfg_.hit_max = read<int>("obs_hit_max");
  ocfg_.hit_decay = read<int>("obs_hit_decay");
  ocfg_.min_cells = read<int>("obs_min_cells");
  ocfg_.min_r = read<double>("obs_min_r");
  ocfg_.max_r = read<double>("obs_max_r");
  ocfg_.r_inflate = read<double>("obs_r_inflate");
  // 障碍时序平滑(跟踪)：压住障碍圆逐帧闪动/瞬移
  ocfg_.track_alpha = read<double>("obs_track_alpha");
  ocfg_.track_assoc_dist = read<double>("obs_track_assoc_dist");
  ocfg_.track_max_misses = read<int>("obs_track_max_misses");
  // latch 智能回收(防幽灵)：视野内连续无命中达帧数阈值 → 撤销 latch。FOV 取与栅格同源。
  ocfg_.fov_deg = gcfg_.fov_deg;
  ocfg_.fov_range = gcfg_.fov_range;
  ocfg_.ghost_clear_frames = read<int>("obs_ghost_clear_frames");

  robot_radius_ = read<double>("robot_radius");

  // ---- 全局点到点绕障（A*，全局层）配置：全场唯一避障手段 ----
  commit_target_tol_ = read<double>("commit_target_tol");
  global_lookahead_ = read<double>("global_lookahead");
  // 探索路线剩余不足此距离即接力下一观察点(原与必达 carrot 前瞻共用 global_lookahead)。
  explore_handoff_m_ = std::max(0.0, read<double>("explore_handoff_m"));
  explore_target_min_dist_ = read<double>("explore_target_min_dist");
  // 换路评估(路径迟滞)：重算出新路后只有它更安全/更快超阈值才换边，根除左右横跳
  path_switch_safety_gain_ = read<double>("path_switch_safety_gain");
  path_switch_time_gain_ = read<double>("path_switch_time_gain");
  path_switch_require_both_ = read<bool>("path_switch_require_both");
  unreach_block_r_ = read<double>("unreach_block_r");
  unreach_clear_step_ = read<double>("unreach_clear_step");
  retreat_trigger_ = read<double>("retreat_trigger");
  retreat_step_ = read<double>("retreat_step");
  retreat_max_dist_ = read<double>("retreat_max_dist");
  retreat_v_max_ = read<double>("retreat_v_max");
  route_rejoin_v_max_ = std::max(0.0, read<double>("route_rejoin_v_max"));
  route_rejoin_min_mps_ = std::max(0.0, read<double>("route_rejoin_min_mps"));
  retreat_timeout_ = read<double>("retreat_timeout_s");
  ggcfg_.min_x = gcfg_.min_x;
  ggcfg_.min_y = gcfg_.min_y;
  ggcfg_.max_x = gcfg_.max_x;
  ggcfg_.max_y = gcfg_.max_y;
  ggcfg_.cell = read<double>("global_cell");
  ggcfg_.robot_radius = robot_radius_;
  ggcfg_.inflate = read<double>("global_margin");
  fcfg_.minimum_clearance = ggcfg_.robot_radius + ggcfg_.inflate;
  // 探索段执行口径：巡航横漂补偿/定点纠偏的短投影校验只保留 EXPLORE_GUARD_INFLATE 的余量。
  // 规划与采纳仍用完整 global_margin；路线本身就贴着规划圈，跟踪偏内几厘米不应被判成
  // "任何非径向向外的运动都不安全"(seed42 基线 11 s 停滞、r4 128 拍拦截均发生在净距 0.38~0.41 m)。
  explore_guard_inflate_ = std::clamp(read<double>("explore_guard_inflate"), 0.0, ggcfg_.inflate);
  ggcfg_.wall_margin = read<double>("global_wall_margin");
  home_goal_wall_margin_ = read<double>("home_goal_wall_margin");
  if (!std::isfinite(home_goal_wall_margin_) || home_goal_wall_margin_ < 0.0 ||
      home_goal_wall_margin_ > ggcfg_.wall_margin)
    throw std::invalid_argument("home_goal_wall_margin must be between 0 and global_wall_margin");
  ggcfg_.required_connector_length = std::max(0.0, read<double>("required_connector_length"));
  required_motion_inflate_ =
      std::clamp(read<double>("required_motion_inflate"), 0.0, std::max(0.0, ggcfg_.inflate));
  required_path_exec_inflate_ =
      std::clamp(read<double>("required_path_exec_inflate"), 0.0, std::max(0.0, ggcfg_.inflate));
  required_blocked_replan_s_ = std::max(0.0, read<double>("required_blocked_replan_s"));
  // ★机头锥★：A* 起点段只朝机头延伸(根除新路从侧后方起步→边转边走横切撞柱)
  ggcfg_.head_cone_half = read<double>("global_head_cone_deg") * M_PI / 180.0;
  ggcfg_.head_cone_radius = read<double>("global_head_cone_radius");
  // ★原地转身找解★：锥内无解但有路(只是不在机头方向)时原地转身改朝向重搜
  turn_solve_yaw_rate_ = read<double>("turn_solve_yaw_rate");
  turn_solve_research_every_ = read<int>("turn_solve_research_every");
  turn_solve_max_rev_ = read<double>("turn_solve_max_rev");
  turn_solve_timeout_ = read<double>("turn_solve_timeout_s");

  if (!(gcfg_.min_x < gcfg_.max_x && gcfg_.min_y < gcfg_.max_y) || gcfg_.small_cell <= 0 ||
      gcfg_.big_cell < gcfg_.small_cell || ggcfg_.cell <= 0 || ocfg_.cell <= 0 || arc_ds_ <= 0 ||
      search_coverage_ <= 0 || search_coverage_ > 1 || gcfg_.coverage_thresh <= 0 ||
      gcfg_.coverage_thresh > 1 || gcfg_.fov_deg <= 0 || gcfg_.fov_deg > 360 ||
      ground_z_ >= ceil_z_ || gains_.v_max <= 0 || gains_.v_min < 0 ||
      gains_.v_min > gains_.v_max || gains_.lookahead <= 0 || gains_.max_accel <= 0 ||
      gains_.max_decel <= 0 || goal_tol_ <= 0 || goal_stop_v_ <= 0 || v_goal_max_ <= 0 ||
      robot_radius_ <= 0 || ggcfg_.inflate < 0 || ggcfg_.wall_margin < 0 || ocfg_.min_r < 0 ||
      ocfg_.max_r < ocfg_.min_r || ocfg_.min_cells <= 0 || ocfg_.hit_thresh <= 0 ||
      ocfg_.hit_max < ocfg_.hit_thresh || ocfg_.hit_inc <= 0 || ocfg_.hit_decay < 0 ||
      ocfg_.hit_max > UINT16_MAX || v_est_alpha_ <= 0 || v_est_alpha_ > 1)
    throw std::invalid_argument("Invalid field, map, trajectory, or coverage configuration");
  for (double cell : {gcfg_.small_cell, ocfg_.cell, ggcfg_.cell}) {
    const double cells = std::ceil((gcfg_.max_x - gcfg_.min_x) / cell) *
                         std::ceil((gcfg_.max_y - gcfg_.min_y) / cell);
    if (!std::isfinite(cells) || cells > 4000000)
      throw std::invalid_argument("Configured field exceeds grid capacity");
  }
  if (gcfg_.big_cell > std::min(gcfg_.max_x - gcfg_.min_x, gcfg_.max_y - gcfg_.min_y) ||
      gcfg_.big_cell / gcfg_.small_cell > 2000 || ocfg_.track_alpha <= 0 || ocfg_.track_alpha > 1 ||
      fcfg_.band_width <= 0 || turn_solve_research_every_ < 1 || turn_blend_samples_ < 1 ||
      gains_.max_yaw_rate <= 0 || gains_.max_yaw_accel <= 0 || gains_.max_lateral_accel <= 0 ||
      gains_.max_v_lat < 0 || replan_period_ <= 0 || target_stale_r_ < 0)
    throw std::invalid_argument(
        "Invalid sampling resolution, planning interval, or lateral/yaw limits");
  grid_ = std::make_unique<GridMap>(gcfg_);
  tracker_ = std::make_unique<TrajectoryTracker>(gains_);
  global_gains_ = gains_;
  //   刹停投影对航向滞后很敏感。Smith/指令低通/减半的横漂阻尼只用于探索段
  //   (v4 仿真：归航段加了这些后末端反复急刹，倾角高频 0.14°→0.54°；基线归航本就平稳)。
  global_gains_.smith_time = global_gains_.smith_yaw_time = 0.0;
  global_gains_.command_filter_tau = 0.0;
  global_gains_.lateral_slow_gain = read<double>("return_lateral_slow_gain");
  global_gains_.hold_slide = false;
  global_gains_.stop_anchor_latency = read<double>("return_stop_anchor_latency_s");
  global_gains_.stop_anchor_max_m = read<double>("return_stop_anchor_max_m");
  if (global_gains_.lateral_slow_gain < 0 || global_gains_.stop_anchor_latency < 0 ||
      global_gains_.stop_anchor_max_m < 0)
    throw std::invalid_argument(
        "Return stop-anchor and lateral damping values must be nonnegative");
  global_gains_.lateral_slow_deadband = global_gains_.lateral_slow_tau = 0.0;
  global_gains_.drift_velocity_gain = std::max(0.0, read<double>("required_drift_kd"));
  global_gains_.v_max = v_goal_max_;
  // 必达点停车律同样与限速解耦：减速半径处速度固定为 REQUIRED_SLOW_V(0.6
  // 时代验证过的末端手感)，半径外按减速度提前平滑减速。
  //   V_GOAL_MAX 0.6→0.75 时旧律(斜率 v_goal_max/R)让离终点 0.5~1.5 m
  //   的刹停段倾角高频升到 1.1~1.3°(r137/r138/r140)。
  global_gains_.endpoint_slow_speed =
      std::min(v_goal_max_, std::max(0.05, read<double>("required_slow_speed")));
  global_gains_.lookahead_time = 0.0; // 必达点保持固定前瞻
  global_gains_.v_min = std::min(global_gains_.v_min, v_goal_max_);
  global_gains_.lookahead = global_lookahead_;
  global_gains_.endpoint_slow_r =
      std::max({global_gains_.endpoint_slow_r,
                std::max(0.0, gains_.prediction_time) * v_goal_max_ +
                    v_goal_max_ * v_goal_max_ / (2.0 * std::max(1e-3, gains_.max_accel)),
                read<double>("required_slow_radius_m")});
  home_slow_radius_ = std::max(global_gains_.endpoint_slow_r, read<double>("home_slow_radius_m"));
  home_slow_speed_ = std::min(v_goal_max_, std::max(0.05, read<double>("home_slow_speed")));
  home_straight_approach_m_ = read<double>("home_straight_approach_m");
  home_approach_speed_ = read<double>("home_approach_speed");
  if (!std::isfinite(home_straight_approach_m_) || home_straight_approach_m_ < 0.0 ||
      !std::isfinite(home_approach_speed_) || home_approach_speed_ <= 0.0)
    throw std::invalid_argument("home approach distance must be nonnegative and speed positive");
  global_tracker_ = std::make_unique<TrajectoryTracker>(global_gains_);
  obs_map_ = std::make_unique<ObstacleMap>(ocfg_);
}

} // namespace exploration
