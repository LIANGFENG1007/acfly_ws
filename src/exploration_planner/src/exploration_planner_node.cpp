// ============================================================================
//  exploration_planner_node.cpp  ── 全局自主探索算法 主节点
//
//  订阅：
//    /aft_mapped_to_init  (nav_msgs/Odometry, camera_init/SLAM, ~20Hz) 位姿
//    /exploration/goal    (geometry_msgs/PointStamped, latched) 探索终点(进入时一次)
//  发布：
//    /exploration/cmd_vel (geometry_msgs/TwistStamped) 机体系速度 50Hz
//                          linear.x=前进 v_fwd, linear.y=横向纠偏 v_lat, angular.z=yaw_rate
//    /exploration/finished(std_msgs/Bool, latched) 扫完且到终点 → true
//
//  流程：收到 goal → 动态前沿覆盖选点 → A* 绕障 → 平滑 → 轨迹跟踪逐拍发速度；
//        每拍把 FOV_DEG/FOV_RANGE(当前 100°/3m)扇形标进双层栅格；覆盖率达标且到终点 → finished。
//  ★频率★：控制 50Hz(TIMER_PERIOD_MS=20)，可视化 20Hz(VIZ_PERIOD_MS=50)。二者独立。
//  可视化：OpenCV 弹窗（主线程刷新，spin 在子线程；栅格走 snapshot 拷贝，无竞态）。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <limits>
#include <thread>
#include <unordered_set>

#include "exploration_planner/params.hpp"
#include "exploration_planner/types.hpp"
#include "exploration_planner/grid_map.hpp"
#include "exploration_planner/coverage_planner.hpp"
#include "exploration_planner/bezier.hpp"
#include "exploration_planner/trajectory_tracker.hpp"
#include "exploration_planner/visualizer.hpp"
#include "exploration_planner/obstacle_map.hpp"
#include "exploration_planner/global_planner.hpp"
#include "exploration_planner/corridor_controller.hpp"
#include "exploration_planner/sensor_freshness.hpp"
#include "exploration_planner/vision_shm.hpp"
#include "exploration_planner/vision_candidates.hpp"
#include "exploration_planner/vision_target_queue.hpp"
#include "exploration_planner/vision_near_confirmation.hpp"
#include "exploration_planner/cloud_xyz.hpp"

using namespace std::chrono_literals;
using namespace exploration;

class ExplorationNode : public rclcpp::Node
{
public:
    ExplorationNode()
        : Node("exploration_planner_node")
    {
        // ---- 参数 ----
        auto dd = [this](const std::string& n, double v) { return this->declare_parameter<double>(n, v); };
        exploration_enabled_ = declare_parameter<bool>("exploration_enabled", params::EXPLORATION_ENABLED);
        const bool vision_shm_enabled = declare_parameter<bool>("vision_shm_enabled", params::VISION_SHM_ENABLED);
        const auto vision_shm_path = declare_parameter<std::string>("vision_shm_path", params::VISION_SHM_PATH);
        const double vision_max_age = dd("vision_shm_max_age_s", params::VISION_SHM_MAX_AGE_S);
        const double vision_future_tolerance = dd("vision_shm_future_tol_s", params::VISION_SHM_FUTURE_TOL_S);
        if (vision_shm_enabled) {
            vision_reader_ = std::make_unique<VisionShmReader>(vision_shm_path, vision_max_age, vision_future_tolerance);
            RCLCPP_INFO(get_logger(), "[视觉SHM] 只读接收 %s，每帧最多6个目标；当前阶段管理队列/近距确认，不触发目标飞行",
                vision_shm_path.c_str());
        }
        VisionCandidateConfig vision_config;
        vision_config.confirm_frames = declare_parameter<int>("vision_far_confirm_frames", params::VISION_FAR_CONFIRM_FRAMES);
        vision_config.association_radius = dd("vision_assoc_radius_m", params::VISION_ASSOC_RADIUS_M);
        vision_config.weight_min_distance = dd("vision_weight_min_distance_m", params::VISION_WEIGHT_MIN_DISTANCE_M);
        vision_config.frame_gap_timeout = dd("vision_frame_gap_timeout_s", params::VISION_FRAME_GAP_TIMEOUT_S);
        vision_config.tentative_timeout = dd("vision_tentative_timeout_s", params::VISION_TENTATIVE_TIMEOUT_S);
        const int vision_max_candidates = declare_parameter<int>("vision_max_candidates", params::VISION_MAX_CANDIDATES);
        if (vision_max_candidates < 1) throw std::invalid_argument("vision_max_candidates must be positive");
        vision_config.max_tracks = static_cast<size_t>(vision_max_candidates);
        vision_pose_timeout_ = dd("vision_pose_timeout_s", params::VISION_POSE_TIMEOUT_S);
        if (!std::isfinite(vision_pose_timeout_) || vision_pose_timeout_ <= 0)
            throw std::invalid_argument("vision_pose_timeout_s must be positive");
        vision_candidates_ = std::make_unique<VisionCandidates>(vision_config);
        VisionQueueConfig vision_queue_config;
        vision_queue_config.association_radius = vision_config.association_radius;
        vision_queue_config.blacklist_radius = dd("vision_blacklist_radius_m", params::VISION_BLACKLIST_RADIUS_M);
        vision_queue_config.retry_delay = dd("vision_retry_delay_s", params::VISION_RETRY_DELAY_S);
        vision_queue_config.retry_observation_age = vision_config.frame_gap_timeout;
        const int vision_queue_capacity = declare_parameter<int>("vision_queue_capacity", params::VISION_QUEUE_CAPACITY);
        if (vision_queue_capacity < 1) throw std::invalid_argument("vision_queue_capacity must be positive");
        vision_queue_config.max_pending = static_cast<size_t>(vision_queue_capacity);
        vision_queue_ = std::make_unique<VisionTargetQueue>(vision_queue_config);
        VisionNearConfig vision_near_config;
        vision_near_config.confirm_frames = declare_parameter<int>("vision_near_confirm_frames", params::VISION_NEAR_CONFIRM_FRAMES);
        vision_near_config.near_distance = dd("vision_near_distance_m", params::VISION_NEAR_DISTANCE_M);
        vision_near_config.speed_cap = dd("vision_near_speed_mps", params::VISION_NEAR_SPEED_MPS);
        vision_near_config.association_radius = vision_config.association_radius;
        vision_near_config.weight_min_distance = vision_config.weight_min_distance;
        vision_near_config.frame_gap_timeout = vision_config.frame_gap_timeout;
        vision_near_config.confirmation_timeout = dd("vision_near_timeout_s", params::VISION_NEAR_TIMEOUT_S);
        vision_near_ = std::make_unique<VisionNearConfirmation>(vision_near_config);
        vision_hover_s_ = dd("vision_hover_s", params::VISION_HOVER_S);
        vision_straight_approach_m_ = dd("vision_straight_approach_m", params::VISION_STRAIGHT_APPROACH_M);
        vision_target_tol_ = dd("vision_target_tol_m", params::VISION_TARGET_TOL_M);
        vision_target_stop_speed_ = dd("vision_target_stop_speed_mps", params::VISION_TARGET_STOP_SPEED_MPS);
        if (!std::isfinite(vision_straight_approach_m_) || vision_straight_approach_m_ < 0.0 ||
            !std::isfinite(vision_hover_s_) || vision_hover_s_ < 0.0 ||
            !std::isfinite(vision_target_tol_) || vision_target_tol_ <= 0.0 ||
            !std::isfinite(vision_target_stop_speed_) || vision_target_stop_speed_ <= 0.0)
            throw std::invalid_argument("Invalid visual target hover limits");
        carlike_mode_ = declare_parameter<bool>(
            "exploration_carlike_mode", params::EXPLORATION_CARLIKE_MODE);
        gcfg_.min_x = dd("field_min_x", params::FIELD_MIN_X);
        gcfg_.min_y = dd("field_min_y", params::FIELD_MIN_Y);
        gcfg_.max_x = dd("field_max_x", params::FIELD_MAX_X);
        gcfg_.max_y = dd("field_max_y", params::FIELD_MAX_Y);
        corridor_enabled_ = declare_parameter<bool>("corridor_enabled", params::CORRIDOR_ENABLED);
        corridor_cloud_topic_ = declare_parameter<std::string>("corridor_cloud_topic", params::CORRIDOR_CLOUD_TOPIC);
        ccfg_.initial_yaw = dd("corridor_initial_yaw_deg", params::CORRIDOR_INITIAL_YAW_DEG) * M_PI / 180.0;
        ccfg_.entry_speed = dd("corridor_entry_speed", params::CORRIDOR_ENTRY_SPEED);
        ccfg_.cruise_speed = dd("corridor_cruise_speed", params::CORRIDOR_CRUISE_SPEED);
        ccfg_.gate_speed = dd("corridor_gate_speed", params::CORRIDOR_GATE_SPEED);
        ccfg_.align_speed = dd("corridor_align_speed", params::CORRIDOR_ALIGN_SPEED);
        ccfg_.yaw_kp = dd("corridor_yaw_kp", params::CORRIDOR_YAW_KP);
        ccfg_.max_yaw_rate = dd("corridor_max_yaw_rate", params::CORRIDOR_MAX_YAW_RATE);
        ccfg_.yaw_accel = dd("corridor_yaw_accel", params::CORRIDOR_YAW_ACCEL);
        ccfg_.acceleration = dd("corridor_acceleration", params::CORRIDOR_ACCEL);
        ccfg_.position_kp = dd("corridor_position_kp", params::CORRIDOR_POSITION_KP);
        ccfg_.velocity_kd = dd("corridor_velocity_kd", params::CORRIDOR_VELOCITY_KD);
        ccfg_.velocity_filter_tau = dd("corridor_velocity_filter_s", params::CORRIDOR_VELOCITY_FILTER_S);
        ccfg_.arrival_hysteresis = dd("corridor_arrival_hysteresis", params::CORRIDOR_ARRIVAL_HYSTERESIS);
        ccfg_.heading_tolerance = dd("corridor_yaw_tol_deg", params::CORRIDOR_YAW_TOL_DEG) * M_PI / 180.0;
        ccfg_.heading_stop = dd("corridor_heading_stop_deg", params::CORRIDOR_HEADING_STOP_DEG) * M_PI / 180.0;
        ccfg_.point_tolerance = dd("corridor_point_tol", params::CORRIDOR_POINT_TOL);
        ccfg_.stop_speed = dd("corridor_stop_speed", params::CORRIDOR_STOP_SPEED);
        ccfg_.settle_time = dd("corridor_settle_s", params::CORRIDOR_SETTLE_S);
        ccfg_.lookahead = dd("corridor_lookahead", params::CORRIDOR_LOOKAHEAD);
        ccfg_.approach_distance = dd("corridor_approach_m", params::CORRIDOR_APPROACH_M);
        ccfg_.continuous_approach = declare_parameter<bool>("corridor_continuous_approach", params::CORRIDOR_CONTINUOUS_APPROACH);
        ccfg_.continuous_center_reserve = dd("corridor_continuous_center_reserve", params::CORRIDOR_CONTINUOUS_CENTER_RESERVE);
        ccfg_.continuous_time_margin = dd("corridor_continuous_time_margin", params::CORRIDOR_CONTINUOUS_TIME_MARGIN);
        ccfg_.moving_lookahead = dd("corridor_moving_lookahead", params::CORRIDOR_MOVING_LOOKAHEAD);
        ccfg_.center_prediction_time = dd("corridor_center_prediction_s", params::CORRIDOR_CENTER_PREDICTION_S);
        ccfg_.exit_distance = dd("corridor_exit_m", params::CORRIDOR_EXIT_M);
        ccfg_.center_tolerance = dd("corridor_center_tol", params::CORRIDOR_CENTER_TOL);
        ccfg_.gate_association = dd("corridor_gate_assoc_m", params::CORRIDOR_GATE_ASSOC_M);
        ccfg_.confirm_frames = declare_parameter<int>("corridor_confirm_frames", params::CORRIDOR_CONFIRM_FRAMES);
        ccfg_.cloud_timeout = dd("corridor_cloud_timeout_s", params::CORRIDOR_CLOUD_TIMEOUT_S);
        ccfg_.cloud_window = dd("corridor_cloud_window_s", params::CORRIDOR_CLOUD_WINDOW_S);
        ccfg_.pose_timeout = dd("corridor_pose_timeout_s", params::CORRIDOR_POSE_TIMEOUT_S);
        corridor_z_below_ = dd("corridor_z_below", params::CORRIDOR_Z_BELOW);
        corridor_z_above_ = dd("corridor_z_above", params::CORRIDOR_Z_ABOVE);
        corridor_self_radius_ = dd("corridor_self_radius", params::CORRIDOR_SELF_RADIUS);
        corridor_sensor_range_ = dd("corridor_sensor_range", params::CORRIDOR_SENSOR_RANGE);
        corridor_cloud_max_yaw_rate_ = dd("corridor_cloud_max_yaw_rate", params::CORRIDOR_CLOUD_MAX_YAW_RATE);
        auto& pcfg = ccfg_.perception;
        pcfg.corridor_width = dd("corridor_width", params::CORRIDOR_WIDTH);
        pcfg.robot_width = dd("corridor_robot_width", params::CORRIDOR_ROBOT_WIDTH);
        pcfg.minimum_gap_extra = dd("corridor_min_gap_extra", params::CORRIDOR_MIN_GAP_EXTRA);
        pcfg.cell_size = dd("corridor_cell", params::CORRIDOR_CELL);
        pcfg.lookahead = corridor_sensor_range_;
        pcfg.lookbehind = dd("corridor_lookbehind", params::CORRIDOR_LOOKBEHIND);
        pcfg.wall_search_tolerance = dd("corridor_wall_search_m", params::CORRIDOR_WALL_SEARCH_M);
        pcfg.wall_exclusion_band = dd("corridor_wall_exclusion_m", params::CORRIDOR_WALL_EXCLUSION_M);
        pcfg.wall_min_span = dd("corridor_wall_min_span", params::CORRIDOR_WALL_MIN_SPAN);
        pcfg.wall_min_points = declare_parameter<int>("corridor_wall_min_points", params::CORRIDOR_WALL_MIN_POINTS);
        pcfg.max_wall_gap = dd("corridor_max_wall_gap", params::CORRIDOR_MAX_WALL_GAP);
        pcfg.gate_min_span = dd("corridor_gate_min_span", params::CORRIDOR_GATE_MIN_SPAN);
        pcfg.gate_cluster_depth = dd("corridor_gate_depth_cluster", params::CORRIDOR_GATE_DEPTH_CLUSTER);
        pcfg.gate_max_depth = dd("corridor_gate_max_depth", params::CORRIDOR_GATE_MAX_DEPTH);
        pcfg.surface_sample_gap = dd("corridor_surface_sample_gap", params::CORRIDOR_SURFACE_SAMPLE_GAP);
        pcfg.gate_min_points = declare_parameter<int>("corridor_gate_min_points", params::CORRIDOR_GATE_MIN_POINTS);
        pcfg.gate_min_clear_rays = declare_parameter<int>("corridor_gate_min_clear_rays", params::CORRIDOR_GATE_MIN_CLEAR_RAYS);
        pcfg.endpoint_exclusion = dd("corridor_endpoint_exclusion", params::CORRIDOR_ENDPOINT_EXCLUSION);
        pcfg.min_observed_ahead = dd("corridor_min_observed_ahead", params::CORRIDOR_MIN_OBSERVED_AHEAD);
        corridor_ = std::make_unique<CorridorController>(ccfg_);
        gcfg_.big_cell        = dd("big_cell", params::BIG_CELL);
        gcfg_.small_cell      = dd("small_cell", params::SMALL_CELL);
        gcfg_.coverage_thresh = dd("coverage_thresh", params::COVERAGE_THRESH);
        gcfg_.fov_deg         = dd("fov_deg", params::FOV_DEG);
        gcfg_.fov_range       = dd("fov_range", params::FOV_RANGE);

        // lane_spacing：旧静态牛耕(plan_boustrophedon)的车道间距。动态前沿覆盖已不用它，
        //   此处仍 declare 只为保持 ROS 参数表兼容(外部脚本/launch 可能仍在传)。读了不用是有意的。
        lane_spacing_ = dd("lane_spacing", params::LANE_SPACING);
        arc_ds_       = dd("arc_sample_ds", params::ARC_SAMPLE_DS);

        gains_.v_max           = dd("v_max", params::V_MAX);
        gains_.v_min           = dd("v_min", params::V_MIN);
        gains_.k_curv          = dd("k_curv", carlike_mode_ ? params::CARLIKE_K_CURV : params::K_CURV);
        gains_.lookahead       = dd("lookahead", carlike_mode_ ? params::CARLIKE_LOOKAHEAD : params::LOOKAHEAD);
        gains_.endpoint_slow_r = dd("endpoint_slow_r", params::ENDPOINT_SLOW_R);
        gains_.kp_yaw          = dd("kp_yaw", params::KP_YAW);
        gains_.kd_yaw          = dd("kd_yaw", carlike_mode_ ? params::CARLIKE_KD_YAW : params::KD_YAW);
        gains_.max_yaw_rate    = dd("max_yaw_rate", carlike_mode_ ? params::CARLIKE_MAX_YAW_RATE : params::MAX_YAW_RATE);
        gains_.kp_lat          = dd("kp_lat", params::KP_LAT);
        gains_.kd_lat          = dd("kd_lat", params::KD_LAT);
        gains_.max_v_lat       = dd("max_v_lat", params::MAX_V_LAT);
        gains_.heading_gate_rad =
            dd("heading_gate_deg",
               carlike_mode_ ? params::CARLIKE_HEADING_GATE_DEG : params::HEADING_GATE_DEG) * M_PI / 180.0;
        gains_.forward_only = carlike_mode_;
        gains_.max_accel = dd("carlike_max_accel", params::CARLIKE_MAX_ACCEL);
        gains_.max_yaw_accel = dd("carlike_max_yaw_accel", params::CARLIKE_MAX_YAW_ACCEL);
        gains_.yaw_filter_tau = dd("carlike_yaw_filter_s", params::CARLIKE_YAW_FILTER_S);
        gains_.prediction_time = dd("carlike_prediction_s", params::CARLIKE_PREDICTION_S);
        gains_.lateral_prediction_time = dd("carlike_lateral_prediction_s", params::CARLIKE_LATERAL_PREDICTION_S);
        gains_.align_resume_rad = dd("carlike_align_resume_deg", params::CARLIKE_ALIGN_RESUME_DEG) * M_PI / 180.0;
        gains_.align_stop_speed = dd("carlike_align_stop_speed", params::CARLIKE_ALIGN_STOP_SPEED);
        gains_.align_stop_yaw_rate = dd("carlike_align_stop_yaw_rate", params::CARLIKE_ALIGN_STOP_YAW_RATE);
        gains_.align_settle_s = dd("carlike_align_settle_s", params::CARLIKE_ALIGN_SETTLE_S);
        gains_.stop_align_rad = dd("carlike_stop_align_deg", params::CARLIKE_STOP_ALIGN_DEG) * M_PI / 180.0;
        gains_.corner_stop_rad = dd("carlike_corner_stop_deg", params::CARLIKE_CORNER_STOP_DEG) * M_PI / 180.0;
        gains_.max_lateral_accel = dd("carlike_max_lateral_accel", params::CARLIKE_MAX_LATERAL_ACCEL);
        turn_blend_m_ = dd("carlike_turn_blend_m", params::CARLIKE_TURN_BLEND_M);
        turn_blend_min_angle_rad_ =
            dd("carlike_turn_blend_angle_deg", params::CARLIKE_TURN_BLEND_ANGLE_DEG) * M_PI / 180.0;
        turn_blend_max_angle_rad_ = std::clamp(
            dd("turn_blend_max_angle_deg", params::TURN_BLEND_MAX_ANGLE_DEG), 0.0, 180.0) * M_PI / 180.0;
        turn_blend_samples_ = static_cast<int>(std::lround(
            dd("carlike_turn_blend_samples", static_cast<double>(params::CARLIKE_TURN_BLEND_SAMPLES))));
        turn_round_min_m_ = std::max(0.0, dd("turn_round_min_m", params::TURN_ROUND_MIN_M));
        turn_round_max_curvature_ = std::max(0.0,
            dd("turn_round_max_curvature", params::TURN_ROUND_MAX_CURVATURE));
        // D 项差分周期 = 主循环真实周期。★与 timer 同源推导★，勿写死常数(见 TrackerGains::dt)。
        gains_.dt = params::TIMER_PERIOD_MS / 1000.0;

        done_coverage_ = dd("done_coverage", params::DONE_COVERAGE);
        goal_tol_      = dd("goal_tol_xy", params::GOAL_TOL_XY);
        goal_stop_v_   = dd("goal_stop_v", params::GOAL_STOP_V);
        kp_goal_       = dd("kp_goal",    params::KP_GOAL);
        kd_goal_       = dd("kd_goal",    params::KD_GOAL);
        v_goal_max_    = dd("v_goal_max", params::V_GOAL_MAX);
        v_est_alpha_   = dd("v_est_alpha", params::V_EST_ALPHA);

        // 动态前沿覆盖配置
        fcfg_.min_x = gcfg_.min_x; fcfg_.min_y = gcfg_.min_y;
        fcfg_.max_x = gcfg_.max_x; fcfg_.max_y = gcfg_.max_y;
        fcfg_.big_cell       = gcfg_.big_cell;
        fcfg_.margin         = dd("wall_margin",       params::WALL_MARGIN);
        fcfg_.near_weight    = dd("frontier_near_w",   params::FRONTIER_NEAR_W);
        fcfg_.turn_penalty   = dd("frontier_turn_pen", params::FRONTIER_TURN_PEN);
        fcfg_.cluster_weight = dd("frontier_cluster_w",params::FRONTIER_CLUSTER_W);
        // 孤格饥饿修复：cluster 加成饱和上限（防遮挡小区被排到队尾、走远才回头补扫）
        fcfg_.cluster_cap    = static_cast<int>(std::lround(
                               dd("frontier_cluster_cap", static_cast<double>(params::FRONTIER_CLUSTER_CAP))));
        fcfg_.horizon        = dd("explore_horizon_m", params::EXPLORE_HORIZON_M);
        fcfg_.chain_gap      = dd("chain_gap_m",       params::CHAIN_GAP_M);
        fcfg_.band_width     = dd("band_width",        params::BAND_WIDTH);
        fcfg_.band_tol       = dd("band_tol",          params::BAND_TOL);
        fcfg_.along_bonus    = dd("along_bonus",       params::ALONG_BONUS);
        fcfg_.band_clear_cnt = static_cast<int>(std::lround(
                               dd("band_clear_cnt", static_cast<double>(params::BAND_CLEAR_CNT))));
        frontier_observation_enabled_ = declare_parameter<bool>(
            "frontier_observation_enabled", params::FRONTIER_OBSERVATION_ENABLED);
        fcfg_.small_region_cells = declare_parameter<int>("frontier_small_region_cells", params::FRONTIER_SMALL_REGION_CELLS);
        fcfg_.small_region_penalty = dd("frontier_small_region_penalty", params::FRONTIER_SMALL_REGION_PENALTY);
        fcfg_.observation_standoff = dd("frontier_observation_standoff", params::FRONTIER_OBSERVATION_STANDOFF);
        fcfg_.preferred_clearance = dd("frontier_preferred_clearance", params::FRONTIER_PREFERRED_CLEARANCE);
        fcfg_.gain_weight = dd("frontier_gain_weight", params::FRONTIER_GAIN_WEIGHT);
        fcfg_.clearance_weight = dd("frontier_clearance_weight", params::FRONTIER_CLEARANCE_WEIGHT);
        fcfg_.observation_min_distance = dd("frontier_observation_min_distance", params::FRONTIER_OBSERVATION_MIN_DISTANCE);
        fcfg_.continuity_turn_penalty = dd("frontier_continuity_turn_pen", params::FRONTIER_CONTINUITY_TURN_PEN);
        guide_min_segment_ = std::max(0.0, dd("explore_guide_min_segment_m", params::EXPLORE_GUIDE_MIN_SEGMENT_M));
        guide_max_deviation_ = std::max(0.0, dd("explore_guide_max_deviation_m", params::EXPLORE_GUIDE_MAX_DEVIATION_M));
        explore_plan_reserve_ = std::max(0.0, dd("explore_plan_reserve_m", params::EXPLORE_PLAN_RESERVE_M));
        frontier_continuity_lookahead_ = std::max(0.10,
            dd("frontier_continuity_lookahead_m", params::FRONTIER_CONTINUITY_LOOKAHEAD_M));
        replan_period_ = dd("replan_period_s", params::REPLAN_PERIOD_S);
        replan_dev_    = dd("replan_dev_m",    params::REPLAN_DEV_M);
        candidate_period_ = std::max(0.0,
            dd("replan_candidate_period_s", params::REPLAN_CANDIDATE_PERIOD_S));
        // 目标失效检测：承诺目标邻域此半径内已无未扫大格 → 放弃承诺改投别处。0=关闭
        target_stale_r_ = dd("target_stale_r", params::TARGET_STALE_R);

        viz_          = declare_parameter<bool>("viz", true);

        // ---- 雷达感知：点云聚类(obstacle_map) 配置 ----
        ground_z_      = dd("ground_z",        params::GROUND_Z);
        ceil_z_        = dd("obs_z_max",       params::OBS_Z_MAX);
        obs_skip_yaw_rate_ = dd("obs_skip_yaw_rate", params::OBS_SKIP_YAW_RATE);
        self_margin_   = dd("obs_self_margin", params::OBS_SELF_MARGIN);
        ocfg_.min_x = gcfg_.min_x; ocfg_.min_y = gcfg_.min_y;
        ocfg_.max_x = gcfg_.max_x; ocfg_.max_y = gcfg_.max_y;
        ocfg_.cell        = dd("obs_cell",        params::OBS_CELL);
        ocfg_.edge_ignore = dd("obs_edge_ignore", params::OBS_EDGE_IGNORE);
        ocfg_.hit_thresh  = static_cast<int>(std::lround(dd("obs_hit_thresh", static_cast<double>(params::OBS_HIT_THRESH))));
        ocfg_.hit_inc     = static_cast<int>(std::lround(dd("obs_hit_inc",   static_cast<double>(params::OBS_HIT_INC))));
        ocfg_.hit_max     = static_cast<int>(std::lround(dd("obs_hit_max",   static_cast<double>(params::OBS_HIT_MAX))));
        ocfg_.hit_decay   = static_cast<int>(std::lround(dd("obs_hit_decay", static_cast<double>(params::OBS_HIT_DECAY))));
        ocfg_.min_cells   = static_cast<int>(std::lround(dd("obs_min_cells",  static_cast<double>(params::OBS_MIN_CELLS))));
        ocfg_.min_r       = dd("obs_min_r",     params::OBS_MIN_R);
        ocfg_.max_r       = dd("obs_max_r",     params::OBS_MAX_R);
        ocfg_.r_inflate   = dd("obs_r_inflate", params::OBS_R_INFLATE);
        // 障碍时序平滑(跟踪)：压住障碍圆逐帧闪动/瞬移
        ocfg_.track_alpha      = dd("obs_track_alpha",      params::OBS_TRACK_ALPHA);
        ocfg_.track_assoc_dist = dd("obs_track_assoc_dist", params::OBS_TRACK_ASSOC_DIST);
        ocfg_.track_max_misses = static_cast<int>(std::lround(dd("obs_track_max_misses", static_cast<double>(params::OBS_TRACK_MAX_MISSES))));
        // latch 智能回收(防幽灵)：视野内连续无命中达帧数阈值 → 撤销 latch。FOV 取与栅格同源。
        ocfg_.fov_deg   = gcfg_.fov_deg;
        ocfg_.fov_range = gcfg_.fov_range;
        ocfg_.ghost_clear_frames = static_cast<int>(std::lround(dd("obs_ghost_clear_frames", static_cast<double>(params::OBS_GHOST_CLEAR_FRAMES))));

        robot_radius_ = dd("robot_radius", params::ROBOT_RADIUS);

        // ---- 全局点到点绕障（A*，全局层）配置：全场唯一避障手段 ----
        commit_target_tol_       = dd("commit_target_tol", params::COMMIT_TARGET_TOL);
        global_lookahead_        = dd("global_lookahead", params::GLOBAL_LOOKAHEAD);
        explore_target_min_dist_ = dd("explore_target_min_dist", params::EXPLORE_TARGET_MIN_DIST);
        // 换路评估(路径迟滞)：重算出新路后只有它更安全/更快超阈值才换边，根除左右横跳
        path_switch_safety_gain_  = dd("path_switch_safety_gain", params::PATH_SWITCH_SAFETY_GAIN);
        path_switch_time_gain_    = dd("path_switch_time_gain",   params::PATH_SWITCH_TIME_GAIN);
        path_switch_require_both_ = declare_parameter<bool>("path_switch_require_both", params::PATH_SWITCH_REQUIRE_BOTH);
        unreach_block_r_         = dd("unreach_block_r",   params::UNREACH_BLOCK_R);
        unreach_clear_step_      = dd("unreach_clear_step", params::UNREACH_CLEAR_STEP);
        retreat_trigger_         = dd("retreat_trigger",  params::RETREAT_TRIGGER_M);
        retreat_step_            = dd("retreat_step",     params::RETREAT_STEP_M);
        retreat_max_dist_        = dd("retreat_max_dist", params::RETREAT_MAX_DIST);
        retreat_v_max_           = dd("retreat_v_max",    params::RETREAT_V_MAX);
        retreat_timeout_         = dd("retreat_timeout_s", params::RETREAT_TIMEOUT_S);
        ggcfg_.min_x = gcfg_.min_x; ggcfg_.min_y = gcfg_.min_y;
        ggcfg_.max_x = gcfg_.max_x; ggcfg_.max_y = gcfg_.max_y;
        ggcfg_.cell         = dd("global_cell",        params::GLOBAL_CELL);
        ggcfg_.robot_radius = robot_radius_;
        ggcfg_.inflate      = dd("global_margin",      params::GLOBAL_MARGIN);
        fcfg_.minimum_clearance = ggcfg_.robot_radius + ggcfg_.inflate;
        ggcfg_.wall_margin  = dd("global_wall_margin", params::GLOBAL_WALL_MARGIN);
        home_goal_wall_margin_ = dd("home_goal_wall_margin", params::HOME_GOAL_WALL_MARGIN);
        if (!std::isfinite(home_goal_wall_margin_) || home_goal_wall_margin_ < 0.0 ||
            home_goal_wall_margin_ > ggcfg_.wall_margin)
            throw std::invalid_argument("home_goal_wall_margin must be between 0 and global_wall_margin");
        ggcfg_.required_connector_length = std::max(0.0,
            dd("required_connector_length", params::REQUIRED_CONNECTOR_LENGTH_M));
        required_motion_inflate_ = std::clamp(
            dd("required_motion_inflate", params::REQUIRED_MOTION_INFLATE),
            0.0, std::max(0.0, ggcfg_.inflate));
        required_blocked_replan_s_ = std::max(0.0,
            dd("required_blocked_replan_s", params::REQUIRED_BLOCKED_REPLAN_S));
        // ★机头锥★：A* 起点段只朝机头延伸(根除新路从侧后方起步→边转边走横切撞柱)
        ggcfg_.head_cone_half   = dd("global_head_cone_deg", params::GLOBAL_HEADING_CONE_DEG) * M_PI / 180.0;
        ggcfg_.head_cone_radius = dd("global_head_cone_radius", params::GLOBAL_HEADING_CONE_RADIUS);
        // ★原地转身找解★：锥内无解但有路(只是不在机头方向)时原地转身改朝向重搜
        turn_solve_yaw_rate_       = dd("turn_solve_yaw_rate", params::TURN_SOLVE_YAW_RATE);
        turn_solve_research_every_ = static_cast<int>(std::lround(
                                       dd("turn_solve_research_every", static_cast<double>(params::TURN_SOLVE_RESEARCH_EVERY))));
        turn_solve_max_rev_        = dd("turn_solve_max_rev", params::TURN_SOLVE_MAX_REV);
        turn_solve_timeout_        = dd("turn_solve_timeout_s", params::TURN_SOLVE_TIMEOUT_S);

        // 注意：不在这里造兜底终点。收到主控发来的 /exploration/goal 之前
        // has_goal_=false → 不规划、不画路线（弹窗只显示空场地+飞机）。

        // ---- 模块 ----
        grid_    = std::make_unique<GridMap>(gcfg_);
        tracker_ = std::make_unique<TrajectoryTracker>(gains_);
        global_gains_ = gains_;
        global_gains_.v_max = v_goal_max_;
        global_gains_.v_min = std::min(global_gains_.v_min, v_goal_max_);
        global_gains_.lookahead = global_lookahead_;
        global_gains_.endpoint_slow_r = std::max(global_gains_.endpoint_slow_r,
            std::max(0.0, gains_.prediction_time) * v_goal_max_ +
            v_goal_max_ * v_goal_max_ / (2.0 * std::max(1e-3, gains_.max_accel)));
        global_tracker_ = std::make_unique<TrajectoryTracker>(global_gains_);
        obs_map_ = std::make_unique<ObstacleMap>(ocfg_);
        if (viz_) viz_obj_ = std::make_unique<Visualizer>(gcfg_, 1000);  // 1000px：小格(0.05m≈5px)能看清

        // ---- ROS 接口 ----
        cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/exploration/cmd_vel", 10);

        rclcpp::QoS latched(1);
        latched.transient_local();
        finished_pub_ = create_publisher<std_msgs::msg::Bool>("/exploration/finished", latched);
        finished_goal_pub_ = create_publisher<std_msgs::msg::Header>("/exploration/finished_goal", latched);
        coverage_pub_ = create_publisher<std_msgs::msg::Float64>("/exploration/coverage", 1);
        active_path_pub_ = create_publisher<nav_msgs::msg::Path>("/exploration/active_path", 1);
        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/exploration/diagnostics", 1);
        corridor_active_pub_ = create_publisher<std_msgs::msg::Bool>("/exploration/corridor_active", latched);
        corridor_route_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/exploration/corridor_route", latched,
            [this](nav_msgs::msg::Path::SharedPtr route) {
                std::lock_guard<std::mutex> lk(mtx_);
                pending_corridor_route_ = *route;
                corridor_route_received_ = true;
                apply_corridor_route();
            });

        // 处理后障碍点云(rviz 看"算法当障碍的点")+ 设定边界框(rviz 画 FIELD_* 四面墙)。
        obs_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/exploration/obstacle_cloud", 5);
        boundary_pub_  = create_publisher<visualization_msgs::msg::Marker>("/exploration/field_boundary", latched);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
            std::bind(&ExplorationNode::on_odom, this, std::placeholders::_1));

        goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/exploration/goal", latched,
            std::bind(&ExplorationNode::on_goal, this, std::placeholders::_1));

        // POI（途中必经点/插点）订阅：放到独立 reentrant 回调组，
        // 配合 MultiThreadedExecutor → 后台监听，不被 20Hz timer 阻塞。
        poi_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions poi_opt;
        poi_opt.callback_group = poi_cbg_;
        poi_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/exploration/poi", rclcpp::QoS(10),
            std::bind(&ExplorationNode::on_poi, this, std::placeholders::_1), poi_opt);

        // 雷达点云订阅：独立 reentrant 回调组，后台聚类不被 20Hz timer 阻塞。
        cloud_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions cloud_opt;
        cloud_opt.callback_group = cloud_cbg_;
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", rclcpp::SensorDataQoS(),
            std::bind(&ExplorationNode::on_cloud, this, std::placeholders::_1), cloud_opt);
        if (corridor_cloud_topic_ != "/cloud_registered") {
            corridor_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                corridor_cloud_topic_, rclcpp::SensorDataQoS(),
                std::bind(&ExplorationNode::on_corridor_cloud, this, std::placeholders::_1), cloud_opt);
        }
        RCLCPP_INFO(get_logger(),
            "[走廊点云] 输入=%s，高度切片=飞机z-%.2f至z+%.2fm，范围=%.1fm，通道宽=%.2fm",
            corridor_cloud_topic_.c_str(), corridor_z_below_, corridor_z_above_,
            corridor_sensor_range_, ccfg_.perception.corridor_width);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(params::TIMER_PERIOD_MS),
            std::bind(&ExplorationNode::on_timer, this));

        last_plan_time_ = now();
        last_global_plan_time_ = now();
        retreat_start_  = now();   // 占位初始化；真正的起点在 do_retreat 首次进入时记

        RCLCPP_INFO(get_logger(),
            "exploration_planner 已启动 (场地 %.1fx%.1fm, 大格 %.2fm, 小格 %.2fm, 动态前沿覆盖, 跟踪=%s)",
            gcfg_.max_x - gcfg_.min_x, gcfg_.max_y - gcfg_.min_y,
            gcfg_.big_cell, gcfg_.small_cell,
            carlike_mode_ ? "车式前进" : "全向");
    }

    bool viz_enabled() const { return viz_; }

    // 主线程可视化循环：返回 false 表示窗口请求退出
    bool spin_viz_once()
    {
        if (!viz_ || !viz_obj_) return true;
        // 取快照（持锁尽量短）
        double px, py, yaw; bool pose_valid;
        Vec2 goal; bool goal_valid;
        Trajectory traj;
        Vec2 look; bool look_valid;
        std::vector<Vec2> pois;
        bool turning; int turn_dir;
        bool unreach_valid; Vec2 unreach_pos;
        CorridorVisualState corridor_visual;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            px = px_; py = py_; yaw = yaw_; pose_valid = has_pose_;
            goal = goal_; goal_valid = has_goal_;
            traj = traj_;                 // 拷贝（轨迹只在收到 goal 时变，拷贝不频繁）
            look = last_look_; look_valid = look_valid_;
            // 插点(目的地)：当前正去/等的 + 队列里待飞的，都画蓝点
            if (poi_mode_ != PoiMode::EXPLORE) pois.push_back(poi_target_);
            for (const auto& q : poi_queue_) pois.push_back(q);
            turning = turning_for_solution_; turn_dir = turn_dir_;
            unreach_valid = has_unreachable_marker_; unreach_pos = unreachable_pos_;
            corridor_visual.configured = corridor_->configured();
            corridor_visual.active = corridor_->active();
            corridor_visual.entry = corridor_->entry();
            corridor_visual.h = corridor_->goal();
            // A startup route is a display preview only. Execution still requires
            // the current goal's matching timestamp in apply_corridor_route().
            if (!corridor_visual.configured && corridor_route_received_ &&
                pending_corridor_route_.header.frame_id == "camera_init" &&
                pending_corridor_route_.poses.size() == 2) {
                const auto& entry = pending_corridor_route_.poses[0].pose.position;
                const auto& h = pending_corridor_route_.poses[1].pose.position;
                if (std::isfinite(entry.x) && std::isfinite(entry.y) &&
                    std::isfinite(h.x) && std::isfinite(h.y) &&
                    std::hypot(h.x - entry.x, h.y - entry.y) >= 0.5) {
                    corridor_visual.configured = true;
                    corridor_visual.entry = {entry.x, entry.y};
                    corridor_visual.h = {h.x, h.y};
                }
            }
            corridor_visual.width = ccfg_.perception.corridor_width;
            corridor_visual.phase = corridor_command_.status;
            if (!corridor_visual.active)
                corridor_visual.phase = has_goal_ ? "EXPLORATION" : "WAITING";
            corridor_visual.points = corridor_->points();
            corridor_visual.route = corridor_command_.path;
            corridor_visual.gates_passed = corridor_->gatesPassed();
            const auto& door = corridor_->gateLocked() ? corridor_->gate() : corridor_->observation();
            corridor_visual.gate_valid = door.gate_passable;
            corridor_visual.gate_center = door.gate_center;
            const double dx = corridor_visual.h.x - corridor_visual.entry.x;
            const double dy = corridor_visual.h.y - corridor_visual.entry.y;
            const double length = std::hypot(dx, dy);
            if (length > 1e-6) {
                const double half_gap = door.gap_width * 0.5;
                corridor_visual.gate_left = {door.gate_center.x - dy / length * half_gap,
                                             door.gate_center.y + dx / length * half_gap};
                corridor_visual.gate_right = {door.gate_center.x + dy / length * half_gap,
                                              door.gate_center.y - dx / length * half_gap};
            }
        }

        // 障碍圆（obs_map_ 自带锁，无需持 mtx_）→ 弹窗画绿圆
        const Obstacles obstacles = obs_map_->snapshot();
        // 栅格同理：GridMap 自带锁，snapshot() 在锁内拷出只读副本。
        //   ★勿改回传 *grid_★：那是在锁外读活地图，而 50Hz 主循环正在 mark_scan 里写它
        //   ——渲染一帧要几毫秒，期间数据被并发改写 = 数据竞争(撕裂画面 + UB)。
        const GridSnapshot grid_snap = grid_->snapshot();

        cv::Mat img = viz_obj_->render(grid_snap, traj, goal, goal_valid,
                                       px, py, yaw, pose_valid, look, look_valid,
                                       pois, obstacles,
                                       turning, turn_dir, unreach_valid, unreach_pos, corridor_visual);
        cv::imshow("Exploration", img);
        const int key = cv::waitKey(params::VIZ_PERIOD_MS);
        if (key == 27 /*ESC*/) return false;
        return true;
    }

private:
    static double steady_seconds()
    {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    // ---------- 回调 ----------
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Validate before constructing rclcpp::Time, which throws on negative stamps.
        if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[探索定位] 忽略时间戳无效的里程计帧");
            return;
        }
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        const double received_at = steady_seconds();

        double yaw = 0.0;
        const double qn = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(qn) || qn <= 1e-9) return;
        if (qn > 1e-9) {
            tf2::Quaternion tq(q.x, q.y, q.z, q.w);
            tf2::Matrix3x3 m(tq);
            double r, pi; m.getRPY(r, pi, yaw);
        }

        const rclcpp::Time stamp = msg->header.stamp;

        std::lock_guard<std::mutex> lk(mtx_);
        if (!pose_freshness_.observe(stamp.nanoseconds(), received_at)) return;
        // 位置差分估速度（local 系），再转机体系给 tracker 当 D 项参考
        if (has_prev_pose_) {
            const double dt = pose_freshness_.source_interval();
            if (dt > 1e-3) {
                const double vx = (p.x - prev_x_) / dt;
                const double vy = (p.y - prev_y_) / dt;
                // local → 机体：前进 = 投影到机头方向，横向 = 投影到左方向
                const double c = std::cos(yaw), s = std::sin(yaw);
                const double vf =  c * vx + s * vy;
                const double vl = -s * vx + c * vy;
                v_fwd_est_ = v_est_alpha_ * vf + (1.0 - v_est_alpha_) * v_fwd_est_;
                v_lat_est_ = v_est_alpha_ * vl + (1.0 - v_est_alpha_) * v_lat_est_;
                // yaw 差分 → 角速度(低通)：给点云高角速度门控用(旋转拖影帧丢弃)
                const double dyaw = wrap_pi(yaw - prev_yaw_);
                const double wz = dyaw / dt;
                yaw_rate_est_ = v_est_alpha_ * wz + (1.0 - v_est_alpha_) * yaw_rate_est_;
            } else if (dt == 0.0) {
                v_fwd_est_ = v_lat_est_ = yaw_rate_est_ = 0.0;
            }
        }
        prev_x_ = p.x; prev_y_ = p.y; prev_yaw_ = yaw; has_prev_pose_ = true;

        px_ = p.x; py_ = p.y; pz_ = p.z; yaw_ = yaw;
        has_pose_ = true;
    }

    void apply_corridor_route()
    {
        if (!has_goal_ || !corridor_route_received_ || corridor_->startedMoving()) return;
        const auto& route = pending_corridor_route_;
        if (route.header.stamp.sec != goal_stamp_.sec ||
            route.header.stamp.nanosec != goal_stamp_.nanosec || route.header.frame_id != "camera_init") return;
        if (route.poses.empty()) {
            corridor_disabled_for_goal_ = true;
            return;
        }
        if (route.poses.size() != 2) {
            RCLCPP_WARN(get_logger(), "走廊路线需要且仅需要入口/H两个点");
            return;
        }
        const auto& a = route.poses[0].pose.position;
        const auto& b = route.poses[1].pose.position;
        if (corridor_->configure({a.x, a.y}, {b.x, b.y})) {
            corridor_disabled_for_goal_ = false;
            RCLCPP_INFO(get_logger(), "走廊路线: 入口(%.2f,%.2f), H(%.2f,%.2f)", a.x, a.y, b.x, b.y);
        } else RCLCPP_WARN(get_logger(), "走廊坐标无效，到探索终点后悬停等待有效路线");
    }

    void on_goal(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y) ||
            msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u ||
            (!msg->header.frame_id.empty() && msg->header.frame_id != "camera_init")) {
            RCLCPP_WARN(get_logger(), "[探索任务] 忽略无效坐标、坐标系或时间戳的目标，保留当前任务");
            return;
        }
        std::lock_guard<std::mutex> lk(mtx_);
        const bool identified = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;
        if (has_goal_ && identified && msg->header == goal_header_) {
            if (msg->point.x != goal_.x || msg->point.y != goal_.y)
                RCLCPP_WARN(get_logger(), "[探索任务] 同一任务标识对应不同终点，忽略冲突消息；新任务需使用新时间戳");
            return;  // Re-delivery must not restart a turn, POI wait or completed mission.
        }
        vision_candidates_->reset();
        vision_queue_->reset();
        vision_near_->reset();
        vision_applied_visits_ = 0;
        vision_near_confirmations_ = 0;
        vision_last_event_ = "new_mission";
        vision_hover_track_ = 0;
        vision_hover_since_ = -1.0;
        reset_visual_approach();
        vision_new_confirmations_.clear();
        vision_collection_active_ = false;
        goal_ = {msg->point.x, msg->point.y};
        goal_stamp_ = msg->header.stamp;
        goal_header_ = msg->header;
        has_goal_ = true;
        finished_ = false;
        // fly_mission sends this goal after takeoff. Reuse the exact-goal
        // navigation and corridor handoff without waiting for coverage.
        homing_ = !exploration_enabled_;
        // Execution state belongs to this goal in both exploration modes.
        tracker_->set_trajectory({});
        traj_.clear();
        explore_raw_.clear();
        explore_has_committed_ = explore_failed_ = explore_goal_projected_ = false;
        cur_band_ = -1;
        look_valid_ = false;
        last_yaw_rate_ = 0.0;
        poi_mode_ = PoiMode::EXPLORE;
        poi_queue_.clear();
        poi_seen_.clear();
        release_pending_ = false;
        global_has_ = false;
        global_failed_ = global_goal_blocked_ = global_at_goal_ = global_braking_blocked_ = false;
        required_goal_holding_ = false;
        retreating_ = retreat_exhausted_ = false;
        command_projection_blocked_ = measured_projection_blocked_ = false;
        required_blocked_time_valid_ = false;
        global_tracker_->set_trajectory({});
        global_raw_.clear();
        global_traj_.clear();
        corridor_disabled_for_goal_ = false;
        corridor_->reset();
        cloud_freshness_ = SensorFreshness{};
        corridor_command_ = {};
        apply_corridor_route();
        std_msgs::msg::Bool reset;
        reset.data = false;
        finished_pub_->publish(reset);
        corridor_active_pub_->publish(reset);
        plan_pending_ = exploration_enabled_;   // 直达模式由必达点规划器处理
        candidate_time_valid_ = false;
        unreachable_.clear();            // 换终点=换任务：清空够不到黑名单，所有区重新给机会
        last_unreach_clear_cov_ = 0.0;
        turning_for_solution_ = false;   // 换任务：打断原地转身找解
        have_observation_target_ = false;
        observation_arrived_ = false;
        observation_yaw_rate_ = 0.0;
        observation_turn_active_ = false;
        has_unreachable_marker_ = false; // 清红叉
        RCLCPP_INFO(get_logger(), "收到探索终点: (%.2f, %.2f)，%s", goal_.x, goal_.y,
            exploration_enabled_ ? "先探索，覆盖率达标后前往终点" : "探索已关闭，直接前往终点");
    }

    // POI（途中必经点/插点）回调。约定：
    //   point.x/y = SLAM 坐标；point.z = 标志位：0=发现插点(去飞)，1=放行(继续探索)。
    //   去重：仅对 z=0 的"插点触发"按 (x,y) 量化指纹去重，相同点只入队一次；
    //         z=1 永远当放行信号(不入队/不去重)，仅在 WAIT_RELEASE 且坐标匹配当前等待点时生效。
    //   主控可重复狂发，本回调天然幂等。
    void on_poi(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        const double x = msg->point.x, y = msg->point.y;
        // Check before quantizing coordinates into an integer deduplication key.
        constexpr double key_limit = (std::numeric_limits<int32_t>::max() - (1LL << 20)) * 0.05;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(msg->point.z) ||
            std::abs(x) > key_limit || std::abs(y) > key_limit ||
            (!msg->header.frame_id.empty() && msg->header.frame_id != "camera_init")) {
            RCLCPP_WARN(get_logger(), "[插点] 忽略无效坐标/坐标系的消息");
            return;
        }
        const bool release = (msg->point.z >= 0.5);   // z=1 放行

        std::lock_guard<std::mutex> lk(mtx_);
        if (!exploration_enabled_) return;

        if (release) {
            // 放行信号：只在正等待、且坐标匹配当前等待点(±POI_SAME_TOL)时接管继续探索
            release_pending_ = true;
            release_x_ = x; release_y_ = y;
            return;
        }

        // 插点触发(z=0)：量化指纹去重
        const uint64_t key = poi_key(x, y);
        if (poi_seen_.count(key)) return;   // 同一插点已收到过 → 丢弃
        poi_seen_.insert(key);
        poi_queue_.push_back({x, y});
        RCLCPP_INFO(get_logger(), "收到插点 #%zu: (%.2f, %.2f)，入队", poi_queue_.size(), x, y);
    }

    // (x,y) 量化成 5cm 栅格指纹（去重粒度）。负坐标偏移后取整，拼成 64 位 key。
    static uint64_t poi_key(double x, double y)
    {
        const int64_t gx = static_cast<int64_t>(std::llround(x / 0.05)) + (1LL << 20);
        const int64_t gy = static_cast<int64_t>(std::llround(y / 0.05)) + (1LL << 20);
        return (static_cast<uint64_t>(gx) << 32) | static_cast<uint64_t>(gy & 0xffffffff);
    }

    void on_corridor_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const double received_at = steady_seconds();
        const CloudXYZ xyz(*msg);
        double px, py, pz, wz; bool ok;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            px = px_; py = py_; ok = has_pose_;
            pz = pz_;
            wz = yaw_rate_est_;
        }

        // Keep receipt separate from acceptance, so a held entry can report
        // missing input, frame/yaw rejection or a completely filtered scan.
        std::string rejection;
        if (!corridor_enabled_) rejection = "disabled";
        else if (!ok) rejection = "no_pose";
        else if (!xyz.valid()) rejection = "invalid_xyz_layout";
        else if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u)
            rejection = "invalid_stamp";
        else if (!msg->header.frame_id.empty() && msg->header.frame_id != "camera_init")
            rejection = "wrong_frame";
        else if (!(std::fabs(wz) <= corridor_cloud_max_yaw_rate_)) rejection = "yaw_rate_exceeded";
        const size_t raw_points = static_cast<size_t>(msg->width) * msg->height;
        if (!rejection.empty()) {
            std::lock_guard<std::mutex> lk(mtx_);
            corridor_cloud_received_at_ = received_at;
            corridor_cloud_raw_points_ = raw_points;
            corridor_cloud_height_points_ = corridor_cloud_kept_points_ = 0;
            corridor_cloud_filter_reason_ = rejection;
            return;
        }

        // 走廊使用独立的机身高度切片，保留墙/横档的实际点，不做障碍圆拟合和探索膨胀。
        if (rejection.empty()) {
            Path2 corridor_points;
            size_t height_points = 0;
            double min_z = std::numeric_limits<double>::infinity();
            double max_z = -std::numeric_limits<double>::infinity();
            xyz.for_each([&](float x, float y, float z) {
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
                min_z = std::min(min_z, static_cast<double>(z));
                max_z = std::max(max_z, static_cast<double>(z));
                if (z < pz - corridor_z_below_ || z > pz + corridor_z_above_) return;
                ++height_points;
                const double d = std::hypot(x - px, y - py);
                if (d < corridor_self_radius_ || d > corridor_sensor_range_) return;
                corridor_points.push_back({x, y});
            });
            std::lock_guard<std::mutex> lk(mtx_);
            corridor_cloud_received_at_ = received_at;
            corridor_cloud_raw_points_ = raw_points;
            corridor_cloud_height_points_ = height_points;
            corridor_cloud_kept_points_ = corridor_points.size();
            corridor_cloud_min_z_ = min_z;
            corridor_cloud_max_z_ = max_z;
            corridor_cloud_filter_reason_ = corridor_cloud_raw_points_ == 0 ? "empty_input" :
                (height_points == 0 ? "height_slice_empty" :
                (corridor_points.empty() ? "range_slice_empty" : "accepted"));
            if (cloud_freshness_.observe(rclcpp::Time(msg->header.stamp).nanoseconds(), received_at)) {
                corridor_->observe(corridor_points, received_at, {px, py});
            } else {
                corridor_cloud_filter_reason_ = "nonadvancing_stamp";
                if (!cloud_freshness_.fresh(received_at, ccfg_.cloud_timeout))
                    corridor_->observe({}, received_at);
            }
        }
    }

    // Registered downsampled scans still feed the exploration obstacle map.
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (corridor_cloud_topic_ == "/cloud_registered") on_corridor_cloud(msg);
        const CloudXYZ xyz(*msg);
        if (!xyz.valid() ||
            (!msg->header.frame_id.empty() && msg->header.frame_id != "camera_init")) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[探索点云] 忽略 XYZ 结构或坐标系不正确的点云");
            return;
        }
        // An empty packet supplies no free-space evidence for obstacle removal.
        if (msg->width == 0 || msg->height == 0) return;
        double px, py, yaw, wz; bool ok, corridor_active;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            px = px_; py = py_; yaw = yaw_; wz = yaw_rate_est_;
            ok = has_pose_; corridor_active = corridor_->active();
        }
        if (corridor_active) return;

        // ★高角速度门控★：飞机快速旋转时点云拖影(点被甩到障碍外侧)会污染累积、被 latch 钉成虚胖大圆。
        //   整帧丢弃(不累积、不 latch)；旋转停下后正常帧补回真实边界。阈值<=0 关闭。
        if (obs_skip_yaw_rate_ > 0.0 && std::fabs(wz) > obs_skip_yaw_rate_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "旋转过快(|yaw_rate|=%.2f>%.2f rad/s)→丢弃本帧点云防拖影污染", std::fabs(wz), obs_skip_yaw_rate_);
            return;
        }

        const double self_r = robot_radius_ + self_margin_;
        const double self_r2 = self_r * self_r;

        std::vector<Vec2> pts;
        pts.reserve(static_cast<size_t>(msg->width) * msg->height / 2 + 1);

        xyz.for_each([&](float x, float y, float z) {
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
            if (z < ground_z_) return;                 // 高度窗口·下限：地面/近地杂物滤掉
            if (z > ceil_z_)   return;                 // 高度窗口·上限：天花板/高处墙面/吊挂物滤掉
            if (ok) {
                const double dx = x - px, dy = y - py;
                if (dx * dx + dy * dy < self_r2) return; // 自身回波：滤掉
            }
            pts.push_back({static_cast<double>(x), static_cast<double>(y)});
        });

        obs_map_->integrate(pts, px, py, yaw, ok);   // 传位姿：latch 智能回收判"格是否在当前视野内"
    }

    // 采纳一条探索绕障路径(replan 成功 与 转身重搜成功 共用)：设激活轨迹、清失败态、撤红叉。
    //   需在持有 mtx_ 时调用。
    bool adopt_explore_path(const GlobalResult& gr, const Vec2& scan_target,
                            const FrontierSelection* observation = nullptr)
    {
        explore_handoff_deferred_ = false;
        // Restore one continuously sampled curve. Validate both the resulting
        // curve and its fallback; a rejected curve must never be executed.
        const Obstacles transition_obs = obs_map_->snapshot();
        if (gr.path.size() < 2 || !path_inside_safe_field(gr.path) ||
            !path_clear(gr.path.front(), gr.path, transition_obs, ggcfg_)) {
            ++invalid_paths_;
            return false;
        }
        const Path2 blended = blend_explore_transition(clean_explore_guides(gr.path, transition_obs), transition_obs);
        // Intermediate samples describe a single curve, not stop waypoints.
        const Trajectory smooth_traj = smooth_catmull_rom(blended, arc_ds_);
        Path2 smooth_points;
        smooth_points.reserve(smooth_traj.size());
        for (const auto& tp : smooth_traj) smooth_points.push_back(tp.p);

        // Catmull-Rom 可能在急弯处向障碍内侧切入；最终执行轨迹也必须满足同一
        // 安全口径。若平滑轨迹不安全，退回已经逐段校验过的采样折线。
        const bool field_safe = smooth_points.size() >= 2 && path_inside_safe_field(smooth_points);
        const bool obstacle_safe = field_safe && path_clear(blended.front(), smooth_points, transition_obs, ggcfg_);
        const bool smooth_safe = field_safe && obstacle_safe;
        const Trajectory execution = smooth_safe ? smooth_traj : make_polyline_trajectory(blended);
        if (carlike_mode_ && explore_has_committed_ && tracker_->has_trajectory() &&
            tracker_->remaining_distance() <= global_lookahead_ &&
            !observation_arrived_ && tracker_->remaining_distance() > goal_tol_ &&
            path_clear({px_, py_}, tracker_->remaining_path(px_, py_), transition_obs, ggcfg_)) {
            // Finish the checked incoming segment before a reverse handoff.
            // Arrival, rather than an unattainable zero-speed state, releases it.
            // Use the actual candidate and tracker gate so flowing curves are
            // unaffected; the old validated endpoint provides its normal brake.
            TrajectoryTracker preview(gains_);
            preview.set_trajectory(execution);
            preview.update(px_, py_, yaw_, v_fwd_est_, v_lat_est_, goal_tol_, yaw_rate_est_);
            if (preview.reorienting()) {
                explore_handoff_deferred_ = true;
                last_replan_reason_ = "brake_before_reverse_handoff";
                plan_pending_ = false;
                last_plan_time_ = now();
                return false;
            }
        }
        ++adoptions_;
        explore_raw_ = blended;
        smooth_failure_reason_ = smooth_safe ? "none" : (field_safe ? "obstacle" : "field");
        if (smooth_safe) {
            traj_ = execution;
        } else {
            ++smooth_fallbacks_;
            traj_ = execution;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                "平滑轨迹切入安全区 → 退回安全折线跟踪(障碍外表面余量 %.2fm)",
                ggcfg_.inflate + ggcfg_.robot_radius);
        }
        explore_failed_         = false;
        retreat_exhausted_      = false;
        explore_has_committed_  = true;
        explore_target_         = scan_target;
        explore_goal_projected_ = std::hypot(gr.path.back().x - scan_target.x,
                                             gr.path.back().y - scan_target.y) > 1e-6;
        have_observation_target_ = observation && observation->valid;
        observation_arrived_ = false;
        if (have_observation_target_) {
            observation_target_ = *observation;
            observation_target_.point = gr.path.back();
            explore_target_ = gr.path.back();
            // Normal viewpoints observe along the executed arrival tangent.
            // Only the explicit cleanup fallback requests a separate look-at turn.
            observation_target_.view_heading = observation_target_.requires_turn
                ? std::atan2(observation_target_.look_at.y - explore_target_.y,
                             observation_target_.look_at.x - explore_target_.x)
                : traj_.back().theta;
        }
        observation_yaw_rate_ = 0.0;
        observation_hold_.begin(explore_target_);
        observation_turn_active_ = false;
        tracker_->set_trajectory(traj_);
        plan_pending_           = false;
        last_plan_time_         = now();
        has_unreachable_marker_ = false;                         // 又有路了 → 撤红叉
        return true;
    }

    // 检查轨迹采样点是否都在四面墙的安全内缩区域内。
    bool path_inside_safe_field(const Path2& path) const
    {
        return exploration::path_inside_safe_field(path, ggcfg_);
    }

    bool turn_correction_clear(Vec2 correction, const Obstacles& obstacles) const
    {
        const Vec2 cur{px_, py_};
        const double c = std::cos(yaw_), s = std::sin(yaw_);
        const Vec2 world{c * correction.x - s * correction.y, s * correction.x + c * correction.y};
        const double horizon = std::max(0.5, std::hypot(world.x, world.y) /
                                              (2.0 * gains_.max_accel));
        const Vec2 projected{cur.x + world.x * horizon, cur.y + world.y * horizon};
        return field_motion_clear(cur, projected, ggcfg_) &&
               obstacle_segment_clear(cur, projected, obstacles, ggcfg_);
    }

    Vec2 turn_position_correction(PositionHold& hold, const Obstacles& obstacles)
    {
        const auto correction = hold.update({px_, py_}, yaw_, v_fwd_est_, v_lat_est_,
            kp_goal_, kd_goal_, std::min(0.20, v_goal_max_), gains_.max_accel, gains_.dt);
        if (!turn_correction_clear(correction, obstacles)) {
            hold.stop();
            return {};
        }
        return correction;
    }

    void finish_observation(VelCmd& command, const Obstacles& obstacles)
    {
        if (!have_observation_target_ || (!command.at_goal && !observation_arrived_)) return;
        if (!observation_arrived_) observation_hold_.begin({px_, py_});
        observation_arrived_ = true;
        const Vec2 cur{px_, py_};
        const auto snapshot = grid_->snapshot();
        const double heading = observation_target_.view_heading;
        const double error = wrap_pi(heading - yaw_);
        const bool need_turn = !observation_turn_active_ &&
            visible_unknown_count(snapshot, cur, yaw_, obstacles, fcfg_) == 0 &&
            visible_unknown_count(snapshot, cur, heading, obstacles, fcfg_) > 0 && std::abs(error) > 0.10;
        if (need_turn) observation_turn_active_ = true;
        command = {};
        const auto correction = turn_position_correction(observation_hold_, obstacles);
        command.v_fwd = correction.x;
        command.v_lat = correction.y;
        if (!observation_turn_active_ || std::abs(error) <= 0.10) {
            // This viewpoint has been sampled. Try another viewpoint if its
            // partially occluded cells still need coverage; never mark them here.
            unreachable_.push_back(explore_target_);
            have_observation_target_ = false;
            observation_arrived_ = false;
            explore_has_committed_ = false;
            tracker_->set_trajectory({});
            traj_.clear();
            plan_pending_ = true;
            observation_yaw_rate_ = 0.0;
            observation_turn_active_ = false;
            return;
        }
        if (!obstacle_segment_clear(cur, cur, obstacles, ggcfg_)) {
            observation_yaw_rate_ = 0.0;
            return;
        }
        const double rate = std::clamp(gains_.kp_yaw * error - gains_.kd_yaw * yaw_rate_est_,
                                       -gains_.max_yaw_rate, gains_.max_yaw_rate);
        observation_yaw_rate_ += std::clamp(rate - observation_yaw_rate_,
            -gains_.max_yaw_accel * gains_.dt, gains_.max_yaw_accel * gains_.dt);
        command.yaw_rate = observation_yaw_rate_;
    }

    // 把已经碰撞校验过的折线转换成 tracker 可用的轨迹，作为平滑曲线不安全时的兜底。
    Trajectory make_polyline_trajectory(const Path2& path) const
    {
        Trajectory out;
        if (path.empty()) return out;

        Path2 clean;
        clean.reserve(path.size());
        for (const auto& p : path) {
            if (clean.empty() || std::hypot(p.x - clean.back().x, p.y - clean.back().y) > 1e-6)
                clean.push_back(p);
        }
        if (clean.empty()) return out;

        out.resize(clean.size());
        double s = 0.0;
        for (size_t i = 0; i < clean.size(); ++i) {
            if (i > 0) s += std::hypot(clean[i].x - clean[i - 1].x,
                                       clean[i].y - clean[i - 1].y);
            out[i].p = clean[i];
            out[i].s = s;
            out[i].kappa = 0.0;
            const size_t ia = (i == 0) ? 0 : i - 1;
            const size_t ib = (i + 1 < clean.size()) ? i + 1 : i;
            out[i].theta = std::atan2(clean[ib].y - clean[ia].y,
                                      clean[ib].x - clean[ia].x);
        }
        return out;
    }

    Path2 clean_explore_guides(const Path2& path, const Obstacles& obs) const
    {
        Path2 result = path;
        for (size_t i = 1; i + 1 < result.size();) {
            const Vec2 a = result[i - 1], b = result[i], c = result[i + 1];
            const double dx = c.x - a.x, dy = c.y - a.y;
            const double length2 = dx * dx + dy * dy;
            const double shorter = std::min(std::hypot(b.x - a.x, b.y - a.y),
                                            std::hypot(c.x - b.x, c.y - b.y));
            const double u = length2 > 1e-12
                ? ((b.x - a.x) * dx + (b.y - a.y) * dy) / length2 : -1.0;
            const double deviation = std::hypot(b.x - a.x - u * dx, b.y - a.y - u * dy);
            const Path2 shortcut{a, c};
            if (shorter < guide_min_segment_ && u >= 0.0 && u <= 1.0 &&
                deviation <= guide_max_deviation_ && path_inside_safe_field(shortcut) &&
                path_clear(a, shortcut, obs, ggcfg_)) {
                result.erase(result.begin() + i);
                if (i > 1) --i;
            } else {
                ++i;
            }
        }
        return result;
    }

    // 用实际航向接入第一条引导段，再对完整引导路线统一生成连续曲线。
    // 起步连接必须经过障碍和场界检查，不插入需要回追的单格起步点。
    Path2 blend_explore_transition(const Path2& path, const Obstacles& obs)
    {
        if (!carlike_mode_ || path.size() < 2 || turn_blend_m_ <= 0.0)
            return path;

        const auto safe = [&](const Path2& candidate) {
            return candidate.size() >= 2 && path_inside_safe_field(candidate) &&
                path_clear(candidate.front(), candidate, obs, ggcfg_);
        };
        // Keep original guide vertices for continuous Catmull-Rom smoothing.
        const Path2 rounded = path;
        if (rounded.size() < 2) return path;

        const Vec2 p0 = rounded.front();
        const Vec2 p3 = rounded[1];
        const double d03 = std::hypot(p3.x - p0.x, p3.y - p0.y);
        if (d03 < 1e-3) return rounded;

        // Only join the measured body heading to the first line. An old raw
        // tangent may already be stale, and the next line belongs to its own
        // interior corner; neither may bend this initial segment backwards.
        const Vec2 old_dir{std::cos(yaw_), std::sin(yaw_)};
        const Vec2 new_dir{(p3.x - p0.x) / d03, (p3.y - p0.y) / d03};
        const double dot = std::clamp(old_dir.x * new_dir.x + old_dir.y * new_dir.y, -1.0, 1.0);
        const double angle = std::acos(dot);
        if (angle < turn_blend_min_angle_rad_ || angle > turn_blend_max_angle_rad_) return rounded;

        const double handle = std::min(turn_blend_m_, d03 * 0.45);
        if (handle < turn_round_min_m_) return rounded;

        const Vec2 c1{p0.x + old_dir.x * handle, p0.y + old_dir.y * handle};
        const Vec2 c2{p3.x - new_dir.x * handle, p3.y - new_dir.y * handle};

        const int samples = std::max({4, turn_blend_samples_,
            static_cast<int>(std::ceil((d03 + 2.0 * handle) / std::max(.005, arc_ds_)))});
        Path2 blended;
        blended.reserve(static_cast<size_t>(samples) + rounded.size());
        blended.push_back(p0);
        for (int k = 1; k <= samples; ++k) {
            const double t = static_cast<double>(k) / samples;
            const double u = 1.0 - t;
            const double w0 = u * u * u;
            const double w1 = 3.0 * u * u * t;
            const double w2 = 3.0 * u * t * t;
            const double w3 = t * t * t;
            blended.push_back({
                w0 * p0.x + w1 * c1.x + w2 * c2.x + w3 * p3.x,
                w0 * p0.y + w1 * c1.y + w2 * c2.y + w3 * p3.y});
        }
        // Evaluate the cubic derivatives more densely than the executed
        // samples before accepting a start transition. Curvature clipping
        // would conceal a hook rather than make it flyable.
        for (int sample = 0; sample <= samples * 4; ++sample) {
            const double t = static_cast<double>(sample) / (samples * 4), u = 1.0 - t;
            const Vec2 derivative{
                3 * u * u * (c1.x - p0.x) + 6 * u * t * (c2.x - c1.x) + 3 * t * t * (p3.x - c2.x),
                3 * u * u * (c1.y - p0.y) + 6 * u * t * (c2.y - c1.y) + 3 * t * t * (p3.y - c2.y)};
            const Vec2 second{
                6 * u * (c2.x - 2 * c1.x + p0.x) + 6 * t * (p3.x - 2 * c2.x + c1.x),
                6 * u * (c2.y - 2 * c1.y + p0.y) + 6 * t * (p3.y - 2 * c2.y + c1.y)};
            const double speed = std::hypot(derivative.x, derivative.y);
            if (speed < 1e-6 || derivative.x * new_dir.x + derivative.y * new_dir.y < -1e-9 ||
                std::abs(derivative.x * second.y - derivative.y * second.x) /
                    (speed * speed * speed) > turn_round_max_curvature_) return rounded;
        }
        for (size_t i = 2; i < rounded.size(); ++i) blended.push_back(rounded[i]);

        if (!safe(blended)) return rounded;
        return blended;
    }

    // 进入【原地转身找解】：锥内无解但无锥 probe 有解(=有路只是不在机头方向)时调用。
    //   转身方向取朝 probe 给出的"开口"方向(最短转到位)。需在持有 mtx_ 时调用。
    void enter_turn_for_solution(const Vec2& cur, const Vec2& scan_target,
                                 const GlobalResult& probe, int candidate_band,
                                 const FrontierSelection* observation = nullptr)
    {
        ++turn_entries_;
        turning_for_solution_   = true;
        turn_target_            = scan_target;
        turn_hold_.begin(cur);
        turn_candidate_band_    = candidate_band;
        turn_observation_ = observation ? *observation : FrontierSelection{};
        retreating_             = false;          // 与后退互斥
        turn_prev_yaw_          = yaw_;
        turn_accum_             = 0.0;
        turn_start_time_        = now();
        turn_research_tick_     = 0;
        has_unreachable_marker_ = false;
        // probe 首段方位 = 存在解的起步方向；锥内失败⇒|该方位−yaw|>锥半角，sign 必有定义。
        double bx = (probe.path.size() >= 2) ? probe.path[1].x - cur.x : scan_target.x - cur.x;
        double by = (probe.path.size() >= 2) ? probe.path[1].y - cur.y : scan_target.y - cur.y;
        const double bearing = (std::hypot(bx, by) > 1e-6) ? std::atan2(by, bx)
                                                           : std::atan2(scan_target.y - cur.y, scan_target.x - cur.x);
        turn_heading_ = bearing;
        turn_heading_valid_ = false;  // Validate once at the actual turn-start position.
        turn_probe_path_.clear();
        turn_command_rate_ = 0.0;
        turn_settled_s_ = 0.0;
        turn_dir_ = (wrap_pi(bearing - yaw_) >= 0.0) ? +1 : -1;
    }

    // 原地转身找解一拍。返回 true=本拍已写 cmd(继续转 / 刚判真围死悬停)；
    //   false=刚解出(turning_for_solution_ 已置 false、traj_ 已设)→交外层本拍跟随。
    //   需在持有 mtx_ 时调用。
    bool step_turn_for_solution(const Obstacles& obs, geometry_msgs::msg::TwistStamped& cmd)
    {
        const Vec2 cur{px_, py_};

        // 1) 累计净转角(带符号 *turn_dir_)：小幅来回正负相消不虚增，firm yaw_rate 远大于里程计噪声→单调逼近
        turn_accum_   += turn_dir_ * wrap_pi(yaw_ - turn_prev_yaw_);
        turn_prev_yaw_ = yaw_;

        if (!turn_heading_valid_) {
            auto fresh = search_explore_global(cur, turn_target_, obs);
            if (fresh.ok && fresh.path.size() >= 2) {
                fresh.path = clean_explore_guides(fresh.path, obs);
                turn_probe_path_ = fresh.path;
                turn_heading_ = std::atan2(fresh.path[1].y - cur.y, fresh.path[1].x - cur.x);
                turn_heading_valid_ = true;
                turn_dir_ = wrap_pi(turn_heading_ - yaw_) >= 0 ? 1 : -1;
                turn_accum_ = 0.0;
                turn_settled_s_ = 0.0;
                turn_research_tick_ = 0;
            } else {
                turning_for_solution_ = false;
                unreachable_.push_back(turn_target_);
                plan_pending_ = false;
                last_plan_time_ = now();
                last_replan_reason_ = "turn_start_probe_unavailable";
                cmd.twist.linear.x = cmd.twist.linear.y = cmd.twist.angular.z = 0.0;
                RCLCPP_WARN(get_logger(), "转向起点到当前观察点暂无路径，等待下一轮选点 (%.2f,%.2f)",
                    turn_target_.x, turn_target_.y);
                return true;
            }
        }
        const double error = wrap_pi(turn_heading_ - yaw_);
        const double heading_tolerance = std::min(gains_.align_resume_rad,
            ggcfg_.head_cone_half > 0 ? ggcfg_.head_cone_half * .5 : gains_.align_resume_rad);
        const bool settled = turn_heading_valid_ &&
            std::abs(yaw_rate_est_) <= gains_.align_stop_yaw_rate &&
            std::abs(error) <= heading_tolerance;
        turn_settled_s_ = settled ? turn_settled_s_ + gains_.dt : 0.0;
        // A transient open cone while still spinning is not a safe handoff.
        // Keep one heading target until angular momentum has settled.
        if (++turn_research_tick_ >= turn_solve_research_every_ &&
            turn_settled_s_ >= std::max(gains_.dt, gains_.align_settle_s)) {
            turn_research_tick_ = 0;
            GlobalResult gr;
            if (turn_probe_path_.size() >= 2) {
                Path2 cached = turn_probe_path_;
                cached.front() = cur;
                const double bearing = std::atan2(cached[1].y - cur.y, cached[1].x - cur.x);
                if (std::abs(wrap_pi(bearing - yaw_)) <= heading_tolerance &&
                    path_inside_safe_field(cached) && path_clear(cur, cached, obs, ggcfg_)) {
                    gr.ok = true;
                    gr.path = std::move(cached);
                }
            }
            if (!gr.ok) gr = search_explore_global(cur, turn_target_, obs, yaw_);
            if (!gr.ok) {
                auto aligned = search_explore_global(cur, turn_target_, obs);
                if (aligned.ok && aligned.path.size() >= 2) {
                    aligned.path = clean_explore_guides(aligned.path, obs);
                    const auto& a = aligned.path[0];
                    const auto& b = aligned.path[1];
                    const double fresh_heading = std::atan2(b.y - a.y, b.x - a.x);
                    if (std::abs(wrap_pi(fresh_heading - yaw_)) <= heading_tolerance) {
                        gr = std::move(aligned);
                    } else {
                        // The obstacle view or stopping position changed. A
                        // reachable target is not made unreachable by facing
                        // the stale probe bearing from before that change.
                        turn_heading_ = fresh_heading;
                        turn_probe_path_ = aligned.path;
                        turn_settled_s_ = 0.0;
                        turn_dir_ = wrap_pi(fresh_heading - yaw_) >= 0 ? 1 : -1;
                    }
                }
            }
            if (gr.ok && gr.path.size() >= 2 && adopt_explore_path(gr, turn_target_, &turn_observation_)) {
                turning_for_solution_ = false;
                cur_band_ = turn_candidate_band_;
                RCLCPP_INFO(get_logger(),
                    "原地转身找解成功 → 机头方向出现可走路径，沿新方向继续 (目标 %.2f,%.2f)",
                    turn_target_.x, turn_target_.y);
                return false;
            }
        }

        // A turn timeout is not proof that a whole region is unreachable.
        const bool full_circle = std::fabs(turn_accum_) >= 2.0 * M_PI * turn_solve_max_rev_;
        const bool timed_out   = (now() - turn_start_time_).seconds() > turn_solve_timeout_;
        if (full_circle || timed_out) {
            turning_for_solution_   = false;
            unreachable_.push_back(turn_target_);
            plan_pending_           = false;
            last_plan_time_ = now();
            last_replan_reason_ = "turn_timeout_retry";
            has_unreachable_marker_ = true;
            unreachable_pos_        = turn_target_;
            cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
            RCLCPP_WARN(get_logger(),
                "本轮转向未完成 → 暂缓观察点 (%.2f,%.2f)，下一规划周期重新选点",
                turn_target_.x, turn_target_.y);
            return true;
        }

        // XY position correction and yaw are one action. Horizontal drift must
        // not pause the turn or relatch its world-frame anchor.
        double desired_rate = 0.0;
        if (turn_heading_valid_) {
            const double predicted = error - gains_.prediction_time * yaw_rate_est_;
            const double limit = std::sqrt(2.0 * gains_.max_yaw_accel * std::abs(predicted));
            const double maximum = std::min(turn_solve_yaw_rate_, gains_.max_yaw_rate);
            desired_rate = std::clamp(std::clamp(gains_.kp_yaw * predicted, -limit, limit) -
                gains_.kd_yaw * yaw_rate_est_, -maximum, maximum);
        }
        turn_command_rate_ += std::clamp(desired_rate - turn_command_rate_,
            -gains_.max_yaw_accel * gains_.dt, gains_.max_yaw_accel * gains_.dt);
        const auto correction = turn_position_correction(turn_hold_, obs);
        cmd.twist.linear.x  = correction.x;
        cmd.twist.linear.y  = correction.y;
        if (!obstacle_segment_clear(cur, cur, obs, ggcfg_)) turn_command_rate_ = 0.0;
        cmd.twist.angular.z = turn_command_rate_;
        last_look_ = turn_target_; look_valid_ = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "原地转身找解中…(已转 %.0f°，朝 %s)", std::fabs(turn_accum_) * 180.0 / M_PI,
            turn_dir_ > 0 ? "逆时针" : "顺时针");
        return true;
    }

    // 探索重规划：plan_explore 给覆盖路径(可能直穿障碍) → 挑一个中段目标 scan_target →
    //   全局 A* 把它改造成绕障(含离墙)折线 → 平滑交 tracker 跟随。覆盖策略 plan_explore 不动；
    //   mark_scan 只看位姿+视野，与走法无关，故改用 A* 绕障轨迹不影响扫描覆盖。
    //   A* 到 scan_target 无解(被围死) → 放弃该区域：跳下一条带、本拍清轨迹悬停、下一拍重规划去别处。
    //   需在持有 mtx_ 时调用（读写 px_/yaw_/cur_band_/traj_ 等）。
    void replan_locked(const Vec2& cur)
    {
        if (!plan_pending_ && !tracker_->has_trajectory() &&
            (now() - last_plan_time_).seconds() < replan_period_) return;
        ++replan_checks_;
        bool all_explored = false;
        const Obstacles obs = obs_map_->snapshot();
        if (!plan_pending_ && have_observation_target_ && explore_has_committed_ &&
            (observation_arrived_ || (tracker_->remaining_distance() <= goal_tol_ &&
             std::hypot(cur.x - explore_target_.x, cur.y - explore_target_.y) <= goal_tol_)) &&
            obstacle_segment_clear(cur, cur, obs, ggcfg_)) {
            last_plan_time_ = now();
            return;  // Finish the observation at this safe viewpoint before selecting another.
        }
        // Keep a safe reference while holding XY and turning. A new obstacle or an
        // explicit mission request still interrupts the recovery immediately.
        if (!plan_pending_ && tracker_->reorienting() && explore_has_committed_ &&
            path_clear(cur, tracker_->remaining_path(cur.x, cur.y), obs, ggcfg_)) {
            last_plan_time_ = now();
            return;
        }

        // 栅格快照：本次规划期间只读一次地图(一把锁+一次拷贝)，
        //   下面的覆盖率判定与 plan_explore 逐格选点都基于【同一时刻】的视图，不会半新半旧。
        const GridSnapshot gsnap = grid_->snapshot();

        // 黑名单按覆盖率台阶清空：每涨过 unreach_clear_step_(默认5%) 给死路区一次重试机会
        //   (飞机已移动、视角已变，之前 A* 够不到的区可能现在进得去)。换终点也会清(见 on_goal)。
        const double cov_now = gsnap.coverage_ratio();
        if (cov_now - last_unreach_clear_cov_ >= unreach_clear_step_) {
            unreachable_.clear();
            last_unreach_clear_cov_ = cov_now;
        }

        const bool have_commit = tracker_->has_trajectory() && explore_has_committed_ && !explore_failed_;
        const Path2 remaining = have_commit ? tracker_->remaining_path(cur.x, cur.y) : Path2{};
        const bool old_clear = have_commit && path_clear(cur, remaining, obs, ggcfg_);
        const double view_heading = have_observation_target_ ? observation_target_.view_heading : yaw_;
        active_target_gain_ = have_commit && have_observation_target_
            ? visible_unknown_count(gsnap, explore_target_, view_heading, obs, fcfg_) : -1;
        const bool target_stale = have_commit && (have_observation_target_
            ? active_target_gain_ == 0
            : (target_stale_r_ > 0.0 && !gsnap.has_gain_within(explore_target_, target_stale_r_)));
        const PathScore old_score = have_commit ? score_path(cur, remaining, obs, ggcfg_) : PathScore{};
        const bool handoff_due = have_commit && old_score.length < global_lookahead_;
        const auto decision_time = now();
        const double candidate_age = candidate_time_valid_
            ? (decision_time - last_candidate_time_).seconds() : candidate_period_;
        if (!plan_pending_ && old_clear && !target_stale && !handoff_due &&
            candidate_age >= 0.0 && candidate_age < candidate_period_) {
            ++candidate_skips_;
            last_plan_time_ = decision_time;
            return;
        }
        last_candidate_time_ = decision_time;
        candidate_time_valid_ = true;

        // Candidate selection may advance bands, but rejected plans must not
        // change the band followed by the still-active route.
        int candidate_band = cur_band_;
        FrontierSelection observation;
        Path2 wp;
        if (frontier_observation_enabled_) {
            double route_heading = std::numeric_limits<double>::quiet_NaN();
            if (!plan_pending_ && old_clear && tracker_->remaining_distance() > goal_tol_) {
                const double ahead_s = tracker_->progress_distance() + frontier_continuity_lookahead_;
                Vec2 ahead = traj_.back().p;
                for (const auto& point : traj_) {
                    if (point.s >= ahead_s) { ahead = point.p; break; }
                }
                if (std::hypot(ahead.x - cur.x, ahead.y - cur.y) > goal_tol_)
                    route_heading = std::atan2(ahead.y - cur.y, ahead.x - cur.x);
            }
            observation = select_observation_target(gsnap, fcfg_, cur, yaw_, candidate_band,
                                                   obs, &unreachable_, unreach_block_r_, route_heading);
            wp.push_back(cur);
            if (observation.valid) wp.push_back(observation.point);
        } else {
            wp = plan_explore(gsnap, fcfg_, cur, yaw_, candidate_band, all_explored,
                              &unreachable_, unreach_block_r_);
        }
        if (wp.size() < 2) {
            // An empty candidate list is not a request to A* back to cur.
            // Retry after a normal planning period as observations update.
            if (!old_clear) {
                traj_.clear();
                tracker_->set_trajectory({});
                explore_has_committed_ = false;
                explore_failed_ = true;
            }
            unreachable_.clear();
            last_replan_reason_ = "no_observation_candidate";
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[探索选点] 暂无新观察点，覆盖率 %.1f%%，%s；下一规划周期重试",
                cov_now * 100.0, old_clear ? "继续当前安全路线" : "等待可用路线");
            plan_pending_ = false;
            last_plan_time_ = now();
            return;
        }

        // 挑覆盖路径上第一个距当前 ≥ explore_target_min_dist_ 的点作 A* 终点：
        //   太近绕行无意义、易频繁过期；只需绕近处障碍，到了自然周期重规划接力。
        Vec2 scan_target = wp.empty() ? cur : wp.back();
        for (size_t i = 1; !frontier_observation_enabled_ && i < wp.size(); ++i) {
            if (std::hypot(wp[i].x - cur.x, wp[i].y - cur.y) >= explore_target_min_dist_) {
                scan_target = wp[i];
                break;
            }
        }

        // ★路径承诺/迟滞★：已有绕障折线时，先判旧折线是否仍无碰撞、目标是否漂移。
        //   仅在【目标大幅移动(换区)】或【旧折线被挡(会撞)】时才考虑重算 A*。
        const bool target_moved = have_commit &&
            std::hypot(scan_target.x - explore_target_.x,
                       scan_target.y - explore_target_.y) > commit_target_tol_;

        // ★目标失效(2026-08)★：承诺目标【邻域内已无任何未扫大格】→ 这条承诺已无信息可拿。
        //   成因：飞向目标的途中，机载视野(FOV 100°/3m)常常早就把那片扫完了；但承诺/迟滞
        //   只看"目标动没动、路撞不撞"，从不问"这个目标还值不值得去"，于是飞机咬着一个
        //   零收益的点一路飞到跟前 —— 正是"明明已覆盖、还在往前走"的直接原因。
        //   失效即【绕过下面两道迟滞闸】立刻改投新目标(见 stale 在两处的短路)。
        //   ★半径口径★：看邻域而非仅目标格自身。仿真(6 场景, 主指标=达 90% 覆盖用时)：
        //     基线 281.0s | 仅看目标格 283.3s(无改善,3 场景更慢) | 邻域 R=1.0m 242.3s(-13.8%,全面更快)
        //     R=2.0m 反而 293.3s —— 半径过大会过早放弃仍有价值的目标。故取 R=1.0m。
        // 目标几乎没动 且 旧折线仍无碰撞 且 目标仍有收益 → 直接续用，连 A* 都不搜(最省、最稳，绝不翻边)。
        if (have_commit && !target_moved && old_clear && !target_stale) {
            plan_pending_ = false;
            last_plan_time_ = now();   // 续命，避免下一拍又因 age 触发重搜
            return;
        }
        GlobalResult gr = search_explore_global(cur, scan_target, obs, yaw_);
        if (gr.ok && gr.path.size() >= 2) {
            // ★换路评估(迟滞第二道闸)★：目标漂移触发了重算，但只要旧折线【仍无碰撞】就别急着换——
            //   给旧路/新路各打分(score_path: 剩余弧长=时间效益, 最小障碍边距=安全性)，新路需
            //   【更安全≥safety_gain】且/或【更快≥time_gain】(require_both 控制"且/或")才换，否则续用旧路。
            //   关键：左右横跳的两条路是 near-tie(几乎等长、等安全)→两项都达不到阈值→不换→根除横跳。
            //   例外：旧折线已被挡(old_clear=false 会撞)、旧路快走完(剩余<前瞻)、
            //         或【旧目标已失效(扫完了)】→ 跳过评估直接采纳(安全/进度优先)。
            //   ★target_stale 必须在这里也短路★：否则失效目标虽被识别，却仍可能因
            //   "新路不够安全/不够快"被这道闸打回、继续咬着零收益目标飞 —— 修了等于没修。
            if (old_clear && !target_stale) {
                const PathScore& so = old_score;   // 当前真正执行轨迹的剩余段
                const PathScore sn = score_path(cur, gr.path,      obs, ggcfg_);   // 新路
                const bool exhausted = so.length < global_lookahead_;             // 旧路快走完→无可咬死，放行换路
                const bool safer  = sn.min_clear >= so.min_clear * (1.0 + path_switch_safety_gain_);
                const bool faster = sn.length    <= so.length    * (1.0 - path_switch_time_gain_);
                const bool worth  = exhausted ||
                    (path_switch_require_both_ ? (safer && faster) : (safer || faster));
                if (!worth) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1500,
                        "换路评估→保持旧路(旧 长%.2f 边距%.2f | 新 长%.2f 边距%.2f; 更安全=%d 更快=%d)",
                        so.length, so.min_clear, sn.length, sn.min_clear, safer ? 1 : 0, faster ? 1 : 0);
                    plan_pending_ = false;
                    last_plan_time_ = now();
                    return;                    // 不够好 → 续用旧轨迹，绝不翻边
                }
            }
            if (adopt_explore_path(gr, scan_target, &observation)) {
                last_replan_reason_ = !have_commit ? "initial_or_recovery" :
                    (!old_clear ? "route_blocked" : (target_stale ? "view_exhausted" :
                    (handoff_due ? "route_end" : "better_route")));
                RCLCPP_INFO(get_logger(), "探索换路[%s] → (%.2f, %.2f)，原目标可见未扫格=%d",
                    last_replan_reason_.c_str(), scan_target.x, scan_target.y, active_target_gain_);
                cur_band_ = candidate_band;
                return;
            }
            if (explore_handoff_deferred_) return;
        }
        if (old_clear && !target_stale) {
            // Failure of a speculative candidate does not invalidate the
            // safe, productive trajectory already being flown.
            last_replan_reason_ = "keep_route_candidate_failed";
            ++kept_after_candidate_failure_;
            plan_pending_ = false;
            last_plan_time_ = now();
            return;
        }
        {
            // ★机头锥内无解★：先用【无锥 probe】判定到底是"任何朝向都没路"还是"有路只是不在机头方向"。
            explore_failed_ = true;
            explore_has_committed_ = false;
            traj_ = Trajectory{};
            tracker_->set_trajectory(traj_);

            GlobalResult probe = search_explore_global(cur, scan_target, obs);
            if (probe.ok && probe.path.size() >= 2) {
                // 有路，只是不在机头方向 → 原地转身找解(改朝向重搜)，不后退/不跳带。
                enter_turn_for_solution(cur, scan_target, probe, candidate_band, &observation);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "机头方向无路但侧向有路 → 原地转身找解 (目标 %.2f,%.2f)", scan_target.x, scan_target.y);
            } else {
                // 任何朝向都没路 → 走原有 retreat(贴脸脱困) / 跳带(真被围死) 逻辑。
                const double od = nearest_obstacle_dist(obs);
                const bool can_retreat = (od < retreat_trigger_) && retreat_available();
                if (can_retreat) {
                    // 贴障且未退够 → 不跳带/不拉黑，交主循环后退脱困，退出去下拍重规划接着绕。
                    plan_pending_ = true;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "探索目标 (%.2f, %.2f) A* 无解且贴障 → 后退脱困重试", scan_target.x, scan_target.y);
                } else {
                    // 不是贴脸卡死/已退到极限仍无路 = 真够不到 → 放弃该区：拉黑 + 跳带 + 红叉 + 去别处。
                    retreating_ = false;
                    unreachable_.push_back(scan_target);   // 拉黑够不到的目标：选点不再回头试这条死路
                    plan_pending_ = false;
                    last_plan_time_ = now();
                    has_unreachable_marker_ = true;        // ★画红叉★
                    unreachable_pos_ = scan_target;
                    RCLCPP_WARN(get_logger(),
                        "探索观察点 (%.2f, %.2f) 暂无路径 → 暂缓此点，下一规划周期重试其他观察点",
                        scan_target.x, scan_target.y);
                }
            }
        }
        last_plan_time_ = now();
    }

    // PD 核心：朝 heading_pt 方向前进，速度/刹停按 dist_for_speed(到【最终目标】距离)。
    //   方向取自 heading_pt(可为绕障轨迹上的 carrot 前瞻点)；速度只看到真目标的距离，
    //   逼近时距离项趋零 + D 项吃惯性 → 自然刹停；机头没对正时收住前进(乘 cos)。
    //   配套 yaw_rate 存到 last_yaw_rate_。
    Vec2 pd_core(const Vec2& heading_pt, double dist_for_speed)
    {
        const double ex = heading_pt.x - px_;
        const double ey = heading_pt.y - py_;
        const double dir = std::hypot(ex, ey);

        const double v_now = std::hypot(v_fwd_est_, v_lat_est_);
        double v_des = kp_goal_ * dist_for_speed - kd_goal_ * v_now;
        v_des = std::clamp(v_des, 0.0, v_goal_max_);

        double yaw_rate = 0.0, e_yaw = 0.0;
        if (dist_for_speed > goal_tol_ && dir > 1e-6) {
            const double yaw_des = std::atan2(ey, ex);
            e_yaw = yaw_des - yaw_;
            while (e_yaw >  M_PI) e_yaw -= 2.0 * M_PI;
            while (e_yaw <= -M_PI) e_yaw += 2.0 * M_PI;
            const double predicted_error = e_yaw - (carlike_mode_ ? gains_.prediction_time * yaw_rate_est_ : 0.0);
            const double damping = carlike_mode_ ? gains_.kd_yaw * yaw_rate_est_ : 0.0;
            yaw_rate = std::clamp(gains_.kp_yaw * predicted_error - damping,
                                  -gains_.max_yaw_rate, gains_.max_yaw_rate);
        }

        Vec2 vb{0.0, 0.0};
        if (dir > 1e-6) {
            const double ux = ex / dir, uy = ey / dir;
            const double c = std::cos(yaw_), s = std::sin(yaw_);
            const double fwd =  c * ux + s * uy;
            const double lat = -s * ux + c * uy;
            // ★先转再走·朝向门控★：机头偏离目标方向越大,越压住移动(前进+横向都乘),先转够再走。
            //   |e_yaw| > 阈值 → 门=0,本拍只转身不移动(防身后/大角度目标时机体平移甩出去撞柱);
            //   阈值内 cos(e_yaw) 平滑过渡。光压前进不压横向→飞机仍侧移甩出去,故横向也乘。
            double head_gate = (std::fabs(e_yaw) > gains_.heading_gate_rad)
                ? 0.0 : std::max(0.0, std::cos(e_yaw));
            if (carlike_mode_) {
                const double ratio = std::clamp((std::fabs(e_yaw) - gains_.heading_gate_rad) /
                    std::max(1e-3, gains_.stop_align_rad - gains_.heading_gate_rad), 0.0, 1.0);
                head_gate = 1.0 - ratio * ratio * (3.0 - 2.0 * ratio);
                const double turning_rate = std::max(std::fabs(yaw_rate), std::fabs(yaw_rate_est_));
                if (turning_rate > 1e-3) v_des = std::min(v_des, gains_.max_lateral_accel / turning_rate);
            }
            vb.x = v_des * fwd * head_gate;
            vb.y = std::clamp(v_des * lat, -gains_.max_v_lat, gains_.max_v_lat) * head_gate;
        }
        last_yaw_rate_ = yaw_rate;
        return vb;
    }

    // 算法侧 PD 直奔某点(机体系)：机头转向目标、平滑到点。归航与去插点共用(无障碍/关全局绕障时)。
    Vec2 pd_to_point(const Vec2& target)
    {
        return pd_core(target, std::hypot(target.x - px_, target.y - py_));
    }

    void hold_required_path(const std::string& reason)
    {
        required_goal_holding_ = false;
        global_failed_ = true;
        global_has_ = global_at_goal_ = global_braking_blocked_ = false;
        command_projection_blocked_ = measured_projection_blocked_ = false;
        required_blocked_time_valid_ = false;
        global_raw_.clear();
        global_traj_.clear();
        global_tracker_->set_trajectory({});
        global_block_reason_ = reason;
        traj_.clear();
        last_look_ = {px_, py_};
        look_valid_ = true;
        last_yaw_rate_ = 0.0;
        retreating_ = false;
    }

    GlobalConfig required_config() const
    {
        GlobalConfig cfg = ggcfg_;
        if (homing_) cfg.required_goal_field_margin = home_goal_wall_margin_;
        return cfg;
    }

    bool required_motion_clear(const Vec2& stop, const Obstacles& obs) const
    {
        const Vec2 cur{px_, py_};
        GlobalConfig motion_config = required_config();
        motion_config.inflate = required_motion_inflate_;
        const double field_margin = required_field_margin(motion_config);
        const auto allowed_field = [this, field_margin](const Vec2& p) {
            return p.x >= ggcfg_.min_x + field_margin &&
                   p.x <= ggcfg_.max_x - field_margin &&
                   p.y >= ggcfg_.min_y + field_margin &&
                   p.y <= ggcfg_.max_y - field_margin;
        };
        if (!obstacle_segment_clear(cur, stop, obs, motion_config)) return false;
        const double low[2]{ggcfg_.min_x + ggcfg_.wall_margin, ggcfg_.min_y + ggcfg_.wall_margin};
        const double high[2]{ggcfg_.max_x - ggcfg_.wall_margin, ggcfg_.max_y - ggcfg_.wall_margin};
        const double a[2]{cur.x, cur.y};
        const auto monotonic_entry = [&](const Vec2& end) {
            const double b[2]{end.x, end.y};
            for (size_t axis = 0; axis < 2; ++axis) {
                if (a[axis] < low[axis]) {
                    if (b[axis] < a[axis] || b[axis] > high[axis]) return false;
                } else if (a[axis] > high[axis]) {
                    if (b[axis] > a[axis] || b[axis] < low[axis]) return false;
                } else if (b[axis] < low[axis] || b[axis] > high[axis]) return false;
            }
            return true;
        };
        // Match the planner's initial field-entry policy: an aircraft taking
        // off on a task boundary may move inward before attaining the normal
        // body/wall inset. Otherwise measured speed repeatedly trips this guard
        // and clamps visual/required-point takeoff motion to the stop threshold.
        // Physical obstacle clearance above still applies to the whole segment.
        if (monotonic_entry(stop)) return true;
        if (!allowed_field(cur) || !allowed_field(stop)) return false;

        // Only the approved straight terminal connector can leave the full
        // field inset. Its execution capsule uses the existing arrival tolerance.
        const auto in_connector = [this](const Vec2& p) {
            const double dx = global_target_.x - global_connector_start_.x;
            const double dy = global_target_.y - global_connector_start_.y;
            const double length2 = dx * dx + dy * dy;
            const double u = length2 > 1e-12
                ? std::clamp(((p.x - global_connector_start_.x) * dx +
                              (p.y - global_connector_start_.y) * dy) / length2, 0.0, 1.0)
                : 0.0;
            return std::hypot(p.x - global_connector_start_.x - u * dx,
                              p.y - global_connector_start_.y - u * dy) <= goal_tol_;
        };
        if (in_connector(cur) && in_connector(stop)) return true;

        // A single stopping segment can span the ordinary inset and the
        // approved terminal connector. Check its exact exit from the inset;
        // requiring the aircraft to be in the terminal capsule already caused
        // full-speed/zero-speed oscillation on short fields.
        const double delta[2]{stop.x - cur.x, stop.y - cur.y};
        double enter = 0.0, leave = 1.0;
        for (size_t axis = 0; axis < 2; ++axis) {
            if (std::abs(delta[axis]) < 1e-12) {
                if (a[axis] < low[axis] || a[axis] > high[axis]) return false;
                continue;
            }
            double first = (low[axis] - a[axis]) / delta[axis];
            double last = (high[axis] - a[axis]) / delta[axis];
            if (first > last) std::swap(first, last);
            enter = std::max(enter, first);
            leave = std::min(leave, last);
        }
        if (enter > leave || leave < 0.0 || leave > 1.0) return false;
        const Vec2 exit{std::clamp(cur.x + leave * delta[0], low[0], high[0]),
                        std::clamp(cur.y + leave * delta[1], low[1], high[1])};
        return monotonic_entry(exit) && in_connector(exit) && in_connector(stop);
    }

    bool required_velocity_clear(const Vec2& world_velocity, const Obstacles& obs) const
    {
        const double speed = std::hypot(world_velocity.x, world_velocity.y);
        if (!std::isfinite(speed)) return false;
        // This is a short command safety projection, not a replacement for
        // the validated A* corridor.  Using the full prediction horizon here
        // made a valid required path look blocked near corners and latched
        // the controller at zero speed.  Keep a bounded look-ahead and let
        // required_path_clear() provide the long-range obstacle guarantee.
        const double horizon = std::min(0.35, std::max(0.0, global_gains_.prediction_time)) +
            speed / (2.0 * std::max(1e-3, global_gains_.max_accel));
        return required_motion_clear({px_ + world_velocity.x * horizon,
                                      py_ + world_velocity.y * horizon}, obs);
    }

    // Required points keep an exact, independently validated terminal segment.
    // Position errors affect speed only; heading always follows the checked path.
    Vec2 pd_to_point_avoid(const Vec2& target, const Obstacles& obs)
    {
        const Vec2 cur{px_, py_};
        const GlobalConfig required_cfg = required_config();
        const double field_margin = required_field_margin(required_cfg);
        global_goal_blocked_ = !path_clear(target, Path2{target, target}, obs, ggcfg_) ||
            target.x < ggcfg_.min_x + field_margin ||
            target.x > ggcfg_.max_x - field_margin ||
            target.y < ggcfg_.min_y + field_margin ||
            target.y > ggcfg_.max_y - field_margin;
        if (global_goal_blocked_) {
            global_target_ = target;
            hold_required_path("required_goal_blocked");
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Required goal (%.2f, %.2f) is occupied or outside the safe field; holding",
                target.x, target.y);
            return {};
        }
        const bool target_moved = std::hypot(target.x - global_target_.x, target.y - global_target_.y) > 1e-6;
        if (target_moved) required_goal_holding_ = false;
        VelCmd command;
        bool updated_existing = false;
        if (global_has_ && !target_moved && global_tracker_->has_trajectory()) {
            command = global_tracker_->update(px_, py_, yaw_, v_fwd_est_, v_lat_est_, goal_tol_, yaw_rate_est_);
            updated_existing = true;
            if (command.needs_replan) global_has_ = false;
            Path2 remaining = global_tracker_->remaining_reference_path();
            if (remaining.size() == 1) remaining.push_back(target);
            if (!required_path_clear(cur, remaining, target, obs, required_cfg)) global_has_ = false;
        }
        if (!global_has_ || target_moved) {
            required_goal_holding_ = false;
            const double age = (now() - last_global_plan_time_).seconds();
            if (global_failed_ && !target_moved && age >= 0.0 && age < replan_period_) {
                hold_required_path("required_path_unavailable");
                return {};
            }
            global_target_ = target;
            last_global_plan_time_ = now();
            ++astar_searches_;
            ++global_path_searches_;
            GlobalResult gr = plan_required_path(cur, target, obs, required_cfg, yaw_);
            if (!gr.ok || gr.path.size() < 2) {
                ++astar_searches_;
                ++global_path_searches_;
                gr = plan_required_path(cur, target, obs, required_cfg);
            }
            if (!gr.ok || gr.path.size() < 2 ||
                !required_path_clear(cur, gr.path, target, obs, required_cfg)) {
                hold_required_path("required_path_unavailable");
                return {};
            }
            global_connector_start_ = gr.path[gr.path.size() - 2];
            Path2 prefix(gr.path.begin(), gr.path.end() - 1);
            Trajectory candidate = smooth_catmull_rom(prefix, arc_ds_);
            const double tail_length = std::hypot(target.x - global_connector_start_.x,
                                                  target.y - global_connector_start_.y);
            TrajPoint terminal{};
            terminal.p = target;
            terminal.theta = std::atan2(target.y - global_connector_start_.y,
                                        target.x - global_connector_start_.x);
            terminal.s = candidate.back().s + tail_length;
            if (candidate.size() == 1) candidate.back().theta = terminal.theta;
            candidate.push_back(terminal);
            Path2 executed;
            for (const auto& point : candidate) executed.push_back(point.p);
            if (!required_path_clear(cur, executed, target, obs, required_cfg)) {
                ++global_smooth_fallbacks_;
                candidate = make_polyline_trajectory(gr.path);
                if (candidate.size() == 1) candidate.push_back(candidate.front());
                executed.clear();
                for (const auto& point : candidate) executed.push_back(point.p);
            }
            if (!required_path_clear(cur, executed, target, obs, required_cfg)) {
                ++global_invalid_paths_;
                hold_required_path("required_trajectory_invalid");
                return {};
            }
            global_raw_ = executed;
            global_traj_ = candidate;
            global_tracker_->set_trajectory(global_traj_);
            global_failed_ = false;
            global_has_ = true;
            required_blocked_time_valid_ = false;
            if (updated_existing) {
                // A route invalidated this tick must brake before using its
                // replacement; do not advance the tracker twice in one period.
                command.v_fwd = command.v_lat = 0.0;
                command.at_goal = false;
                global_tracker_->constrain_forward_command(0.0);
            } else {
                command = global_tracker_->update(px_, py_, yaw_, v_fwd_est_, v_lat_est_, goal_tol_, yaw_rate_est_);
            }
        }
        traj_ = global_traj_;
        last_look_ = global_tracker_->last_lookahead();
        look_valid_ = true;
        global_at_goal_ = command.at_goal;
        const double remaining = global_tracker_->remaining_distance();
        if (command.at_goal || required_goal_holding_) {
            if (!required_goal_holding_) {
                required_goal_hold_.begin(target);
                required_goal_yaw_ = yaw_;
                required_goal_holding_ = true;
            }
            const auto correction = required_goal_hold_.update(cur, yaw_, v_fwd_est_, v_lat_est_,
                kp_goal_, kd_goal_, std::min(0.20, v_goal_max_), gains_.max_accel, gains_.dt);
            command.v_fwd = correction.x;
            command.v_lat = correction.y;
            command.yaw_rate = std::clamp(gains_.kp_yaw * wrap_pi(required_goal_yaw_ - yaw_) -
                gains_.kd_yaw * yaw_rate_est_, -gains_.max_yaw_rate, gains_.max_yaw_rate);
            command.holding_position = true;
            global_at_goal_ = std::hypot(target.x - px_, target.y - py_) <= goal_tol_;
        }
        const double speed = std::hypot(command.v_fwd, command.v_lat);
        if (speed > 1e-9 && !command.holding_position) {
            // Apply the distance/damping envelope continuously. Switching it
            // on only at endpoint_slow_r introduced a discrete speed drop at
            // that radius even on a straight, completely clear home route.
            const double toward_speed = std::max(0.0,
                (v_fwd_est_ * command.v_fwd + v_lat_est_ * command.v_lat) / speed);
            const double cap = std::max(0.0, kp_goal_ * remaining - kd_goal_ * toward_speed);
            const double scale = std::min(1.0, cap / speed);
            command.v_fwd *= scale;
            command.v_lat *= scale;
            global_tracker_->constrain_forward_command(command.v_fwd);
        }
        return guard_required_command(command, obs, vision_queue_ && vision_queue_->active());
    }

    Vec2 guard_required_command(VelCmd command, const Obstacles& obs, bool checked_visual_path)
    {
        const Vec2 cur{px_, py_};
        const double c = std::cos(yaw_), s = std::sin(yaw_);
        const Vec2 desired{c * command.v_fwd - s * command.v_lat,
                           s * command.v_fwd + c * command.v_lat};
        const Vec2 measured{c * v_fwd_est_ - s * v_lat_est_, s * v_fwd_est_ + c * v_lat_est_};
        const double measured_speed = std::hypot(measured.x, measured.y);
        // Visual targets already follow the independently validated required
        // A* trajectory.  The generic long stopping projection can intersect
        // a nearby obstacle even when the checked path is clear, causing an
        // endless replan/zero-speed loop.  Keep measured-velocity protection
        // below, but do not reject the path-following command itself here.
        // XY hold corrections are not along the checked visual route; validate
        // their own direction even when that route is still clear.
        command_projection_blocked_ = (!checked_visual_path || command.holding_position) &&
            !required_velocity_clear(desired, obs);
        if (command_projection_blocked_ && required_velocity_clear({0.0, 0.0}, obs)) {
            // A stopping projection beyond the permitted corridor may still
            // admit a slower command. Approach that limit continuously instead
            // of repeatedly dropping the entire forward command to zero.
            // Every candidate uses the same obstacle and field checks.
            double safe = 0.0, unsafe = 1.0;
            for (int i = 0; i < 16; ++i) {
                const double scale = .5 * (safe + unsafe);
                if (required_velocity_clear({scale * desired.x, scale * desired.y}, obs)) safe = scale;
                else unsafe = scale;
            }
            if (safe * std::hypot(desired.x, desired.y) > 1e-5) {
                command.v_fwd *= safe;
                command.v_lat *= safe;
                global_tracker_->constrain_forward_command(command.v_fwd);
                command_projection_blocked_ = false;
            }
        }
        measured_projection_blocked_ = (!std::isfinite(measured_speed) || measured_speed > goal_stop_v_) &&
            !required_velocity_clear(measured, obs);
        const bool blocked = command_projection_blocked_ || measured_projection_blocked_;
        if (blocked) {
            if (!global_braking_blocked_) {
                ++global_velocity_brakes_;
                required_brake_speed_ = 0.0;
            }
            command.v_fwd = command.v_lat = 0.0;
            global_tracker_->constrain_forward_command(0.0);
            // A safe counter-velocity actively brakes drift. Zero velocity alone
            // left the aircraft drifting and prevented this guard from releasing.
            if (std::isfinite(measured_speed) && measured_speed > 1e-9) {
                const double cap = std::min({0.20, v_goal_max_, kd_goal_ * measured_speed,
                    required_brake_speed_ + global_gains_.max_accel * global_gains_.dt});
                const Vec2 braking{-measured.x * cap / measured_speed, -measured.y * cap / measured_speed};
                if (required_velocity_clear(braking, obs)) {
                    command.v_fwd = c * braking.x + s * braking.y;
                    command.v_lat = -s * braking.x + c * braking.y;
                    required_brake_speed_ = cap;
                } else {
                    required_brake_speed_ = 0.0;
                }
            }
            if (!required_motion_clear(cur, obs)) command.yaw_rate = 0.0;
        }
        global_braking_blocked_ = blocked;
        global_block_reason_ = blocked ? "required_stopping_projection" : "none";
        if (blocked) {
            const double current_time = steady_seconds();
            if (!required_blocked_time_valid_) {
                required_blocked_since_ = current_time;
                required_blocked_time_valid_ = true;
            }
            if (required_blocked_replan_s_ > 0.0 &&
                current_time - required_blocked_since_ >= required_blocked_replan_s_) {
                ++required_blocked_replans_;
                global_has_ = false;
                required_blocked_time_valid_ = false;
                global_block_reason_ = "required_blocked_replan";
            }
        } else {
            required_blocked_time_valid_ = false;
        }
        last_yaw_rate_ = command.yaw_rate;
        return {command.v_fwd, command.v_lat};
    }

    void reset_visual_approach()
    {
        vision_straight_track_ = 0;
        vision_straight_velocity_ = {};
    }

    Vec2 visual_target_velocity(const VisionVisitTarget& target, const Obstacles& obs)
    {
        const Vec2 cur{px_, py_};
        const double distance = std::hypot(target.position.x - px_, target.position.y - py_);
        const bool latched = vision_straight_track_ == target.track_id;
        const bool eligible = !homing_ && vision_straight_approach_m_ > 0.0 &&
            (latched || distance <= vision_straight_approach_m_);
        const Path2 direct{cur, target.position};
        if (!eligible || !required_path_clear(cur, direct, target.position, obs, required_config())) {
            if (vision_straight_track_ != 0) {
                reset_visual_approach();
                global_has_ = global_failed_ = false;
                global_tracker_->set_trajectory({});
            }
            return pd_to_point_avoid(target.position, obs);
        }
        const double c = std::cos(yaw_), s = std::sin(yaw_);
        const Vec2 measured{c * v_fwd_est_ - s * v_lat_est_, s * v_fwd_est_ + c * v_lat_est_};
        if (!latched) {
            vision_straight_track_ = target.track_id;
            vision_straight_velocity_ = measured;
            global_tracker_->set_trajectory({});
            required_blocked_time_valid_ = false;
            RCLCPP_INFO(get_logger(), "[视觉目标] 区域#%llu 距离 %.2fm，停止主动转头，切换XY直线接近",
                static_cast<unsigned long long>(target.track_id), distance);
        }
        // World-frame position PD removes the heading gate and permits lateral
        // or backward correction. Never chase the goal bearing with yaw here.
        Vec2 desired{kp_goal_ * (target.position.x - px_) - kd_goal_ * measured.x,
                     kp_goal_ * (target.position.y - py_) - kd_goal_ * measured.y};
        const double cap = std::min(v_goal_max_, vision_near_->speed_cap(gains_.v_max));
        const auto limit = [cap](Vec2& v) {
            const double speed = std::hypot(v.x, v.y);
            if (speed > cap && speed > 1e-9) { v.x *= cap / speed; v.y *= cap / speed; }
        };
        limit(desired);
        const Vec2 change{desired.x - vision_straight_velocity_.x, desired.y - vision_straight_velocity_.y};
        const double step = std::hypot(change.x, change.y);
        const double scale = step > 1e-9 ? std::min(1.0, gains_.max_accel * gains_.dt / step) : 1.0;
        desired = {vision_straight_velocity_.x + scale * change.x,
                   vision_straight_velocity_.y + scale * change.y};
        limit(desired);
        global_target_ = target.position;
        global_connector_start_ = cur;
        global_raw_ = direct;
        global_traj_ = make_polyline_trajectory(direct);
        traj_ = global_traj_;
        last_look_ = target.position;
        look_valid_ = true;
        global_has_ = true;
        global_failed_ = global_goal_blocked_ = false;
        global_at_goal_ = distance <= vision_target_tol_;
        VelCmd command;
        command.v_fwd = c * desired.x + s * desired.y;
        command.v_lat = -s * desired.x + c * desired.y;
        command.yaw_rate = 0.0;
        const Vec2 result = guard_required_command(command, obs, false);
        vision_straight_velocity_ = {c * result.x - s * result.y, s * result.x + c * result.y};
        return result;
    }

    // 飞机到最近障碍【边缘】的距离(圆心距 − 障碍半径)。无障碍返回很大值。
    double nearest_obstacle_dist(const Obstacles& obs) const
    {
        double best = 1e9;
        for (const auto& o : obs) {
            const double d = std::hypot(o.cx - px_, o.cy - py_) - o.r;
            best = std::min(best, d);
        }
        return best;
    }

    // ★脱困后退★：A* 无解且飞机贴着障碍时调用。朝【远离最近障碍】方向取 retreat_step_ 远的点，
    //   低速 PD 退过去(限 retreat_v_max_)，边退边在主循环重算 A*——退出去通道打开就自然接着绕。
    //   返回机体系速度；写 traj_(后退直线，可视化)+last_yaw_rate_。首次进入记 retreat_origin_。
    Vec2 do_retreat(const Obstacles& obs)
    {
        // 取最近障碍，后退方向 = 障碍→飞机(把飞机往外推)
        double bd = 1e9; Vec2 oc{px_, py_};
        for (const auto& o : obs) {
            const double d = std::hypot(o.cx - px_, o.cy - py_) - o.r;
            if (d < bd) { bd = d; oc = {o.cx, o.cy}; }
        }
        double ux = px_ - oc.x, uy = py_ - oc.y;
        const double n = std::hypot(ux, uy);
        if (n < 1e-6) { ux = std::cos(yaw_ + M_PI); uy = std::sin(yaw_ + M_PI); }  // 退化:贴圆心,朝身后退
        else          { ux /= n; uy /= n; }

        // ★避墙★：纯"远离障碍"方向常常正好朝墙(柱子在墙边时)，会把飞机往墙里推→更贴墙→更无解。
        //   若沿此方向退一步会进墙禁入区/出界，就把【朝墙的分量】翻成朝场内，使后退转为沿墙切向退。
        const double wm = ggcfg_.wall_margin;
        {
            double tx = px_ + ux * retreat_step_, ty = py_ + uy * retreat_step_;
            if (tx > ggcfg_.max_x - wm && ux > 0) ux = -std::fabs(ux);   // 太靠东墙且在朝东退 → 翻向西
            if (tx < ggcfg_.min_x + wm && ux < 0) ux =  std::fabs(ux);   // 太靠西墙 → 翻向东
            if (ty > ggcfg_.max_y - wm && uy > 0) uy = -std::fabs(uy);   // 太靠北墙 → 翻向南
            if (ty < ggcfg_.min_y + wm && uy < 0) uy =  std::fabs(uy);   // 太靠南墙 → 翻向北
            const double nn = std::hypot(ux, uy);
            if (nn < 1e-6) {                 // 翻完合成为零(两墙夹角)：沿当前墙切向朝场内
                const double cxw = 0.5 * (ggcfg_.min_x + ggcfg_.max_x);
                const double cyw = 0.5 * (ggcfg_.min_y + ggcfg_.max_y);
                ux = cxw - px_; uy = cyw - py_;                          // 直接朝场地中心退
                const double n2 = std::hypot(ux, uy);
                if (n2 < 1e-6) { ux = -1.0; uy = 0.0; } else { ux /= n2; uy /= n2; }
            } else { ux /= nn; uy /= nn; }
        }

        if (!retreating_) { retreating_ = true; retreat_origin_ = {px_, py_}; retreat_start_ = now(); }

        const Vec2 back{ px_ + ux * retreat_step_, py_ + uy * retreat_step_ };
        traj_ = smooth_catmull_rom(Path2{ {px_, py_}, back }, arc_ds_);   // 可视化:后退直线
        last_look_ = back; look_valid_ = true;

        // ★不走 pd_core(2026-08 修死锁)★
        //   旧实现 pd_core(back, step) 有致命缺陷：后退点在飞机【身后】，e_yaw≈180° →
        //   朝向门控 head_gate=0 → 速度恒为 0，只发 yaw_rate 让飞机原地掉头 180°。
        //   实测要转 72 拍(1.44s)速度才解禁；而这期间飞机没动 → traveled 恒为 0 →
        //   try_retreat 的"已退 0.00m"永远成立 → 无限重试；若期间障碍圆微抖使 back 方向
        //   改变、或 replan_locked 复位 retreat_origin_，则永远转不出去 → 彻底卡死
        //   (日志表现：反复刷"A* 无解且贴障 → 后退脱困(已退 0.00m)"，飞机悬在原地不动)。
        //   ★后退本来就不该先掉头★：飞机是全向的，直接把"世界系后退方向"投影到机体系
        //   即可，机头保持不动(也不该动——机头还朝着障碍/未扫区，转走反而丢失视野)。
        const double c = std::cos(yaw_), s = std::sin(yaw_);
        Vec2 vb{ (  c * ux + s * uy) * retreat_v_max_,     // 机体前向分量
                 ( -s * ux + c * uy) * retreat_v_max_ };   // 机体横向分量
        last_yaw_rate_ = 0.0;                              // 后退期间不转头，保持视野朝向
        // 合速度限幅（分量已按 retreat_v_max_ 缩放，这里只防数值溢出）
        const double sp = std::hypot(vb.x, vb.y);
        if (sp > retreat_v_max_ && sp > 1e-6) { const double k = retreat_v_max_ / sp; vb.x *= k; vb.y *= k; }
        if (!obstacle_segment_clear({px_, py_}, back, obs, ggcfg_) ||
            !field_motion_clear({px_, py_}, back, ggcfg_)) {
            // Moving away from the closest obstacle is not enough: another
            // obstacle or wall can occupy the proposed retreat segment.
            traj_.clear();
            last_look_ = {px_, py_};
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[后退脱困] 退路受障碍/边界限制，停止后退；本轮超时后改选观察点");
            return {};
        }
        return vb;
    }

    bool retreat_available()
    {
        const Vec2 cur{px_, py_};
        if (retreat_exhausted_) {
            if (std::hypot(cur.x - retreat_exhausted_at_.x, cur.y - retreat_exhausted_at_.y) <
                std::max(.10, goal_tol_)) return false;
            retreat_exhausted_ = false;
        }
        if (!retreating_) return true;
        const double traveled = std::hypot(px_ - retreat_origin_.x, py_ - retreat_origin_.y);
        const double elapsed = (now() - retreat_start_).seconds();
        if (traveled < retreat_max_dist_ &&
            !(retreat_timeout_ > 0.0 && elapsed >= retreat_timeout_)) return true;
        // Preserve exhaustion across planning/timer calls. Clearing only the
        // running flag allowed the next tick to start the same failed attempt.
        retreat_exhausted_ = true;
        retreat_exhausted_at_ = cur;
        retreating_ = false;
        RCLCPP_WARN(get_logger(),
            "后退脱困结束(%.1fs，已退%.2fm)，暂停原地重复尝试，改选观察点；位置改变或有效新路线后可重试",
            elapsed, traveled);
        return false;
    }

    // 统一处理"A* 到 target 无解"：贴障且未退够 → 后退脱困(返回 true=本拍已写 cmd 后退速度)；
    //   否则(不是贴脸卡死/已退到极限仍无路=真被围死) → 返回 false 交调用方走原放弃/悬停逻辑。
    //   need_obstacle_near：是否要求"贴障才退"。探索/POI/归航都传 true(只救贴脸卡死，真围死不乱退)。
    bool try_retreat(const Obstacles& obs, geometry_msgs::msg::TwistStamped& cmd)
    {
        if (!retreat_available()) return false;
        const double od = nearest_obstacle_dist(obs);
        const double traveled = retreating_
            ? std::hypot(px_ - retreat_origin_.x, py_ - retreat_origin_.y) : 0.0;
        const double elapsed = retreating_ ? (now() - retreat_start_).seconds() : 0.0;
        if (od < retreat_trigger_) {
            const Vec2 vb = do_retreat(obs);
            cmd.twist.linear.x = vb.x;
            cmd.twist.linear.y = vb.y;
            cmd.twist.angular.z = last_yaw_rate_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "A* 无解且贴障(边距 %.2fm) → 后退脱困(已退 %.2fm, %.1fs)", od, traveled, elapsed);
            return true;
        }
        retreating_ = false;   // 已不贴障，交正常规划逻辑。
        return false;
    }

    GlobalResult search_global(const Vec2& start, const Vec2& goal, const Obstacles& obstacles,
                               const GlobalConfig& config, double yaw = NAN)
    {
        ++astar_searches_;
        return plan_global_path(start, goal, obstacles, config, yaw);
    }

    GlobalResult search_explore_global(const Vec2& start, const Vec2& goal,
                                       const Obstacles& obstacles, double yaw = NAN)
    {
        GlobalConfig config = ggcfg_;
        config.validate_complete_path = true;
        if (explore_plan_reserve_ > 0.0) {
            config.inflate += explore_plan_reserve_;
            auto reserved = search_global(start, goal, obstacles, config, yaw);
            if (reserved.ok) return reserved;
            config.inflate = ggcfg_.inflate;
        }
        return search_global(start, goal, obstacles, config, yaw);
    }

    // ---------- 20Hz 主循环 ----------
    void activate_visual_target(double current_time)
    {
        if (vision_queue_->active()) return;
        reset_visual_approach();
        if (!vision_queue_->activate_next(current_time)) { vision_near_->reset(); return; }
        const auto& active = *vision_queue_->active();
        // A visual target takes ownership from exploration.  Drop every
        // cached required-path state so the first tick cannot reuse an old
        // exploration trajectory or its failure/braking latch.
        global_has_ = false;
        global_failed_ = false;
        global_goal_blocked_ = false;
        global_braking_blocked_ = false;
        required_blocked_time_valid_ = false;
        global_tracker_->set_trajectory({});
        global_raw_.clear();
        global_traj_.clear();
        vision_near_->begin(active, current_time, std::max(active.confirmed_stamp, vision_last_frame_.stamp));
        vision_near_->position({px_, py_}, current_time);
        vision_last_event_ = "active_target_selected";
        vision_hover_track_ = 0;
        vision_hover_since_ = -1.0;
        RCLCPP_INFO(get_logger(), "[视觉队列] 激活区域#%llu 粗坐标(%.3f, %.3f)，待处理%zu（已接入目标飞行）",
            static_cast<unsigned long long>(active.track_id), active.position.x, active.position.y,
            vision_queue_->pending().size());
    }

    void poll_vision_shm()
    {
        vision_new_frame_.reset();
        vision_new_confirmations_.clear();
        const double current_time = steady_seconds();
        for (; vision_applied_visits_ < vision_queue_->visited().size(); ++vision_applied_visits_) {
            const auto& visited = vision_queue_->visited()[vision_applied_visits_];
            vision_candidates_->discard_visited(visited.track_id, visited.position, vision_queue_->blacklist_radius());
        }
        vision_candidates_->expire(current_time);
        // Keep collecting frames during takeoff.  A target seen before the
        // first fresh odometry frame must survive until navigation is ready;
        // the aircraft position is only required when selecting/approaching
        // the target, not when accumulating its far observations.
        vision_collection_active_ = vision_reader_ && has_pose_ && has_goal_ && !finished_ &&
            !homing_ && !corridor_->active() && pose_freshness_.fresh(current_time, vision_pose_timeout_);
        if (!vision_collection_active_) { vision_candidates_->interrupt(); vision_near_->interrupt(); }
        if (!vision_reader_) return;
        vision_new_frame_ = vision_reader_->poll(current_time);
        if (vision_new_frame_) {
            vision_last_frame_ = *vision_new_frame_;
            if (vision_reader_->statistics().frames == 1 || vision_new_frame_->restarted)
                RCLCPP_INFO(get_logger(), "[视觉SHM] %s seq=%llu targets=%zu epoch=%llu（seq是图像帧编号，id是类别）",
                    vision_new_frame_->restarted ? "写端重启后恢复" : "收到首个有效新帧",
                    static_cast<unsigned long long>(vision_new_frame_->seq), vision_new_frame_->count,
                    static_cast<unsigned long long>(vision_new_frame_->epoch));
            if (vision_collection_active_) {
                const auto filtered = vision_queue_->filter_blacklisted(*vision_new_frame_);
                vision_queue_->observe(filtered, current_time);
                vision_new_confirmations_ = vision_candidates_->update(filtered, {px_, py_}, current_time);
                for (const auto& candidate : vision_new_confirmations_)
                    RCLCPP_INFO(get_logger(), "[视觉确认] 区域#%llu 连续%d帧，粗坐标(%.3f, %.3f)，类别暂不判定（本阶段不改变飞行目标）",
                        static_cast<unsigned long long>(candidate.track_id), candidate.consecutive_frames,
                        candidate.position.x, candidate.position.y);
                // Retry previously capacity-limited confirmations too. The queue
                // retains admitted track identities, so snapshots do not duplicate entries.
                vision_queue_->enqueue(vision_candidates_->snapshot(), {px_, py_});
                activate_visual_target(current_time);
                if (has_pose_ && vision_queue_->active()) {
                    const auto uid = vision_queue_->active()->track_id;
                    const auto refined = vision_near_->update(filtered, {px_, py_}, current_time,
                                                              vision_queue_->other_positions(uid));
                    if (refined) {
                        if (vision_queue_->refine_active(*refined)) {
                            ++vision_near_confirmations_;
                            vision_last_event_ = "near_confirmed";
                    RCLCPP_INFO(get_logger(), "[视觉近距] 区域#%llu %d帧投票确认ID=%d，精确坐标(%.3f, %.3f)，继续前往目标",
                                static_cast<unsigned long long>(uid), refined->samples, refined->class_id,
                                refined->position.x, refined->position.y);
                        } else {
                            vision_queue_->defer_active(uid, current_time);
                            vision_near_->reset();
                            vision_last_event_ = "invalid_refinement_deferred";
                        }
                    }
                }
            }
        }
        if (vision_collection_active_) {
            if (has_pose_) activate_visual_target(current_time);
            if (has_pose_) vision_near_->position({px_, py_}, current_time);
            if (vision_queue_->active() && vision_near_->timed_out(current_time)) {
                const auto uid = vision_queue_->active()->track_id;
                vision_queue_->defer_active(uid, current_time);
                vision_near_->reset();
                vision_last_event_ = "near_timeout_deferred";
                RCLCPP_WARN(get_logger(), "[视觉近距] 区域#%llu 超时未确认，暂缓并处理其他目标；不记为完成",
                    static_cast<unsigned long long>(uid));
                activate_visual_target(current_time);
            }
        }
    }

    void on_timer()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = now();
        cmd.header.frame_id = "base_link";   // 机体系

        bool publish_finished = false;
        std_msgs::msg::Header completed_goal;
        bool corridor_active = false;
        bool publish_diagnostics = false;
        std_msgs::msg::Float64 coverage;
        nav_msgs::msg::Path active_path;
        diagnostic_msgs::msg::DiagnosticArray diagnostics;

        {
            std::lock_guard<std::mutex> lk(mtx_);

            // Stages 3/4: data-only queue and near confirmation. These targets
            // still do not change the navigation goal or flight commands.
            poll_vision_shm();

            if (!has_pose_) {
                // 无位姿：发零速度占位
                cmd_pub_->publish(cmd);
                return;
            }

            if (!corridor_->active() && !pose_freshness_.fresh(steady_seconds(), ccfg_.pose_timeout)) {
                // The same odometry requirement applies before the corridor:
                // neither flight control nor coverage can use a frozen pose.
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[探索里程计] 有效定位已 %.3fs 未更新(阈值 %.3fs)，暂停运动/覆盖更新，等新定位恢复",
                    pose_freshness_.age(steady_seconds()), ccfg_.pose_timeout);
                cmd_pub_->publish(cmd);
                return;
            }

            corridor_active = corridor_->active();
            if (corridor_active && corridor_disabled_for_goal_ && !corridor_->startedMoving()) {
                if (!finished_) { finished_ = true; publish_finished = true; }
                corridor_command_ = {};
                corridor_command_.status = "Corridor disabled for this mission";
            } else if (corridor_active) {
                const double local_now = steady_seconds();
                const bool pose_fresh = pose_freshness_.fresh(local_now, ccfg_.pose_timeout);
                if (!pose_fresh) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[走廊里程计] 距本机收到新有效帧 %.3fs，超时阈值 %.3fs；检查 /aft_mapped_to_init 是否断流或时间戳停止推进",
                        pose_freshness_.age(local_now), ccfg_.pose_timeout);
                }
                corridor_command_ = corridor_->update({px_, py_}, yaw_, v_fwd_est_, v_lat_est_,
                                                       local_now, pose_fresh);
                cmd.twist.linear.x = corridor_command_.forward;
                cmd.twist.linear.y = corridor_command_.lateral;
                cmd.twist.angular.z = corridor_command_.yaw_rate;
                last_look_ = corridor_command_.target;
                look_valid_ = true;
                traj_.clear();
                for (const auto& p : corridor_command_.path) {
                    TrajPoint tp{};
                    tp.p = p;
                    traj_.push_back(tp);
                }
                if (corridor_command_.finished && !finished_) { finished_ = true; publish_finished = true; }
                if (corridor_command_.status != last_corridor_status_) {
                    last_corridor_status_ = corridor_command_.status;
                    RCLCPP_INFO(get_logger(), "[走廊] %s (已过门 %d)",
                                last_corridor_status_.c_str(), corridor_->gatesPassed());
                }
                if (corridor_->phase() != CorridorPhase::Done) {
                    const auto& observed = corridor_->observation();
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[走廊识别] reason=%s cloud=%zu walls=%d bounds=[%.2f,%.2f] observed_s=%.2f gate=%d gate_s=%.2f gap=%.2f rays=%d body=%.2f topic=%s rx_age=%.2fs filter=%s points=%zu/%zu/%zu z=[%.2f,%.2f] raw_z=[%.2f,%.2f] entry=(%.2f,%.2f) H=(%.2f,%.2f)",
                        observed.reason.c_str(), corridor_->points().size(), observed.walls_observed,
                        observed.right_wall, observed.left_wall, observed.observed_until,
                        observed.gate_observed, observed.gate_s, observed.gap_width, observed.clear_rays, ccfg_.perception.robot_width,
                        corridor_cloud_topic_.c_str(), corridor_cloud_received_at_ >= 0.0
                            ? local_now - corridor_cloud_received_at_ : -1.0,
                        corridor_cloud_filter_reason_.c_str(), corridor_cloud_raw_points_,
                        corridor_cloud_height_points_, corridor_cloud_kept_points_,
                        pz_ - corridor_z_below_, pz_ + corridor_z_above_,
                        corridor_cloud_min_z_, corridor_cloud_max_z_,
                        corridor_->entry().x, corridor_->entry().y, corridor_->goal().x, corridor_->goal().y);
                }
            } else {
            // 取一份当前障碍圆（DWA 避障 + 视野遮挡 + 可视化共用）
            const Obstacles obstacles = obs_map_->snapshot();

            if (exploration_enabled_) {
                // 把当前视野标进栅格（障碍背后被遮挡的格不标，模拟摄像头）
                grid_->mark_scan(px_, py_, yaw_, obstacles);
                // 障碍实体内部无需扫描，直接计为已知。
                grid_->fill_obstacle_cells(obstacles);
            }

            // ============================================================
            // POI（途中必经点/插点）状态机 —— 优先级高于探索与归航判定。
            //   EXPLORE      : 无插点活动，走下面常规探索/归航。
            //   GOTO_POI     : 飞向当前插点(走 tracker)，到点 → 转 WAIT_RELEASE。
            //   WAIT_RELEASE : 到点后持续修正XY，等待主控外部数据流放行；
            //                  收到同点 z=1 放行 → 弹队、回 EXPLORE 继续探索。
            // 多插点：排队当前优先；放行后弹下一个。
            // ============================================================
            bool poi_active = false;
            bool vision_target_active = false;

            // 空闲(EXPLORE)且队列有插点 → 取队首开始去飞（归航后不再接插点）
            if (poi_mode_ == PoiMode::EXPLORE && !homing_ && !poi_queue_.empty()) {
                poi_target_ = poi_queue_.front();
                poi_mode_ = PoiMode::GOTO_POI;
                reset_visual_approach();
                turning_for_solution_ = false;   // 切去插点：打断探索的原地转身找解
                global_has_ = global_failed_ = false;
                global_tracker_->set_trajectory({});
                RCLCPP_INFO(get_logger(), "前往插点 (%.2f, %.2f)", poi_target_.x, poi_target_.y);
            }

            if (poi_mode_ == PoiMode::GOTO_POI) {
                poi_active = true;
                // Required points use a separate tracker and a checked exact endpoint.
                const Vec2 vb = pd_to_point_avoid(poi_target_, obstacles);  // 内部写 traj_/last_look_
                retreating_ = false;
                if (global_failed_) {
                    cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Required POI (%.2f, %.2f) has no validated path; holding",
                        poi_target_.x, poi_target_.y);
                } else {
                    retreating_ = false;   // A* 又通了 → 退出后退态
                    cmd.twist.linear.x  = vb.x;
                    cmd.twist.linear.y  = vb.y;
                    cmd.twist.angular.z = last_yaw_rate_;
                }
                const double d = std::hypot(poi_target_.x - px_, poi_target_.y - py_);
                const bool exact_path = !global_traj_.empty() &&
                    std::hypot(global_traj_.back().p.x - poi_target_.x,
                               global_traj_.back().p.y - poi_target_.y) < 1e-6;
                if (!global_failed_ && !global_braking_blocked_ && global_at_goal_ && exact_path &&
                    global_tracker_->remaining_distance() <= goal_tol_ && d <= goal_tol_) {
                    poi_mode_ = PoiMode::WAIT_RELEASE;
                    release_pending_ = false;   // 清掉到点前可能误收的放行
                    RCLCPP_INFO(get_logger(),
                        "已到插点 (%.2f, %.2f)，悬停等待主控放行(z=1)", poi_target_.x, poi_target_.y);
                }
            } else if (poi_mode_ == PoiMode::WAIT_RELEASE) {
                poi_active = true;
                // 等待放行期间仍纠正位置漂移；主控自己的动作优先级不变。
                const Vec2 correction = pd_to_point_avoid(poi_target_, obstacles);
                cmd.twist.linear.x = correction.x; cmd.twist.linear.y = correction.y;
                cmd.twist.angular.z = last_yaw_rate_;
                last_look_ = poi_target_; look_valid_ = true;
                // 放行：z=1 且坐标匹配当前等待点(±POI_SAME_TOL) → 弹队、回探索
                const bool match = release_pending_ &&
                    std::hypot(release_x_ - poi_target_.x, release_y_ - poi_target_.y) <= POI_SAME_TOL;
                if (match) {
                    release_pending_ = false;
                    if (!poi_queue_.empty()) poi_queue_.pop_front();
                    poi_mode_ = PoiMode::EXPLORE;
                    global_has_ = global_failed_ = global_goal_blocked_ = false;
                    global_tracker_->set_trajectory({});
                    plan_pending_ = true;   // 回探索后立刻重规划接着扫
                    RCLCPP_INFO(get_logger(), "收到放行 → 继续探索");
                }
            }

            // Visual target execution uses the same exact-goal A* and tracker
            // as required POIs. Completion/hover is deliberately handled by a
            // later stage; this branch only proves safe motion and near speed.
            if (!poi_active && !homing_ && has_pose_ && vision_queue_->active()) {
                vision_target_active = true;
                const auto target = *vision_queue_->active();
                const Vec2 vb = visual_target_velocity(target, obstacles);
                const bool straight_approach = vision_straight_track_ == target.track_id;
                if (global_failed_ || global_goal_blocked_) {
                    cmd.twist.linear.x = cmd.twist.linear.y = cmd.twist.angular.z = 0.0;
                    vision_last_event_ = "visual_path_unavailable";
                    // An occupied/out-of-bounds visual point must not stall
                    // exploration indefinitely.  Defer it for a later fresh
                    // observation and release control back to exploration.
                    const auto uid = target.track_id;
                    vision_queue_->defer_active(uid, steady_seconds());
                    reset_visual_approach();
                    vision_near_->reset();
                    vision_hover_track_ = 0;
                    vision_hover_since_ = -1.0;
                    global_has_ = global_failed_ = global_goal_blocked_ = false;
                    global_braking_blocked_ = false;
                    global_tracker_->set_trajectory({});
                    plan_pending_ = true;
                    vision_target_active = false;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[视觉目标] 区域#%llu 当前不可达，暂缓并继续探索",
                        static_cast<unsigned long long>(uid));
                } else {
                    const double normal_speed = std::hypot(vb.x, vb.y);
                    const double cap = vision_near_->speed_cap(gains_.v_max);
                    const double scale = normal_speed > cap && normal_speed > 1e-9
                        ? cap / normal_speed : 1.0;
                    cmd.twist.linear.x = vb.x * scale;
                    cmd.twist.linear.y = vb.y * scale;
                    // The tracker/guard already validated this vector and its
                    // acceleration. Only reduce its magnitude here: boosting or
                    // changing its direction would undo turn holds and braking.
                    cmd.twist.angular.z = last_yaw_rate_;
                    vision_last_event_ = straight_approach ? "visual_target_straight_approach" :
                        (vision_near_->near_entered() ? "visual_target_near_limited" : "visual_target_path_following");
                    const bool exact_target = target.refined &&
                        std::hypot(target.position.x - px_, target.position.y - py_) <= vision_target_tol_;
                    // First entry into the position tolerance starts the dwell,
                    // independently of velocity or tracker stopping state. Continue
                    // correcting drift above, without restarting this timer.
                    if (exact_target || vision_hover_track_ == target.track_id) {
                        vision_last_event_ = "visual_target_hovering";
                        if (vision_hover_track_ != target.track_id) {
                            vision_hover_track_ = target.track_id;
                            vision_hover_since_ = steady_seconds();
                            RCLCPP_INFO(get_logger(), "[视觉目标] 区域#%llu 进入目标范围，开始悬停 %.1fs（不等待停稳，偏移继续矫正，不重置计时）",
                                static_cast<unsigned long long>(target.track_id), vision_hover_s_);
                        } else if (steady_seconds() - vision_hover_since_ >= vision_hover_s_) {
                            if (vision_queue_->complete_active(target.track_id, steady_seconds())) {
                                const auto completed = vision_queue_->visited().back();
                                vision_candidates_->discard_visited(completed.track_id, completed.position,
                                    vision_queue_->blacklist_radius());
                                vision_near_->reset();
                                vision_applied_visits_ = vision_queue_->visited().size();
                                vision_hover_track_ = 0;
                                vision_hover_since_ = -1.0;
                                reset_visual_approach();
                                vision_last_event_ = "visual_target_completed";
                                global_has_ = global_failed_ = false;
                                global_tracker_->set_trajectory({});
                                plan_pending_ = true;
                                activate_visual_target(steady_seconds());
                                RCLCPP_INFO(get_logger(), "[视觉目标] 完成并加入 %.2fm 黑名单，切换下一个目标",
                                    vision_queue_->blacklist_radius());
                            }
                        }
                    }
                }
            }

            // 完程度(覆盖率)达标 → 进入归航刹停阶段（一旦进入不再回退）。
            // 注意：仅在没有插点活动时才判定/接管，避免插点途中误触发归航。
            //   覆盖率只读一次并复用：判定与日志同源，日志里印的就是真正触发归航的那个值。
            if (!poi_active && !vision_target_active && has_goal_ && !homing_) {
                const double cov = grid_->coverage_ratio();
                if (cov >= done_coverage_) {
                    homing_ = true;
                    turning_for_solution_ = false;            // 转归航：打断探索的原地转身找解
                    tracker_->set_trajectory(Trajectory{});
                    global_has_ = global_failed_ = false;
                    global_tracker_->set_trajectory({});
                    RCLCPP_INFO(get_logger(),
                        "完程度达标(%.0f%%) → 归航，沿安全轨迹到终点 (%.2f, %.2f) 刹停",
                        cov * 100.0, goal_.x, goal_.y);
                }
            }

            if (poi_active || vision_target_active) {
                // 插点活动中：命令已在上面 POI 状态机里写好，这里不再覆盖。
            } else if (has_goal_ && homing_) {
                // ---- 归航：直奔真实终点(不夹取)，平滑刹停。终点可能藏在障碍后 → 全局 A* 绕障。 ----
                Vec2 vb = pd_to_point_avoid(goal_, obstacles);   // 内部写 traj_/last_look_
                retreating_ = false;
                if (global_failed_) {
                    cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Required home goal (%.2f, %.2f) has no validated path; holding",
                        goal_.x, goal_.y);
                } else {
                    retreating_ = false;   // A* 又通了 → 退出后退态
                    cmd.twist.linear.x  = vb.x;
                    cmd.twist.linear.y  = vb.y;
                    cmd.twist.angular.z = last_yaw_rate_;
                }

                // 到达有效终点即可交接，XY纠偏延续到下一动作，不等待速度归零。
                const double d = std::hypot(goal_.x - px_, goal_.y - py_);
                const bool exact_path = !global_traj_.empty() &&
                    std::hypot(global_traj_.back().p.x - goal_.x,
                               global_traj_.back().p.y - goal_.y) < 1e-6;
                const bool stopped = !global_failed_ && !global_braking_blocked_ &&
                    global_at_goal_ && global_tracker_->remaining_distance() <= goal_tol_ && exact_path &&
                    (d <= goal_tol_);
                if (stopped) {
                    if (corridor_enabled_ && !corridor_disabled_for_goal_) {
                        corridor_->start({px_, py_}, steady_seconds());
                        corridor_active = true;
                        tracker_->set_trajectory({});
                        global_tracker_->set_trajectory({});
                        traj_.clear();
                        last_look_ = {px_, py_};
                        RCLCPP_INFO(get_logger(), "到达探索交接点，开始转向/入口横移/穿门/H任务");
                    } else if (!finished_) { finished_ = true; publish_finished = true; }
                }
            } else {
                // ---- 探索：周期/偏离重规划(A* 绕障重铺覆盖路径) + tracker 跟随 ----
                // Visual/POI visits overwrite the shared display trajectory.
                // Restore the actual exploration reference before replanning:
                // keeping an existing route does not call adopt_explore_path().
                traj_ = tracker_->trajectory();
                //   绕障全由 replan_locked 内的 A* 完成(含离墙)，跟随手感与原状一致；无障碍时
                //   A* 退化成直线，行为不变。A* 无解 → replan_locked 内已跳带悬停(放弃该区)。
                bool handled = false;

                // ★原地转身找解★优先：锥内无解但侧向有路时，本拍原地转身改朝向重搜(画旋转标志)。
                //   step 返回 true=本拍已写 cmd(继续转/刚判真围死悬停)；false=刚解出(traj_已设)→落到下面跟随。
                if (turning_for_solution_) {
                    handled = step_turn_for_solution(obstacles, cmd);
                }

                if (!handled) {
                    if (has_goal_ && !finished_) {
                        bool need = plan_pending_;          // 外部触发(新goal/POI放行)始终尊重
                        if (!need) {
                            if (!tracker_->has_trajectory()) {
                                need = true;
                            } else {
                                const double age = (now() - last_plan_time_).seconds();
                                if (age >= replan_period_) need = true;
                                else if (tracker_->last_nearest_dist() > replan_dev_) {
                                    const Vec2 cur{px_, py_};
                                    need = !path_clear(cur, tracker_->remaining_path(px_, py_), obstacles, ggcfg_);
                                }
                            }
                        }
                        if (need) replan_locked({px_, py_});
                    }

                    // tracker 跟随当前 traj_(已是 A* 绕障轨迹)。A* 无解时 traj_ 被清空。
                    if (tracker_->has_trajectory()) {
                        retreating_ = false;   // 有轨迹可走 → 退出后退态
                        const bool was_reorienting = tracker_->reorienting();
                        VelCmd vc = observation_arrived_ && have_observation_target_ ? VelCmd{} :
                            tracker_->update(px_, py_, yaw_, v_fwd_est_, v_lat_est_, goal_tol_, yaw_rate_est_);
                        if (vc.needs_replan) {
                            plan_pending_ = true;
                            explore_has_committed_ = false;
                            tracker_->set_trajectory({});
                        }
                        finish_observation(vc, obstacles);
                        if (vc.holding_position && !turn_correction_clear({vc.v_fwd, vc.v_lat}, obstacles))
                            vc.v_fwd = vc.v_lat = 0.0;
                        if (vc.holding_position &&
                            !obstacle_segment_clear({px_, py_}, {px_, py_}, obstacles, ggcfg_))
                            vc.yaw_rate = 0.0;
                        if (tracker_->reorienting() != was_reorienting) {
                            if (tracker_->reorienting()) ++recovery_entries_;
                            RCLCPP_INFO(get_logger(), "[探索纠偏] %s",
                                tracker_->reorienting() ? "大角度偏差：XY定点纠偏并持续转向" : "航向与角速度稳定，平滑恢复前进");
                        }
                        last_look_ = tracker_->last_lookahead();
                        look_valid_ = true;
                        cmd.twist.linear.x  = vc.v_fwd;
                        cmd.twist.linear.y  = vc.v_lat;
                        cmd.twist.angular.z = vc.yaw_rate;
                        if (vc.at_goal && (explore_goal_projected_ || !have_observation_target_)) {
                            // An exhausted route without a separate observation
                            // action must release control too, even when nearby
                            // unseen cells make its target appear productive.
                            unreachable_.push_back(explore_target_);
                            explore_has_committed_ = false;
                            tracker_->set_trajectory({});
                            traj_.clear();
                            plan_pending_ = true;
                        }
                    } else if (turning_for_solution_) {
                        // replan_locked 本拍刚进入转身态(锥内无解+侧向有路) → 本拍即转身一步，别落到悬停。
                        step_turn_for_solution(obstacles, cmd);
                    } else if (explore_failed_) {
                        // A* 无解被清空轨迹：先试贴障后退脱困(退出去下拍重规划接着扫)；
                        //   非贴脸卡死/退到极限仍无路 → 本拍悬停(replan_locked 已跳带，下拍去别处)。
                        if (!try_retreat(obstacles, cmd)) {
                            cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
                        } else {
                            plan_pending_ = true;   // 后退后下一拍立刻重规划(此时可能已能绕)
                        }
                    }
                }
            }

            } // 常规探索/归航；走廊任务独立处理墙与门。

            if (++diagnostic_ticks_ % 10 == 0) {
                publish_diagnostics = true;
                coverage.data = grid_->coverage_ratio();
                active_path.header.stamp = cmd.header.stamp;
                active_path.header.frame_id = "camera_init";
                for (const auto& point : traj_) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = active_path.header;
                    pose.pose.position.x = point.p.x;
                    pose.pose.position.y = point.p.y;
                    pose.pose.position.z = pz_;
                    pose.pose.orientation.w = 1.0;
                    active_path.poses.push_back(pose);
                }
                diagnostics.header = active_path.header;
                diagnostic_msgs::msg::DiagnosticStatus status;
                status.name = "exploration/planner";
                status.hardware_id = "exploration_planner";
                status.message = corridor_active ? corridor_command_.status :
                    (homing_ ? "homing" : (tracker_->reorienting() ? "reorienting" : "exploration"));
                auto value = [&status](const std::string& key, const std::string& text) {
                    diagnostic_msgs::msg::KeyValue item;
                    item.key = key;
                    item.value = text;
                    status.values.push_back(item);
                };
                auto number = [&value](const std::string& key, auto scalar) {
                    value(key, std::to_string(scalar));
                };
                value("mode", corridor_active ? "corridor" : (homing_ ? "homing" : "exploration"));
                value("exploration_enabled", exploration_enabled_ ? "true" : "false");
                value("vision_shm_status", vision_reader_
                    ? VisionShmReader::status_name(vision_reader_->status()) : "disabled");
                value("vision_task_stage", "queue_and_near_confirmation_only");
                value("vision_last_event", vision_last_event_);
                number("vision_pending_count", vision_queue_->pending().size());
                number("vision_deferred_count", vision_queue_->deferred().size());
                number("vision_visited_count", vision_queue_->visited().size());
                number("vision_queue_enqueued", vision_queue_->statistics().enqueued);
                number("vision_queue_capacity_drops", vision_queue_->statistics().capacity_drops);
                number("vision_blacklisted_detections", vision_queue_->statistics().blacklisted_detections);
                number("vision_near_confirmations", vision_near_confirmations_);
                number("vision_near_entered", vision_near_->near_entered());
                number("vision_near_frames", vision_near_->count());
                number("vision_near_vote_tied", vision_near_->tied());
                number("vision_requested_speed_cap", vision_near_->speed_cap(gains_.v_max));
                number("vision_hover_track", vision_hover_track_);
                number("vision_straight_track", vision_straight_track_);
                number("vision_hover_elapsed_s", vision_hover_since_ >= 0 ? steady_seconds() - vision_hover_since_ : 0.0);
                number("vision_active_track", vision_queue_->active() ? vision_queue_->active()->track_id : 0);
                if (vision_queue_->active()) {
                    const auto& active = *vision_queue_->active();
                    number("vision_active_class", active.class_id);
                    number("vision_active_refined", active.refined);
                    number("vision_active_x", active.position.x);
                    number("vision_active_y", active.position.y);
                }
                for (int id = 1; id <= 6; ++id)
                    number("vision_votes_" + std::to_string(id), vision_near_->votes()[id]);
                size_t queue_index = 0;
                for (const auto& target : vision_queue_->pending()) {
                    const auto prefix = "vision_pending_" + std::to_string(queue_index++) + "_";
                    number(prefix + "track", target.track_id);
                    number(prefix + "x", target.position.x);
                    number(prefix + "y", target.position.y);
                }
                number("vision_collection_active", vision_collection_active_);
                number("vision_candidates", vision_candidates_->size());
                const auto& vision_stats = vision_candidates_->statistics();
                number("vision_far_confirmations", vision_stats.confirmations);
                number("vision_duplicate_detections", vision_stats.suppressed_duplicates);
                number("vision_capacity_drops", vision_stats.capacity_drops);
                number("vision_invalid_detections", vision_stats.invalid_detections);
                for (const auto& candidate : vision_candidates_->snapshot()) {
                    const auto prefix = "vision_region_" + std::to_string(candidate.track_id) + "_";
                    number(prefix + "count", candidate.consecutive_frames);
                    number(prefix + "confirmed", candidate.confirmed);
                    number(prefix + "x", candidate.position.x);
                    number(prefix + "y", candidate.position.y);
                }
                if (vision_reader_) {
                    const auto& stats = vision_reader_->statistics();
                    number("vision_frames_received", stats.frames);
                    number("vision_empty_frames", stats.empty_frames);
                    number("vision_writer_restarts", stats.restarts);
                    number("vision_invalid_polls", stats.invalid_polls);
                    number("vision_frame_seq", vision_last_frame_.seq);
                    number("vision_frame_epoch", vision_last_frame_.epoch);
                    number("vision_frame_age_s", stats.frames ? steady_seconds() - vision_last_frame_.stamp : -1.0);
                    number("vision_target_count", vision_last_frame_.count);
                    for (size_t i = 0; i < vision_last_frame_.count; ++i) {
                        const auto prefix = "vision_target_" + std::to_string(i) + "_";
                        const auto& target = vision_last_frame_.targets[i];
                        number(prefix + "id", target.id);
                        number(prefix + "x", target.x);
                        number(prefix + "y", target.y);
                    }
                }
                number("coverage_ratio", coverage.data);
                number("replan_checks", replan_checks_);
                number("candidate_skips", candidate_skips_);
                number("active_band", cur_band_);
                number("frontier_observation", have_observation_target_);
                number("frontier_view_gain", observation_target_.gain);
                number("active_target_gain", active_target_gain_);
                number("observation_heading", observation_target_.view_heading);
                value("replan_reason", last_replan_reason_);
                number("kept_after_candidate_failure", kept_after_candidate_failure_);
                number("frontier_view_clearance", observation_target_.clearance);
                number("frontier_region_cells", observation_target_.region_size);
                number("frontier_cleanup", observation_target_.cleanup);
                number("astar_searches", astar_searches_);
                number("adoptions", adoptions_);
                number("invalid_paths", invalid_paths_);
                number("required_goal_blocked", global_goal_blocked_);
                number("required_path_blocked", global_failed_);
                number("required_velocity_blocked", global_braking_blocked_);
                number("command_projection_blocked", command_projection_blocked_);
                number("measured_projection_blocked", measured_projection_blocked_);
                number("required_motion_inflate", required_motion_inflate_);
                number("required_blocked_replans", required_blocked_replans_);
                value("required_block_reason", global_block_reason_);
                number("required_path_searches", global_path_searches_);
                number("required_smooth_fallbacks", global_smooth_fallbacks_);
                number("required_invalid_paths", global_invalid_paths_);
                number("required_velocity_brakes", global_velocity_brakes_);
                number("required_heading_error", global_tracker_->heading_error());
                number("required_remaining_s", global_tracker_->remaining_distance());
                number("required_curvature", global_tracker_->reference_curvature());
                number("required_next_corner_distance", global_tracker_->next_corner_distance());
                number("required_at_goal", global_at_goal_);
                number("smooth_fallbacks", smooth_fallbacks_);
                value("smooth_failure_reason", smooth_failure_reason_);
                const Obstacles observed_obstacles = obs_map_->snapshot();
                number("current_obstacle_clearance", nearest_obstacle_dist(observed_obstacles));
                number("turn_entries", turn_entries_);
                number("recovery_entries", recovery_entries_);
                number("target_x", explore_target_.x);
                number("target_y", explore_target_.y);
                number("lookahead_x", last_look_.x);
                number("lookahead_y", last_look_.y);
                number("progress_s", tracker_->progress_distance());
                number("remaining_s", tracker_->remaining_distance());
                number("heading_error", tracker_->heading_error());
                number("reference_curvature", tracker_->reference_curvature());
                number("next_corner_distance", tracker_->next_corner_distance());
                const auto& gate = corridor_->gateLocked() ? corridor_->gate() : corridor_->observation();
                number("gate_center_x", gate.gate_center.x);
                number("gate_center_y", gate.gate_center.y);
                number("gate_width", gate.gap_width);
                value("gate_reason", gate.reason);
                value("corridor_cloud_topic", corridor_cloud_topic_);
                value("corridor_cloud_filter", corridor_cloud_filter_reason_);
                number("corridor_cloud_raw_points", corridor_cloud_raw_points_);
                number("corridor_cloud_height_points", corridor_cloud_height_points_);
                number("corridor_cloud_kept_points", corridor_cloud_kept_points_);
                number("corridor_cloud_receive_age_s", corridor_cloud_received_at_ >= 0.0
                    ? steady_seconds() - corridor_cloud_received_at_ : -1.0);
                number("gates_passed", corridor_->gatesPassed());
                diagnostics.status.push_back(status);
            }
            if (publish_finished) completed_goal = goal_header_;
        }

        std_msgs::msg::Bool active_msg;
        active_msg.data = corridor_active;
        corridor_active_pub_->publish(active_msg);
        cmd_pub_->publish(cmd);
        if (publish_diagnostics) {
            coverage_pub_->publish(coverage);
            active_path_pub_->publish(active_path);
            diagnostics_pub_->publish(diagnostics);
        }

        if (publish_finished) {
            finished_goal_pub_->publish(completed_goal);
            std_msgs::msg::Bool b; b.data = true;
            finished_pub_->publish(b);
            RCLCPP_INFO(get_logger(), "任务完成：已到达%s → finished=true", corridor_active ? "H点" : "探索终点");
        }

        // rviz 可视化：处理后障碍点云(每拍) + 设定边界框(首拍一次)。都在锁外，不阻塞主循环。
        publish_obstacle_cloud();
        if (!boundary_sent_) { publish_field_boundary(); boundary_sent_ = true; }
    }

    // 发布"处理后障碍点云"：obstacle_map 的占据格中心点(算法真正当障碍的点，已滤地面/自身/范围外/
    //   贴墙)。世界系 camera_init，z 抬到可见高度。rviz Add→PointCloud2→/exploration/obstacle_cloud。
    void publish_obstacle_cloud()
    {
        const std::vector<Vec2> pts = obs_map_->occupied_points();
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = now();
        msg.header.frame_id = "camera_init";
        msg.height = 1;
        msg.width  = static_cast<uint32_t>(pts.size());
        msg.is_dense = true;
        msg.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(pts.size());
        sensor_msgs::PointCloud2Iterator<float> ox(msg, "x"), oy(msg, "y"), oz(msg, "z");
        for (const auto& p : pts) {
            *ox = static_cast<float>(p.x);
            *oy = static_cast<float>(p.y);
            *oz = 0.3f;                      // 抬到 0.3m 高度，rviz 里好看、不被地面盖住
            ++ox; ++oy; ++oz;
        }
        obs_cloud_pub_->publish(msg);
    }

    // 发布"设定边界框"：用 FIELD_*(=gcfg_.min/max_*)画一圈矩形 LINE_STRIP。改 FIELD_* 边界框自动跟着变。
    void publish_field_boundary()
    {
        visualization_msgs::msg::Marker m;
        m.header.stamp = now();
        m.header.frame_id = "camera_init";
        m.ns = "field_boundary";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05;                     // 线宽 5cm
        m.color.r = 1.0f; m.color.g = 0.2f; m.color.b = 0.2f; m.color.a = 1.0f;  // 红框
        m.pose.orientation.w = 1.0;
        const double x0 = gcfg_.min_x, y0 = gcfg_.min_y, x1 = gcfg_.max_x, y1 = gcfg_.max_y;
        auto add = [&](double x, double y) {
            geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = 0.0; m.points.push_back(p);
        };
        add(x0, y0); add(x1, y0); add(x1, y1); add(x0, y1); add(x0, y0);  // 闭合矩形
        boundary_pub_->publish(m);
    }

    // ---- 配置 / 模块 ----
    GridConfig    gcfg_;
    std::unique_ptr<VisionShmReader> vision_reader_;
    std::optional<VisionFrame> vision_new_frame_;
    VisionFrame vision_last_frame_;
    std::unique_ptr<VisionCandidates> vision_candidates_;
    std::unique_ptr<VisionTargetQueue> vision_queue_;
    std::unique_ptr<VisionNearConfirmation> vision_near_;
    size_t vision_applied_visits_ = 0;
    std::uint64_t vision_near_confirmations_ = 0;
    std::string vision_last_event_ = "none";
    std::vector<VisionCandidate> vision_new_confirmations_;
    bool vision_collection_active_ = false;
    double vision_pose_timeout_ = params::VISION_POSE_TIMEOUT_S;
    double vision_hover_s_ = params::VISION_HOVER_S;
    double vision_straight_approach_m_ = params::VISION_STRAIGHT_APPROACH_M;
    std::uint64_t vision_straight_track_ = 0;
    Vec2 vision_straight_velocity_{};
    double vision_target_tol_ = params::VISION_TARGET_TOL_M;
    double vision_target_stop_speed_ = params::VISION_TARGET_STOP_SPEED_MPS;
    std::uint64_t vision_hover_track_ = 0;
    double vision_hover_since_ = -1.0;
    CorridorConfig ccfg_;
    std::unique_ptr<CorridorController> corridor_;
    CorridorCommand corridor_command_;
    bool corridor_enabled_ = true;
    std::string corridor_cloud_topic_;
    bool corridor_disabled_for_goal_ = false;
    bool corridor_route_received_ = false;
    nav_msgs::msg::Path pending_corridor_route_;
    builtin_interfaces::msg::Time goal_stamp_;
    std_msgs::msg::Header goal_header_;
    double corridor_z_below_ = 0.3, corridor_z_above_ = 0.3;
    double corridor_self_radius_ = 0.1, corridor_sensor_range_ = 6.0;
    double corridor_cloud_max_yaw_rate_ = 0.6;
    SensorFreshness pose_freshness_, cloud_freshness_;
    std::string last_corridor_status_;
    double corridor_cloud_received_at_ = -1.0;
    size_t corridor_cloud_raw_points_ = 0, corridor_cloud_height_points_ = 0, corridor_cloud_kept_points_ = 0;
    double corridor_cloud_min_z_ = 0.0, corridor_cloud_max_z_ = 0.0;
    std::string corridor_cloud_filter_reason_ = "no_messages";
    bool          carlike_mode_ = params::EXPLORATION_CARLIKE_MODE;
    double        turn_blend_m_ = params::CARLIKE_TURN_BLEND_M;
    double        turn_round_min_m_ = params::TURN_ROUND_MIN_M;
    double        turn_round_max_curvature_ = params::TURN_ROUND_MAX_CURVATURE;
    double        turn_blend_min_angle_rad_ = params::CARLIKE_TURN_BLEND_ANGLE_DEG * M_PI / 180.0;
    double        turn_blend_max_angle_rad_ = params::TURN_BLEND_MAX_ANGLE_DEG * M_PI / 180.0;
    int           turn_blend_samples_ = params::CARLIKE_TURN_BLEND_SAMPLES;
    TrackerGains  gains_;
    FrontierConfig fcfg_;
    bool exploration_enabled_ = params::EXPLORATION_ENABLED;
    bool frontier_observation_enabled_ = params::FRONTIER_OBSERVATION_ENABLED;
    bool have_observation_target_ = false;
    FrontierSelection observation_target_, turn_observation_;
    double observation_yaw_rate_ = 0.0;
    PositionHold observation_hold_, turn_hold_;
    PositionHold required_goal_hold_;
    bool required_goal_holding_ = false;
    double required_goal_yaw_ = 0.0;
    double required_brake_speed_ = 0.0;
    bool observation_turn_active_ = false;
    double frontier_continuity_lookahead_ = params::FRONTIER_CONTINUITY_LOOKAHEAD_M;
    int active_target_gain_ = -1;
    std::string last_replan_reason_ = "none";
    uint64_t kept_after_candidate_failure_ = 0;
    bool explore_handoff_deferred_ = false;
    bool observation_arrived_ = false;
    double guide_min_segment_ = params::EXPLORE_GUIDE_MIN_SEGMENT_M;
    double guide_max_deviation_ = params::EXPLORE_GUIDE_MAX_DEVIATION_M;
    double explore_plan_reserve_ = params::EXPLORE_PLAN_RESERVE_M;
    double        lane_spacing_;       // 旧牛耕车道间距：已不参与任何计算(见构造里说明)，保留仅为参数表兼容
    double        arc_ds_;
    double        done_coverage_, goal_tol_, v_est_alpha_;
    double        goal_stop_v_, kp_goal_, kd_goal_, v_goal_max_;
    double        replan_period_ = 0.7, replan_dev_ = 0.5;
    double        candidate_period_ = params::REPLAN_CANDIDATE_PERIOD_S;
    // 目标失效判定邻域半径 (m)：承诺目标此半径内已无未扫大格 → 该目标已无信息可拿，
    //   立刻放弃承诺改投别处(消除"已覆盖还在往前飞")。实际值由 params::TARGET_STALE_R 覆盖。
    double        target_stale_r_ = 1.00;
    bool          viz_;

    // ---- 雷达感知配置 ----
    ObstacleConfig ocfg_;
    double         robot_radius_ = 0.30;   // 飞机半径(碰撞判定+自身回波滤除+A*禁入)，真机 30cm，由 ROBOT_RADIUS 覆盖
    double         ground_z_ = 0.50;       // 点云高度窗口·下限 (世界 z)：低于此丢弃(地面/近地杂物)
    double         ceil_z_   = 2.00;       // 点云高度窗口·上限 (世界 z)：高于此丢弃(天花板/高处/吊挂物)
    double         obs_skip_yaw_rate_ = 0.60; // 高角速度门控(rad/s)：|yaw_rate|超此丢弃整帧点云(防旋转拖影污染)，由 OBS_SKIP_YAW_RATE 覆盖
    double         self_margin_ = 0.15;    // 自身回波余量

    // ---- 全局点到点绕障（A*，全局层）配置：全场唯一避障手段 ----
    GlobalConfig   ggcfg_;
    double home_goal_wall_margin_ = params::HOME_GOAL_WALL_MARGIN;
    double         commit_target_tol_ = 0.50;  // 路径承诺：目标移动超此距离才算"换地方"重算 A*(m)
    double         global_lookahead_ = 0.50;  // 沿绕障轨迹取 carrot 的前瞻距离 (m)
    double         explore_target_min_dist_ = 1.50; // 探索挑 A* 终点的最小距离 (m)
    // ★换路评估(路径迟滞)★：重算出新 A* 路径后，只有它【更安全/更快】超过下列比例才换路(否则续用旧路)，根除左右横跳。
    //   ★注意★：下面这些成员初值只是"构造函数跑之前的占位"，真正生效的是构造里
    //   declare_parameter 填入的 params:: 常量(当前 0.50 / 0.20 / true)。改参数请改 params.hpp 或用 -p 覆盖。
    double         path_switch_safety_gain_  = 0.50;  // 新路最小障碍边距需比旧路高 ≥此比例才算更安全（实际值见 params::PATH_SWITCH_SAFETY_GAIN）
    double         path_switch_time_gain_    = 0.20;  // 新路剩余弧长需比旧路短 ≥此比例才算更快（实际值见 params::PATH_SWITCH_TIME_GAIN）
    bool           path_switch_require_both_ = true;  // true=两者都满足才换(最稳)；false=任一满足即换(更看重效率)

    std::unique_ptr<GridMap>           grid_;
    std::unique_ptr<TrajectoryTracker> tracker_;
    std::unique_ptr<TrajectoryTracker> global_tracker_;
    TrackerGains global_gains_{};
    std::unique_ptr<Visualizer>        viz_obj_;
    std::unique_ptr<ObstacleMap>       obs_map_;

    // ---- ROS ----
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              finished_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr            finished_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr coverage_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_path_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    uint64_t diagnostic_ticks_ = 0, replan_checks_ = 0, astar_searches_ = 0;
    uint64_t candidate_skips_ = 0;
    uint64_t adoptions_ = 0, smooth_fallbacks_ = 0, turn_entries_ = 0, recovery_entries_ = 0;
    uint64_t invalid_paths_ = 0;
    std::string smooth_failure_reason_ = "none";
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr corridor_active_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr corridor_route_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    obs_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  boundary_pub_;
    bool boundary_sent_ = false;   // 边界框只需发一次(latched)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr poi_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr corridor_cloud_sub_;
    rclcpp::CallbackGroup::SharedPtr poi_cbg_;
    rclcpp::CallbackGroup::SharedPtr cloud_cbg_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- 共享状态（mtx_ 保护）----
    std::mutex mtx_;
    double px_ = 0.0, py_ = 0.0, pz_ = 0.0, yaw_ = 0.0;
    bool   has_pose_ = false;
    Vec2   goal_;
    bool   has_goal_ = false;
    bool   plan_pending_ = false;
    Trajectory traj_;
    Vec2   last_look_;
    bool   look_valid_ = false;
    bool   finished_ = false;
    bool   homing_ = false;            // 覆盖率达标后进入归航刹停阶段（不再走 tracker）
    int    cur_band_ = -1;             // 上层扫描条带进度（-1=未初始化，由 plan_explore 维护）
    rclcpp::Time last_plan_time_;      // 上次重规划时刻（构造里用 now() 初始化）
    rclcpp::Time last_candidate_time_;
    bool candidate_time_valid_ = false;

    // ---- 探索 A* 重铺运行态（mtx_ 保护）----
    bool         explore_failed_ = false;  // 上次探索 A* 无解(目标被围死) → 放弃该区跳带
    Path2        explore_raw_;             // 上次采纳的探索绕障裸折线(供 path_clear 承诺校验)
    bool         explore_has_committed_ = false;  // 已采纳一条探索绕障路径(承诺中)
    bool         explore_goal_projected_ = false;
    Vec2         explore_target_;          // 该承诺路径对应的 scan_target(移动超容差才换边)

    // ---- 放弃区域·黑名单（mtx_ 保护）----
    std::vector<Vec2> unreachable_;            // A* 够不到的探索目标集；plan_explore 选点跳过其邻域
    double       last_unreach_clear_cov_ = 0.0;// 上次按覆盖率台阶清空黑名单时的覆盖率
    double       unreach_block_r_    = 0.80;   // 拉黑邻域半径(m)，构造里由 UNREACH_BLOCK_R 覆盖（-p 可调）
    double       unreach_clear_step_ = 0.05;   // 覆盖率每涨过此台阶清空黑名单一次，由 UNREACH_CLEAR_STEP 覆盖

    // ---- 全局点到点绕障（A*）运行态（mtx_ 保护）----
    Trajectory   global_traj_;            // 缓存的绕障轨迹(可视化 + carrot 跟随)
    Path2        global_raw_;             // 缓存绕障裸折线(供 path_clear 承诺校验)
    Vec2         global_target_;          // 该缓存路径对应的目标(变了就重搜)
    bool         global_has_    = false;  // 有缓存路径
    bool         global_failed_ = false;  // 上次 A* 无解(目标被围死) → POI/终点报警悬停
    bool         global_goal_blocked_ = false;
    bool         global_at_goal_ = false;
    bool         global_braking_blocked_ = false;
    bool         command_projection_blocked_ = false, measured_projection_blocked_ = false;
    double       required_motion_inflate_ = params::REQUIRED_MOTION_INFLATE;
    double       required_blocked_replan_s_ = params::REQUIRED_BLOCKED_REPLAN_S;
    double       required_blocked_since_ = 0.0;
    bool         required_blocked_time_valid_ = false;
    uint64_t     required_blocked_replans_ = 0;
    Vec2         global_connector_start_;
    uint64_t     global_path_searches_ = 0, global_smooth_fallbacks_ = 0;
    uint64_t     global_invalid_paths_ = 0, global_velocity_brakes_ = 0;
    std::string  global_block_reason_ = "none";
    rclcpp::Time last_global_plan_time_;  // 上次 A* 时刻（节流，构造里 now() 初始化）

    // ---- 脱困后退（mtx_ 保护）：A* 无解且贴障时低速退出来，边退边重算 ----
    bool   retreating_ = false;        // 正在后退脱困
    bool   retreat_exhausted_ = false;
    Vec2   retreat_exhausted_at_;
    Vec2   retreat_origin_;            // 本轮后退起点（算累计后退距离，超 retreat_max_dist_ 放弃）
    double retreat_trigger_  = 0.55;   // 触发后退的贴障边距(m)，构造由 RETREAT_TRIGGER_M 覆盖（-p 可调）
    double retreat_step_     = 0.50;   // 每次后退目标点距离(m)，由 RETREAT_STEP_M 覆盖
    double retreat_max_dist_ = 1.20;   // 累计后退上限(m)，由 RETREAT_MAX_DIST 覆盖
    double retreat_v_max_    = 0.35;   // 后退限速(m/s)，由 RETREAT_V_MAX 覆盖
    rclcpp::Time retreat_start_;       // 进入后退态的时刻（超时兜底，防"退不动"无限重试）
    double retreat_timeout_  = 2.0;    // 后退超时(s)：超此仍没退够 → 判退不动，交上层跳带。由 RETREAT_TIMEOUT_S 覆盖

    // ---- 原地转身找解（mtx_ 保护）：探索锥内无解但有路(只是不在机头方向)→原地转身改朝向重搜 ----
    bool          turning_for_solution_ = false;  // 正在原地转身找解
    bool          turn_heading_valid_ = false;
    Path2         turn_probe_path_;
    Vec2          turn_target_;                   // 转身期间重搜用的目标(=触发时的 scan_target)
    int           turn_candidate_band_ = -1;
    int           turn_dir_ = 1;                  // 转身扫向：+1=逆时针(CCW)/-1=顺时针(CW)，朝 probe 开口方向
    double        turn_prev_yaw_ = 0.0;           // 上一拍 yaw（算本拍增量）
    double        turn_heading_ = 0.0;
    double        turn_command_rate_ = 0.0;
    double        turn_settled_s_ = 0.0;
    double        turn_accum_    = 0.0;           // 累计净转角(带符号 *turn_dir_，达 2π*max_rev 判真围死，抗抖)
    rclcpp::Time  turn_start_time_;               // 进入转身时刻（超时兜底）
    int           turn_research_tick_ = 0;        // 重搜节流计数
    double        turn_solve_yaw_rate_ = 1.20;    // 转身角速度(rad/s)，由 TURN_SOLVE_YAW_RATE 覆盖
    int           turn_solve_research_every_ = 1; // 每隔几拍重搜，由 TURN_SOLVE_RESEARCH_EVERY 覆盖
    double        turn_solve_max_rev_  = 1.0;     // 转此圈数仍无解→真围死，由 TURN_SOLVE_MAX_REV 覆盖
    double        turn_solve_timeout_  = 20.0;    // 转身兜底超时(s)，由 TURN_SOLVE_TIMEOUT_S 覆盖

    // ---- 真无解红叉标记（mtx_ 保护）：目标被围死(转一圈仍无解)→弹窗在该位置画红叉 ----
    bool  has_unreachable_marker_ = false;
    Vec2  unreachable_pos_;

    // 角度归一到 (-π, π]
    static double wrap_pi(double a) { while (a > M_PI) a -= 2.0 * M_PI; while (a <= -M_PI) a += 2.0 * M_PI; return a; }

    // ---- POI（途中必经点/插点）状态机（mtx_ 保护）----
    enum class PoiMode { EXPLORE, GOTO_POI, WAIT_RELEASE };
    static constexpr double POI_SAME_TOL = 0.30;   // 放行坐标与等待点匹配容差 (m)
    PoiMode             poi_mode_ = PoiMode::EXPLORE;
    std::deque<Vec2>    poi_queue_;                // 待飞插点队列（FIFO，当前优先）
    Vec2                poi_target_;               // 当前正在去/等的插点
    std::unordered_set<uint64_t> poi_seen_;        // 已收插点指纹（去重，仅 z=0）
    bool                release_pending_ = false;  // 收到 z=1 放行信号
    double              release_x_ = 0.0, release_y_ = 0.0;
    double              last_yaw_rate_ = 0.0;      // pd_to_point 算出的 yaw_rate 暂存

    // 估速度
    double prev_x_ = 0.0, prev_y_ = 0.0;
    double prev_yaw_ = 0.0;                       // 上一帧 yaw(差分算角速度)
    bool   has_prev_pose_ = false;
    double v_fwd_est_ = 0.0, v_lat_est_ = 0.0;
    double yaw_rate_est_ = 0.0;                   // yaw 差分+低通得到的角速度(rad/s)，给点云高角速度门控用
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExplorationNode>();

    if (node->viz_enabled()) {
        // spin 放子线程，主线程跑 OpenCV imshow（GUI 必须主线程）。
        // 用多线程 executor：POI 订阅在独立 reentrant 组，后台监听不被 timer 阻塞。
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(node);
        std::thread spin_thread([&exec]() { exec.spin(); });

        while (rclcpp::ok()) {
            if (!node->spin_viz_once()) break;   // ESC 退出；waitKey 自带节流
        }

        exec.cancel();
        if (spin_thread.joinable()) spin_thread.join();
    } else {
        // 无可视化：多线程 spin（同样让 POI 后台监听）
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(node);
        exec.spin();
    }

    rclcpp::shutdown();
    return 0;
}
