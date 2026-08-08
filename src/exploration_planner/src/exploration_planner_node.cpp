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
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
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
        gcfg_.min_x = dd("field_min_x", params::FIELD_MIN_X);
        gcfg_.min_y = dd("field_min_y", params::FIELD_MIN_Y);
        gcfg_.max_x = dd("field_max_x", params::FIELD_MAX_X);
        gcfg_.max_y = dd("field_max_y", params::FIELD_MAX_Y);
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
        gains_.k_curv          = dd("k_curv", params::K_CURV);
        gains_.lookahead       = dd("lookahead", params::LOOKAHEAD);
        gains_.endpoint_slow_r = dd("endpoint_slow_r", params::ENDPOINT_SLOW_R);
        gains_.kp_yaw          = dd("kp_yaw", params::KP_YAW);
        gains_.kd_yaw          = dd("kd_yaw", params::KD_YAW);
        gains_.max_yaw_rate    = dd("max_yaw_rate", params::MAX_YAW_RATE);
        gains_.kp_lat          = dd("kp_lat", params::KP_LAT);
        gains_.kd_lat          = dd("kd_lat", params::KD_LAT);
        gains_.max_v_lat       = dd("max_v_lat", params::MAX_V_LAT);
        gains_.heading_gate_rad =
            dd("heading_gate_deg", params::HEADING_GATE_DEG) * M_PI / 180.0;
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
        fcfg_.horizon        = dd("explore_horizon_m", params::EXPLORE_HORIZON_M);
        fcfg_.chain_gap      = dd("chain_gap_m",       params::CHAIN_GAP_M);
        fcfg_.band_width     = dd("band_width",        params::BAND_WIDTH);
        fcfg_.band_tol       = dd("band_tol",          params::BAND_TOL);
        fcfg_.along_bonus    = dd("along_bonus",       params::ALONG_BONUS);
        fcfg_.band_clear_cnt = static_cast<int>(std::lround(
                               dd("band_clear_cnt", static_cast<double>(params::BAND_CLEAR_CNT))));
        replan_period_ = dd("replan_period_s", params::REPLAN_PERIOD_S);
        replan_dev_    = dd("replan_dev_m",    params::REPLAN_DEV_M);
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
        ggcfg_.min_x = gcfg_.min_x; ggcfg_.min_y = gcfg_.min_y;
        ggcfg_.max_x = gcfg_.max_x; ggcfg_.max_y = gcfg_.max_y;
        ggcfg_.cell         = dd("global_cell",        params::GLOBAL_CELL);
        ggcfg_.robot_radius = robot_radius_;
        ggcfg_.inflate      = dd("global_margin",      params::GLOBAL_MARGIN);
        ggcfg_.wall_margin  = dd("global_wall_margin", params::GLOBAL_WALL_MARGIN);
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
        obs_map_ = std::make_unique<ObstacleMap>(ocfg_);
        if (viz_) viz_obj_ = std::make_unique<Visualizer>(gcfg_, 1000);  // 1000px：小格(0.05m≈5px)能看清

        // ---- ROS 接口 ----
        cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/exploration/cmd_vel", 10);

        rclcpp::QoS latched(1);
        latched.transient_local();
        finished_pub_ = create_publisher<std_msgs::msg::Bool>("/exploration/finished", latched);

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

        timer_ = create_wall_timer(
            std::chrono::milliseconds(params::TIMER_PERIOD_MS),
            std::bind(&ExplorationNode::on_timer, this));

        last_plan_time_ = now();
        last_global_plan_time_ = now();

        RCLCPP_INFO(get_logger(),
            "exploration_planner 已启动 (场地 %.1fx%.1fm, 大格 %.2fm, 小格 %.2fm, 动态前沿覆盖)",
            gcfg_.max_x - gcfg_.min_x, gcfg_.max_y - gcfg_.min_y,
            gcfg_.big_cell, gcfg_.small_cell);
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
                                       turning, turn_dir, unreach_valid, unreach_pos);
        cv::imshow("Exploration", img);
        const int key = cv::waitKey(params::VIZ_PERIOD_MS);
        if (key == 27 /*ESC*/) return false;
        return true;
    }

private:
    // ---------- 回调 ----------
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;

        double yaw = 0.0;
        const double qn = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (qn > 1e-9) {
            tf2::Quaternion tq(q.x, q.y, q.z, q.w);
            tf2::Matrix3x3 m(tq);
            double r, pi; m.getRPY(r, pi, yaw);
        }

        const rclcpp::Time stamp = msg->header.stamp;

        std::lock_guard<std::mutex> lk(mtx_);
        // 位置差分估速度（local 系），再转机体系给 tracker 当 D 项参考
        if (has_prev_pose_) {
            const double dt = (stamp - prev_stamp_).seconds();
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
            }
        }
        prev_x_ = p.x; prev_y_ = p.y; prev_yaw_ = yaw; prev_stamp_ = stamp; has_prev_pose_ = true;

        px_ = p.x; py_ = p.y; yaw_ = yaw;
        has_pose_ = true;
    }

    void on_goal(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        goal_ = {msg->point.x, msg->point.y};
        has_goal_ = true;
        plan_pending_ = true;   // 下一拍在有位姿时规划
        unreachable_.clear();            // 换终点=换任务：清空够不到黑名单，所有区重新给机会
        last_unreach_clear_cov_ = 0.0;
        turning_for_solution_ = false;   // 换任务：打断原地转身找解
        has_unreachable_marker_ = false; // 清红叉
        RCLCPP_INFO(get_logger(), "收到探索终点: (%.2f, %.2f)，待规划", goal_.x, goal_.y);
    }

    // POI（途中必经点/插点）回调。约定：
    //   point.x/y = SLAM 坐标；point.z = 标志位：0=发现插点(去飞)，1=放行(继续探索)。
    //   去重：仅对 z=0 的"插点触发"按 (x,y) 量化指纹去重，相同点只入队一次；
    //         z=1 永远当放行信号(不入队/不去重)，仅在 WAIT_RELEASE 且坐标匹配当前等待点时生效。
    //   主控可重复狂发，本回调天然幂等。
    void on_poi(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        const double x = msg->point.x, y = msg->point.y;
        const bool release = (msg->point.z >= 0.5);   // z=1 放行

        std::lock_guard<std::mutex> lk(mtx_);

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

    // 雷达点云回调（世界系 /cloud_registered）：滤地面 + 自身回波 → 投影到 2D
    //   → 喂给 obstacle_map 累计聚类。独立线程跑，不阻塞主循环。
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 取一份当前位姿(滤自身回波用 px,py；latch 回收判视野用 yaw) + 角速度(高角速度门控用)
        double px, py, yaw, wz; bool ok;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            px = px_; py = py_; yaw = yaw_; ok = has_pose_;
            wz = yaw_rate_est_;
        }

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
        pts.reserve(msg->width * msg->height / 2 + 1);

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
            const float x = *it_x, y = *it_y, z = *it_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            if (z < ground_z_) continue;                 // 高度窗口·下限：地面/近地杂物滤掉
            if (z > ceil_z_)   continue;                 // 高度窗口·上限：天花板/高处墙面/吊挂物滤掉
            if (ok) {
                const double dx = x - px, dy = y - py;
                if (dx * dx + dy * dy < self_r2) continue; // 自身回波：滤掉
            }
            pts.push_back({static_cast<double>(x), static_cast<double>(y)});
        }

        obs_map_->integrate(pts, px, py, yaw, ok);   // 传位姿：latch 智能回收判"格是否在当前视野内"
    }

    // 采纳一条探索绕障路径(replan 成功 与 转身重搜成功 共用)：设激活轨迹、清失败态、撤红叉。
    //   需在持有 mtx_ 时调用。
    void adopt_explore_path(const GlobalResult& gr, const Vec2& scan_target)
    {
        explore_raw_            = gr.path;                       // 裸折线(供下拍 path_clear 校验)
        traj_                   = smooth_catmull_rom(gr.path, arc_ds_);
        explore_failed_         = false;
        explore_has_committed_  = true;
        explore_target_         = scan_target;
        tracker_->set_trajectory(traj_);
        plan_pending_           = false;
        last_plan_time_         = now();
        has_unreachable_marker_ = false;                         // 又有路了 → 撤红叉
    }

    // 进入【原地转身找解】：锥内无解但无锥 probe 有解(=有路只是不在机头方向)时调用。
    //   转身方向取朝 probe 给出的"开口"方向(最短转到位)。需在持有 mtx_ 时调用。
    void enter_turn_for_solution(const Vec2& cur, const Vec2& scan_target, const GlobalResult& probe)
    {
        turning_for_solution_   = true;
        turn_target_            = scan_target;
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

        // 2) 节流重搜：用当前 yaw 加锥再搜，锥内出解→采纳、退出转身、本拍即交外层跟随
        if (++turn_research_tick_ >= turn_solve_research_every_) {
            turn_research_tick_ = 0;
            GlobalResult gr = plan_global_path(cur, turn_target_, obs, ggcfg_, yaw_);
            if (gr.ok && gr.path.size() >= 2) {
                turning_for_solution_ = false;
                adopt_explore_path(gr, turn_target_);
                RCLCPP_INFO(get_logger(),
                    "原地转身找解成功 → 机头方向出现可走路径，沿新方向继续 (目标 %.2f,%.2f)",
                    turn_target_.x, turn_target_.y);
                return false;
            }
        }

        // 3) 转满一圈 / 超时 → 目标真被围死：红叉 + 拉黑 + 跳带，本拍悬停
        const bool full_circle = std::fabs(turn_accum_) >= 2.0 * M_PI * turn_solve_max_rev_;
        const bool timed_out   = (now() - turn_start_time_).seconds() > turn_solve_timeout_;
        if (full_circle || timed_out) {
            turning_for_solution_   = false;
            unreachable_.push_back(turn_target_);
            cur_band_++;
            plan_pending_           = true;
            has_unreachable_marker_ = true;
            unreachable_pos_        = turn_target_;
            cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
            RCLCPP_WARN(get_logger(),
                "原地转身找解：转满一圈仍无解 → 目标 (%.2f,%.2f) 真被围死，画红叉 + 跳带去别处",
                turn_target_.x, turn_target_.y);
            return true;
        }

        // 4) 否则继续原地转：前进/横向清零，只发 yaw_rate(画旋转标志)
        cmd.twist.linear.x  = 0.0;
        cmd.twist.linear.y  = 0.0;
        cmd.twist.angular.z = turn_dir_ * turn_solve_yaw_rate_;
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
        bool all_explored = false;

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

        Path2 wp = plan_explore(gsnap, fcfg_, cur, yaw_, cur_band_, all_explored,
                                &unreachable_, unreach_block_r_);

        // 挑覆盖路径上第一个距当前 ≥ explore_target_min_dist_ 的点作 A* 终点：
        //   太近绕行无意义、易频繁过期；只需绕近处障碍，到了自然周期重规划接力。
        Vec2 scan_target = wp.empty() ? cur : wp.back();
        for (size_t i = 1; i < wp.size(); ++i) {
            if (std::hypot(wp[i].x - cur.x, wp[i].y - cur.y) >= explore_target_min_dist_) {
                scan_target = wp[i];
                break;
            }
        }

        const Obstacles obs = obs_map_->snapshot();

        // ★路径承诺/迟滞★：已有绕障折线时，先判旧折线是否仍无碰撞、目标是否漂移。
        //   仅在【目标大幅移动(换区)】或【旧折线被挡(会撞)】时才考虑重算 A*。
        const bool have_commit  = tracker_->has_trajectory() && explore_has_committed_ && !explore_failed_;
        const bool old_clear    = have_commit && path_clear(cur, explore_raw_, obs, ggcfg_);
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
        const bool target_stale = have_commit &&
            !gsnap.has_gain_within(explore_target_, target_stale_r_);

        // 目标几乎没动 且 旧折线仍无碰撞 且 目标仍有收益 → 直接续用，连 A* 都不搜(最省、最稳，绝不翻边)。
        if (have_commit && !target_moved && old_clear && !target_stale) {
            plan_pending_ = false;
            last_plan_time_ = now();   // 续命，避免下一拍又因 age 触发重搜
            return;
        }
        if (target_stale) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1500,
                "目标失效(邻域 %.1fm 内已无未扫格) (%.2f,%.2f) → 放弃承诺，改投 (%.2f,%.2f)",
                target_stale_r_, explore_target_.x, explore_target_.y, scan_target.x, scan_target.y);
        }

        GlobalResult gr = plan_global_path(cur, scan_target, obs, ggcfg_, yaw_);  // ★机头锥★：起点段只朝机头延伸
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
                const PathScore so = score_path(cur, explore_raw_, obs, ggcfg_);   // 旧路(剩余段)
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
            adopt_explore_path(gr, scan_target);   // 设激活轨迹/清失败态/撤红叉
        } else {
            // ★机头锥内无解★：先用【无锥 probe】判定到底是"任何朝向都没路"还是"有路只是不在机头方向"。
            explore_failed_ = true;
            explore_has_committed_ = false;
            traj_ = Trajectory{};
            tracker_->set_trajectory(traj_);

            GlobalResult probe = plan_global_path(cur, scan_target, obs, ggcfg_);  // 不传 yaw → 无锥
            if (probe.ok && probe.path.size() >= 2) {
                // 有路，只是不在机头方向 → 原地转身找解(改朝向重搜)，不后退/不跳带。
                enter_turn_for_solution(cur, scan_target, probe);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "机头方向无路但侧向有路 → 原地转身找解 (目标 %.2f,%.2f)", scan_target.x, scan_target.y);
            } else {
                // 任何朝向都没路 → 走原有 retreat(贴脸脱困) / 跳带(真被围死) 逻辑。
                const double od = nearest_obstacle_dist(obs);
                const bool can_retreat = (od < retreat_trigger_) &&
                    (!retreating_ || std::hypot(px_ - retreat_origin_.x, py_ - retreat_origin_.y) < retreat_max_dist_);
                if (can_retreat) {
                    // 贴障且未退够 → 不跳带/不拉黑，交主循环后退脱困，退出去下拍重规划接着绕。
                    plan_pending_ = true;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "探索目标 (%.2f, %.2f) A* 无解且贴障 → 后退脱困重试", scan_target.x, scan_target.y);
                } else {
                    // 不是贴脸卡死/已退到极限仍无路 = 真够不到 → 放弃该区：拉黑 + 跳带 + 红叉 + 去别处。
                    retreating_ = false;
                    unreachable_.push_back(scan_target);   // 拉黑够不到的目标：选点不再回头试这条死路
                    cur_band_++;
                    plan_pending_ = true;
                    has_unreachable_marker_ = true;        // ★画红叉★
                    unreachable_pos_ = scan_target;
                    RCLCPP_WARN(get_logger(),
                        "探索目标 (%.2f, %.2f) A* 无解(任何朝向都被围死) → 画红叉 + 放弃该区域跳带去别处",
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
            yaw_rate = std::clamp(gains_.kp_yaw * e_yaw, -gains_.max_yaw_rate, gains_.max_yaw_rate);
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
            const double head_gate =
                (std::fabs(e_yaw) > gains_.heading_gate_rad) ? 0.0 : std::max(0.0, std::cos(e_yaw));
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

    // 沿全局绕障轨迹 gtraj 用 carrot 跟随：方向取轨迹上前瞻 global_lookahead_ 的点，
    //   速度按到 final_target 距离刹停。逼近真目标(剩余≤前瞻)时直接朝真目标 → 精确停。
    //   写 last_look_=carrot。返回机体系 {v_fwd, v_lat}（yaw_rate 进 last_yaw_rate_）。
    Vec2 pd_follow_path(const Trajectory& gtraj, const Vec2& final_target)
    {
        const double d_goal = std::hypot(final_target.x - px_, final_target.y - py_);
        Vec2 carrot = final_target;
        if (gtraj.size() >= 2 && d_goal > global_lookahead_) {
            // 轨迹上离飞机最近的点
            size_t ni = 0; double best = 1e18;
            for (size_t i = 0; i < gtraj.size(); ++i) {
                const double dd2 = std::hypot(gtraj[i].p.x - px_, gtraj[i].p.y - py_);
                if (dd2 < best) { best = dd2; ni = i; }
            }
            // 沿弧长向前取 global_lookahead_ 作 carrot
            const double s_target = gtraj[ni].s + global_lookahead_;
            size_t ci = gtraj.size() - 1;
            for (size_t i = ni; i < gtraj.size(); ++i)
                if (gtraj[i].s >= s_target) { ci = i; break; }
            carrot = gtraj[ci].p;
        }
        last_look_ = carrot; look_valid_ = true;
        return pd_core(carrot, d_goal);
    }

    // 全局绕障直奔 target：A*(承诺式)搜绕障折线 → Catmull-Rom 平滑 → 沿线 carrot PD 跟随精确刹停。
    //   ★路径承诺/迟滞★：一旦采纳一条绕障路径(选了从障碍某侧绕过)，只要它在当前障碍图下【仍无碰撞】
    //   就续用，绝不因左右绕代价 near-tie 而每次重搜翻边(根因:A* 无记忆,飞机/点云/量化微抖→最短路翻边
    //   →carrot 甩到另一侧→横切撞柱)。仅在【目标大幅移动】或【旧路径真被挡(会撞)】时才重算 A* 换边。
    //   写 traj_(可视化激活轨迹) 与 last_look_；A* 无解(目标被围死)时置 global_failed_=true，
    //   供调用方报警悬停(POI/终点必达，无解属异常，不乱撞)。
    //   需在持有 mtx_ 时调用（读写 px_/py_/global_* 等）。
    Vec2 pd_to_point_avoid(const Vec2& target, const Obstacles& obs)
    {
        const Vec2 cur{px_, py_};
        const bool target_moved =
            std::hypot(target.x - global_target_.x, target.y - global_target_.y) > commit_target_tol_;
        // 旧路径仍无碰撞？(只查障碍圆，与 A* 自洽) —— 无缓存/目标动则无需校验、直接重算
        const bool stale = !global_has_ || target_moved ||
                           !path_clear(cur, global_raw_, obs, ggcfg_);
        if (stale) {
            // ★机头锥优先★：先约束起点段朝机头延伸搜(让 POI/归航起步也偏好机头方向，不侧移撞柱)；
            //   锥内无解则回退【无锥】再搜一次(与今日行为完全一致，零回归)。POI/归航必达，不在此走转身。
            GlobalResult gr = plan_global_path(cur, target, obs, ggcfg_, yaw_);
            if (!(gr.ok && gr.path.size() >= 2))
                gr = plan_global_path(cur, target, obs, ggcfg_);   // 回退：无锥(NaN)
            if (gr.ok && gr.path.size() >= 2) {
                global_raw_    = gr.path;                       // 裸折线(供下拍 path_clear 校验)
                global_traj_   = smooth_catmull_rom(gr.path, arc_ds_);
                global_failed_ = false;
            } else {
                // 目标被围死/不连通 → 直线兜底(仅可视化)，并置 global_failed_ 让调用方报警悬停
                global_raw_    = Path2{ cur, target };
                global_traj_   = smooth_catmull_rom(global_raw_, arc_ds_);
                global_failed_ = true;
            }
            global_target_ = target;
            global_has_    = true;
            last_global_plan_time_ = now();
        }
        traj_ = global_traj_;   // 可视化：把当前激活的绕障轨迹画出来
        return pd_follow_path(global_traj_, target);
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

        if (!retreating_) { retreating_ = true; retreat_origin_ = {px_, py_}; }

        const Vec2 back{ px_ + ux * retreat_step_, py_ + uy * retreat_step_ };
        traj_ = smooth_catmull_rom(Path2{ {px_, py_}, back }, arc_ds_);   // 可视化:后退直线
        last_look_ = back; look_valid_ = true;

        Vec2 vb = pd_core(back, retreat_step_);   // 朝后退点；dist=step 使其全程出力(不提前刹停)
        // 限后退速度：等比缩放(pd_core 已给机体系 vx/vy)
        const double sp = std::hypot(vb.x, vb.y);
        if (sp > retreat_v_max_ && sp > 1e-6) { const double k = retreat_v_max_ / sp; vb.x *= k; vb.y *= k; }
        return vb;
    }

    // 统一处理"A* 到 target 无解"：贴障且未退够 → 后退脱困(返回 true=本拍已写 cmd 后退速度)；
    //   否则(不是贴脸卡死/已退到极限仍无路=真被围死) → 返回 false 交调用方走原放弃/悬停逻辑。
    //   need_obstacle_near：是否要求"贴障才退"。探索/POI/归航都传 true(只救贴脸卡死，真围死不乱退)。
    bool try_retreat(const Obstacles& obs, geometry_msgs::msg::TwistStamped& cmd)
    {
        const double od = nearest_obstacle_dist(obs);
        const double traveled = retreating_
            ? std::hypot(px_ - retreat_origin_.x, py_ - retreat_origin_.y) : 0.0;
        if (od < retreat_trigger_ && traveled < retreat_max_dist_) {
            const Vec2 vb = do_retreat(obs);
            cmd.twist.linear.x = vb.x;
            cmd.twist.linear.y = vb.y;
            cmd.twist.angular.z = last_yaw_rate_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "A* 无解且贴障(边距 %.2fm) → 后退脱困(已退 %.2fm)", od, traveled);
            return true;
        }
        retreating_ = false;   // 不满足后退条件 → 复位，交原逻辑
        return false;
    }

    // ---------- 20Hz 主循环 ----------
    void on_timer()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = now();
        cmd.header.frame_id = "base_link";   // 机体系

        bool publish_finished = false;

        {
            std::lock_guard<std::mutex> lk(mtx_);

            if (!has_pose_) {
                // 无位姿：发零速度占位
                cmd_pub_->publish(cmd);
                return;
            }

            // 取一份当前障碍圆（DWA 避障 + 视野遮挡 + 可视化共用）
            const Obstacles obstacles = obs_map_->snapshot();

            // 把当前视野标进栅格（障碍背后被遮挡的格不标，模拟摄像头）
            grid_->mark_scan(px_, py_, yaw_, obstacles);
            // 障碍圆覆盖的小格直接算已扫(实体内部永远看不到，否则覆盖率永远到不了 100%)。永久保留。
            grid_->fill_obstacle_cells(obstacles);

            // ============================================================
            // POI（途中必经点/插点）状态机 —— 优先级高于探索与归航判定。
            //   EXPLORE      : 无插点活动，走下面常规探索/归航。
            //   GOTO_POI     : 飞向当前插点(走 tracker)，到点 → 转 WAIT_RELEASE。
            //   WAIT_RELEASE : 到点悬停，本算法只发零速，把飞机交给主控外部数据流；
            //                  收到同点 z=1 放行 → 弹队、回 EXPLORE 继续探索。
            // 多插点：排队当前优先；放行后弹下一个。
            // ============================================================
            bool poi_active = false;

            // 空闲(EXPLORE)且队列有插点 → 取队首开始去飞（归航后不再接插点）
            if (poi_mode_ == PoiMode::EXPLORE && !homing_ && !poi_queue_.empty()) {
                poi_target_ = poi_queue_.front();
                poi_mode_ = PoiMode::GOTO_POI;
                turning_for_solution_ = false;   // 切去插点：打断探索的原地转身找解
                RCLCPP_INFO(get_logger(), "前往插点 (%.2f, %.2f)", poi_target_.x, poi_target_.y);
            }

            if (poi_mode_ == PoiMode::GOTO_POI) {
                poi_active = true;
                // 去飞插点(目标可能藏在障碍后)：全局 A* 绕障 → 沿绕障轨迹 carrot PD 跟随、精确停，不走 tracker。
                const Vec2 vb = pd_to_point_avoid(poi_target_, obstacles);  // 内部写 traj_/last_look_
                if (global_failed_) {
                    // A* 无解(插点被围死)：先试贴障后退脱困；退出去通道打开→下拍自然重算成功接着绕。
                    if (!try_retreat(obstacles, cmd)) {
                        // 非贴脸卡死/已退到极限仍无路=真被围死 → 插点必达，报警悬停(无 DWA 兜底)。
                        cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "插点 (%.2f, %.2f) A* 无解(被围死)，悬停等待——请检查障碍/场地参数",
                            poi_target_.x, poi_target_.y);
                    }
                } else {
                    retreating_ = false;   // A* 又通了 → 退出后退态
                    cmd.twist.linear.x  = vb.x;
                    cmd.twist.linear.y  = vb.y;
                    cmd.twist.angular.z = last_yaw_rate_;
                }
                const double d = std::hypot(poi_target_.x - px_, poi_target_.y - py_);
                if (d <= goal_tol_) {
                    poi_mode_ = PoiMode::WAIT_RELEASE;
                    release_pending_ = false;   // 清掉到点前可能误收的放行
                    cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
                    RCLCPP_INFO(get_logger(),
                        "已到插点 (%.2f, %.2f)，悬停等待主控放行(z=1)", poi_target_.x, poi_target_.y);
                }
            } else if (poi_mode_ == PoiMode::WAIT_RELEASE) {
                poi_active = true;
                // 等待期：本算法只发零速悬停，控制权交主控外部数据流
                cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
                last_look_ = poi_target_; look_valid_ = true;
                // 放行：z=1 且坐标匹配当前等待点(±POI_SAME_TOL) → 弹队、回探索
                const bool match = release_pending_ &&
                    std::hypot(release_x_ - poi_target_.x, release_y_ - poi_target_.y) <= POI_SAME_TOL;
                if (match) {
                    release_pending_ = false;
                    if (!poi_queue_.empty()) poi_queue_.pop_front();
                    poi_mode_ = PoiMode::EXPLORE;
                    plan_pending_ = true;   // 回探索后立刻重规划接着扫
                    RCLCPP_INFO(get_logger(), "收到放行 → 继续探索");
                }
            }

            // 完程度(覆盖率)达标 → 进入归航刹停阶段（一旦进入不再回退）。
            // 注意：仅在没有插点活动时才判定/接管，避免插点途中误触发归航。
            //   覆盖率只读一次并复用：判定与日志同源，日志里印的就是真正触发归航的那个值。
            if (!poi_active && has_goal_ && !homing_) {
                const double cov = grid_->coverage_ratio();
                if (cov >= done_coverage_) {
                    homing_ = true;
                    turning_for_solution_ = false;            // 转归航：打断探索的原地转身找解
                    tracker_->set_trajectory(Trajectory{});   // 丢弃探索轨迹，归航不走 tracker
                    RCLCPP_INFO(get_logger(),
                        "完程度达标(%.0f%%) → 归航，朝终点 (%.2f, %.2f) PD 刹停",
                        cov * 100.0, goal_.x, goal_.y);
                }
            }

            if (poi_active) {
                // 插点活动中：命令已在上面 POI 状态机里写好，这里不再覆盖。
            } else if (has_goal_ && homing_) {
                // ---- 归航：直奔真实终点(不夹取)，平滑刹停。终点可能藏在障碍后 → 全局 A* 绕障。 ----
                Vec2 vb = pd_to_point_avoid(goal_, obstacles);   // 内部写 traj_/last_look_
                if (global_failed_) {
                    // A* 无解(终点被围死)：先试贴障后退脱困；退出去通道打开→下拍自然重算成功接着绕。
                    if (!try_retreat(obstacles, cmd)) {
                        // 非贴脸卡死/已退到极限仍无路=真被围死 → 终点必达，报警悬停(无 DWA 兜底)。
                        cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "归航终点 (%.2f, %.2f) A* 无解(被围死)，悬停等待——请检查障碍/场地参数",
                            goal_.x, goal_.y);
                    }
                } else {
                    retreating_ = false;   // A* 又通了 → 退出后退态
                    cmd.twist.linear.x  = vb.x;
                    cmd.twist.linear.y  = vb.y;
                    cmd.twist.angular.z = last_yaw_rate_;
                }

                // 停稳判定：到点容差内且合速度足够小（停稳优先，直接清零）
                const double d = std::hypot(goal_.x - px_, goal_.y - py_);
                const double v_now = std::hypot(v_fwd_est_, v_lat_est_);
                const bool stopped = (d <= goal_tol_) && (v_now <= goal_stop_v_);
                if (stopped) {
                    cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
                    if (!finished_) { finished_ = true; publish_finished = true; }
                }
            } else {
                // ---- 探索：周期/偏离重规划(A* 绕障重铺覆盖路径) + tracker 跟随 ----
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
                                else if (tracker_->last_nearest_dist() > replan_dev_) need = true;
                            }
                        }
                        if (need) replan_locked({px_, py_});
                    }

                    // tracker 跟随当前 traj_(已是 A* 绕障轨迹)。A* 无解时 traj_ 被清空。
                    if (tracker_->has_trajectory()) {
                        retreating_ = false;   // 有轨迹可走 → 退出后退态
                        VelCmd vc = tracker_->update(px_, py_, yaw_, v_fwd_est_, v_lat_est_, goal_tol_);
                        last_look_ = tracker_->last_lookahead();
                        look_valid_ = true;
                        cmd.twist.linear.x  = vc.v_fwd;
                        cmd.twist.linear.y  = vc.v_lat;
                        cmd.twist.angular.z = vc.yaw_rate;
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

        }

        cmd_pub_->publish(cmd);

        // rviz 可视化：处理后障碍点云(每拍) + 设定边界框(首拍一次)。都在锁外，不阻塞主循环。
        publish_obstacle_cloud();
        if (!boundary_sent_) { publish_field_boundary(); boundary_sent_ = true; }

        if (publish_finished) {
            std_msgs::msg::Bool b; b.data = true;
            finished_pub_->publish(b);
            RCLCPP_INFO(get_logger(), "归航完成：已在终点停稳 → finished=true");
        }
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
    TrackerGains  gains_;
    FrontierConfig fcfg_;
    double        lane_spacing_;       // 旧牛耕车道间距：已不参与任何计算(见构造里说明)，保留仅为参数表兼容
    double        arc_ds_;
    double        done_coverage_, goal_tol_, v_est_alpha_;
    double        goal_stop_v_, kp_goal_, kd_goal_, v_goal_max_;
    double        replan_period_ = 0.7, replan_dev_ = 0.5;
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
    std::unique_ptr<Visualizer>        viz_obj_;
    std::unique_ptr<ObstacleMap>       obs_map_;

    // ---- ROS ----
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              finished_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    obs_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  boundary_pub_;
    bool boundary_sent_ = false;   // 边界框只需发一次(latched)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr poi_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_sub_;
    rclcpp::CallbackGroup::SharedPtr poi_cbg_;
    rclcpp::CallbackGroup::SharedPtr cloud_cbg_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- 共享状态（mtx_ 保护）----
    std::mutex mtx_;
    double px_ = 0.0, py_ = 0.0, yaw_ = 0.0;
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

    // ---- 探索 A* 重铺运行态（mtx_ 保护）----
    bool         explore_failed_ = false;  // 上次探索 A* 无解(目标被围死) → 放弃该区跳带
    Path2        explore_raw_;             // 上次采纳的探索绕障裸折线(供 path_clear 承诺校验)
    bool         explore_has_committed_ = false;  // 已采纳一条探索绕障路径(承诺中)
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
    rclcpp::Time last_global_plan_time_;  // 上次 A* 时刻（节流，构造里 now() 初始化）

    // ---- 脱困后退（mtx_ 保护）：A* 无解且贴障时低速退出来，边退边重算 ----
    bool   retreating_ = false;        // 正在后退脱困
    Vec2   retreat_origin_;            // 本轮后退起点（算累计后退距离，超 retreat_max_dist_ 放弃）
    double retreat_trigger_  = 0.55;   // 触发后退的贴障边距(m)，构造由 RETREAT_TRIGGER_M 覆盖（-p 可调）
    double retreat_step_     = 0.50;   // 每次后退目标点距离(m)，由 RETREAT_STEP_M 覆盖
    double retreat_max_dist_ = 1.20;   // 累计后退上限(m)，由 RETREAT_MAX_DIST 覆盖
    double retreat_v_max_    = 0.35;   // 后退限速(m/s)，由 RETREAT_V_MAX 覆盖

    // ---- 原地转身找解（mtx_ 保护）：探索锥内无解但有路(只是不在机头方向)→原地转身改朝向重搜 ----
    bool          turning_for_solution_ = false;  // 正在原地转身找解
    Vec2          turn_target_;                   // 转身期间重搜用的目标(=触发时的 scan_target)
    int           turn_dir_ = 1;                  // 转身扫向：+1=逆时针(CCW)/-1=顺时针(CW)，朝 probe 开口方向
    double        turn_prev_yaw_ = 0.0;           // 上一拍 yaw（算本拍增量）
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
    rclcpp::Time prev_stamp_;
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
