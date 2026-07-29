#include "fly_mission/drone_controller.hpp"
#include "fly_mission/params.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <cstdio>
#include <algorithm>

namespace fly_mission {

// 把任意角度归一化到 [-π, π]
static double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// 把绝对值限到 limit 以内（保持符号）
static double clamp_abs(double v, double limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

// ============================================================================
//   构造：订阅 MAVROS state + 雷达 odom，发布 setpoint_raw
// ============================================================================
DroneController::DroneController(rclcpp::Node* node)
    : node_(node)
{
    state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
            current_state_ = *msg;
        });

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_pose_.header = msg->header;
            current_pose_.pose   = msg->pose.pose;

            // 位置差分估算速度，结果给 PD 的 D 项用
            // dt 异常或第一帧时跳过更新，保留 v_est 上一拍
            const rclcpp::Time now = node_->now();
            if (prev_pose_valid_) {
                const double dt = (now - prev_pose_time_).seconds();
                if (dt > 1e-3 && dt < 0.5) {
                    const double inst_vx = (msg->pose.pose.position.x - prev_pos_x_) / dt;
                    const double inst_vy = (msg->pose.pose.position.y - prev_pos_y_) / dt;
                    const double inst_vz = (msg->pose.pose.position.z - prev_pos_z_) / dt;
                    const double a = params::V_EST_ALPHA;
                    v_est_x_ = a * inst_vx + (1.0 - a) * v_est_x_;
                    v_est_y_ = a * inst_vy + (1.0 - a) * v_est_y_;
                    v_est_z_ = a * inst_vz + (1.0 - a) * v_est_z_;
                }
            }
            prev_pos_x_     = msg->pose.pose.position.x;
            prev_pos_y_     = msg->pose.pose.position.y;
            prev_pos_z_     = msg->pose.pose.position.z;
            prev_pose_time_ = now;
            prev_pose_valid_ = true;

            has_pose_.store(true);

            // 位姿信箱B(主控→视觉)：雷达来一条写一条(20Hz,写一次亚微秒)。视觉端
            //   直读代替订阅 odom(第4步去 rclpy)。yaw 用 current_yaw() 同款公式，
            //   但只算 yaw 不建矩阵(回调里省一点)。
            pose_shm_.write(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z,
                            [&q = msg->pose.pose.orientation] {
                                const double siny = 2.0 * (q.w * q.z + q.x * q.y);
                                const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                                return std::atan2(siny, cosy);
                            }());
        });

    setpoint_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);

    set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arming_client_   = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    // 绕杆环绕：订阅杆检测(仅 description_circle_right 的 COLLECT 阶段真正攒帧)。
    //   契约见 pole_detector：Float64MultiArray data=[x, y, z, yaw_err_deg]，x,y=杆水平轴心。
    //   ★注意★：这让运动库订阅了感知话题(用户明确要求"直接改运动库")，是本类唯一的感知耦合。
    pole_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pole_detector/center", 10,
        std::bind(&DroneController::on_pole_center, this, std::placeholders::_1));

    // 全局避障：订阅多杆检测(整帧覆盖障碍)。独立 Reentrant 组，后台解析不阻塞 20Hz timer。
    //   契约 pole 侧 multi_pole_detector：data=[id,x,y,z,yaw_err_deg,radius]×N，camera_init 系(=current_pose_ 同系)。
    obstacle_cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions obs_opt;
    obs_opt.callback_group = obstacle_cbg_;
    multi_pole_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/multi_pole_detector/center", rclcpp::QoS(10),
        std::bind(&DroneController::on_multi_pole_center, this, std::placeholders::_1), obs_opt);
}

// 仅 COLLECT 阶段攒 杆心x,y + 半径 样本(后台线程写，pole_mtx_ 保护)。
//   契约见 pole_detector：data=[x, y, z, yaw_err_deg, radius]，需 ≥5 个。
void DroneController::on_pole_center(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 5) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[绕杆] /pole_detector/center 只有 %zu 个数(需≥5含 radius)，本条忽略", msg->data.size());
        return;
    }
    std::lock_guard<std::mutex> lk(pole_mtx_);
    if (!pole_collecting_) return;
    pole_sx_.push_back(msg->data[0]);
    pole_sy_.push_back(msg->data[1]);
    pole_sr_.push_back(msg->data[4]);   // 拟合杆半径
}

// ============================================================================
//   全局避障：多杆检测回调 + 每拍算"有效目标"
// ============================================================================

// 整帧覆盖障碍列表。契约 data=[id,x,y,z,yaw_err_deg,radius]×N(每 6 个一根)。后台线程写。
void DroneController::on_multi_pole_center(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    constexpr size_t STRIDE = 6;
    std::vector<avoid::CircleObstacle> parsed;
    if (msg->data.size() >= STRIDE && (msg->data.size() % STRIDE) == 0) {
        parsed.reserve(msg->data.size() / STRIDE);
        for (size_t i = 0; i + STRIDE <= msg->data.size(); i += STRIDE) {
            avoid::CircleObstacle o;
            o.id     = static_cast<int>(msg->data[i]);
            o.x      = msg->data[i + 1];
            o.y      = msg->data[i + 2];
            o.radius = msg->data[i + 5];
            if (!std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(o.radius)) continue;
            if (o.radius <= 0.0 || o.radius > params::AVOID_MAX_RADIUS) continue;   // 坏拟合/非细杆丢弃
            parsed.push_back(o);
        }
    }
    std::lock_guard<std::mutex> lk(obstacles_mtx_);
    obstacles_.swap(parsed);                    // 最新帧整帧覆盖(检测端每帧只发当前可见杆)
    obstacles_stamp_ = node_->now();
    obstacles_valid_ = true;
}

// 每拍(tick 里，PD 之前)算 eff_gx_/eff_gy_：默认=target_x_/y_；避障触发时=贝塞尔前瞻点。
//   ★不改 target_x_/y_★ → is_reached()/各原语语义不变。开关关时=直线，行为逐位一致。
void DroneController::update_effective_goal()
{
    eff_gx_ = target_x_;  eff_gy_ = target_y_;  avoiding_ = false;   // 默认走直线(零行为改变)

    if (!params::AVOID_ENABLE) return;                       // ★总开关★(关闭时彻底静默)

    // 以下均带诊断日志(节流1s)，方便定位"为什么没避"。
    auto diag = [this](const char* why) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[避障诊断] 不避障：%s", why);
    };

    if (action_mode_ == ActionMode::CIRCLE) { diag("当前是CIRCLE(排除)"); return; }
    if (!has_pose_.load())                   { diag("无雷达位姿"); return; }

    std::vector<avoid::CircleObstacle> snap;                 // 锁内只拷贝，锁外算
    {
        std::lock_guard<std::mutex> lk(obstacles_mtx_);
        if (!obstacles_valid_) { diag("从未收到 /multi_pole_detector/center"); return; }
        if (obstacles_.empty()) { diag("收到了但障碍列表为空(可能radius全被过滤/本帧无杆)"); return; }
        const double age = (node_->now() - obstacles_stamp_).seconds();
        if (age > params::AVOID_OBSTACLE_TTL_S) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "[避障诊断] 不避障：障碍数据过期 %.2fs>%.2fs(检测停发?)", age, params::AVOID_OBSTACLE_TTL_S);
            return;
        }
        snap = obstacles_;
    }

    avoid::AvoidCfg cfg;
    cfg.safety_margin = params::AVOID_SAFETY_MARGIN_M;
    cfg.clearance     = params::AVOID_CLEARANCE_M;
    cfg.lookahead     = params::AVOID_LOOKAHEAD_M;
    cfg.max_radius    = params::AVOID_MAX_RADIUS;
    cfg.trigger_dist  = params::AVOID_TRIGGER_DIST_M;
    cfg.rejoin_ahead  = params::AVOID_REJOIN_AHEAD_M;
    cfg.samples       = params::AVOID_BEZIER_SAMPLES;

    const avoid::PlanResult r = avoid::plan_lookahead(
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        target_x_, target_y_, snap, cfg);

    if (r.avoiding && std::isfinite(r.gx) && std::isfinite(r.gy)) {
        eff_gx_ = r.gx;  eff_gy_ = r.gy;  avoiding_ = true;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[避障] 绕行杆 id=%d → 前瞻点(%.2f,%.2f)  当前(%.2f,%.2f)→目标(%.2f,%.2f)",
            r.obstacle_id, eff_gx_, eff_gy_,
            current_pose_.pose.position.x, current_pose_.pose.position.y, target_x_, target_y_);
    } else {
        // 收到了障碍但没挡路：把"最近的杆离航线多远 / 膨胀半径多少"打出来——
        //   若 nearest_dist ≥ nearest_inflate，说明杆不在直线通道上(调大 AVOID_SAFETY_MARGIN_M 或杆没真挡路)。
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[避障诊断] 收到%d杆(有效%d) 但判定不挡路：最近杆id=%d 离航线%.2fm 膨胀半径%.2fm(需<才避) "
            "当前(%.2f,%.2f)→目标(%.2f,%.2f)",
            r.n_obstacles, r.n_valid, r.nearest_id, r.nearest_dist, r.nearest_inflate,
            current_pose_.pose.position.x, current_pose_.pose.position.y, target_x_, target_y_);
    }
}

// 航点被杆占的放宽到达(见头文件说明)。返回 true=判定"已尽力到达此航点，可推进下一点"。
bool DroneController::waypoint_blocked_arrived(double wx, double wy) const
{
    if (!params::AVOID_ENABLE) return false;     // 关避障时不改变原到达语义
    if (!has_pose_.load()) return false;

    const double cx = current_pose_.pose.position.x;
    const double cy = current_pose_.pose.position.y;
    std::lock_guard<std::mutex> lk(obstacles_mtx_);
    if (!obstacles_valid_ || obstacles_.empty()) return false;
    if ((node_->now() - obstacles_stamp_).seconds() > params::AVOID_OBSTACLE_TTL_S) return false;

    for (const auto& o : obstacles_) {
        const double inflate = o.radius + params::AVOID_SAFETY_MARGIN_M;
        // ① 航点确实被这根杆占(落在膨胀圈内 → PD 永远进不了 TOL_XY)
        if (std::hypot(wx - o.x, wy - o.y) >= inflate) continue;
        // ② 飞机已接近到"杆边缘 + slack"内(尽力靠近了)→ 判定到达
        const double d_drone = std::hypot(cx - o.x, cy - o.y);
        if (d_drone <= inflate + params::AVOID_ARRIVE_SLACK_M) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "[避障] 航点(%.2f,%.2f)被杆id=%d占，已贴到杆边(%.2fm)→ 判定到达、推进下一点",
                wx, wy, o.id, d_drone);
            return true;
        }
    }
    return false;
}


void DroneController::tick()
{
    // 没有雷达数据 → 不发(OFFBOARD_PREHEAT=true 时才发零速度占位)
    //   没位姿本就飞不了：BOOT_CHECK 要求 has_pose_ 才往下判 OFFBOARD，
    //   所以这里不发不影响正常起飞流程。
    if (!has_pose_.load()) {
        if (params::OFFBOARD_PREHEAT) publish_setpoint(0.0, 0.0, 0.0, 0.0);
        return;
    }

    // IDLE 模式（任务未启动 / 已停止）：★不发任何 setpoint★
    //   按需求(2026-07-28)去掉"OFFBOARD 前的零速度占位预热"——雷达就绪 + 飞手手动切到
    //   OFFBOARD 后，BOOT_CHECK 当拍就走到 takeoff()，从那一拍起才开始发数据。
    //   ★这意味着切 OFFBOARD 那一刻本程序还没发过任何 setpoint★：
    //     · 若飞控要求"切 OFFBOARD 前必须已有 setpoint 流"，它会拒绝切换/立刻退出；
    //       表现为切不进 OFFBOARD 或刚进就掉出来。真出现这种情况把
    //       params::OFFBOARD_PREHEAT 改回 true 即可恢复占位流。
    //     · 任务中止(stop()→IDLE)后同样不再发 → 飞控按其失联保护动作(通常自行降落/
    //       保持)，本程序不再干预。
    if (action_mode_ == ActionMode::IDLE) {
        if (params::OFFBOARD_PREHEAT) {
            // 预热流(可选)：零速度占位，语义=原地不动，仅为满足飞控对 setpoint 流的要求
            publish_setpoint(0.0, 0.0, 0.0, 0.0);
        }
        return;
    }

    // LAND 模式只切一次 AUTO.LAND，之后等飞控自己降下来
    if (action_mode_ == ActionMode::LAND) {
        if (!land_requested_) {
            land_requested_ = true;
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "AUTO.LAND";
            set_mode_client_->async_send_request(req);
            RCLCPP_INFO(node_->get_logger(), "[降落] 已请求 AUTO.LAND");
        }
        log_progress();
        return;
    }

    // EXTERNAL_VEL：探索算法发来机体系速度，旋转到 local(NED) 系发布；带看门狗保高
    if (action_mode_ == ActionMode::EXTERNAL_VEL) {
        const double cz = current_pose_.pose.position.z;
        // 高度保持：PD 把高度锁回 explore_z_，限到平飞档微调速度
        const double vz = clamp_abs(
            params::KP_Z * (explore_z_ - cz) - params::KD_Z * v_est_z_,
            params::MAX_SPEED_Z_LEVEL);

        // 看门狗：>0.3s 没有新速度命令 → 水平/yaw 清零，只保高悬停
        const double age = ext_valid_ ? (node_->now() - ext_cmd_time_).seconds() : 1e9;
        if (age > 0.3) {
            publish_setpoint(0.0, 0.0, vz, 0.0);
        } else {
            // 机体系 → local：vx=c*v_fwd-s*v_lat, vy=s*v_fwd+c*v_lat
            const double yaw = current_yaw();
            const double c = std::cos(yaw);
            const double s = std::sin(yaw);
            const double vx = c * ext_v_fwd_ - s * ext_v_lat_;
            const double vy = s * ext_v_fwd_ + c * ext_v_lat_;
            publish_setpoint(vx, vy, vz, ext_yaw_rate_);
        }
        log_progress();
        return;
    }

    // ★★★ 起飞段位置环("打点上去")★★★
    //   只在起飞段生效：takeoff() 置 takeoff_pos_mode_，状态机在起飞后悬停稳定时调
    //   exit_takeoff_position_mode() 关掉 → 之后所有动作恢复速度环 PD(逐位同改动前)。
    //   期间不跑 PD、不做避障(避障是改 PD 追的有效目标，位置环下无从介入；起飞段原地
    //   垂直上升也不需要)。到位判定 is_reached() 不受影响(只比实际位姿 vs target_*)。
    if (takeoff_pos_mode_) {
        publish_position_setpoint(target_x_, target_y_, target_z_, target_yaw_);
        log_progress();
        return;
    }

    // 圆周运动：根据已用时间推进目标角，更新 xy 目标点 + 机头朝向圆心
    if (action_mode_ == ActionMode::CIRCLE && circle_active_) {
        update_circle_target();
    }

    // 全局避障：算"有效目标" eff_gx_/eff_gy_(默认=target_x_/y_，被杆挡时=贝塞尔前瞻点)。
    //   ★必须在 CIRCLE 改完 target 之后、compute 之前★。不改 target_x_/y_。
    update_effective_goal();

    // 其他模式：算速度命令并发布
    double vx, vy, vz, yr;
    compute_velocity_command(vx, vy, vz, yr);
    publish_setpoint(vx, vy, vz, yr);

    log_progress();
}

// ============================================================================
//   每 0.2s 打印一次当前动作的进度
//   IDLE 模式不打（节点刚启动时）
// ============================================================================
void DroneController::log_progress()
{
    if (action_mode_ == ActionMode::IDLE) return;

    const auto now = node_->now();
    const double elapsed = log_valid_ ? (now - last_log_).seconds() : 1e9;
    if (elapsed < 0.2) return;
    last_log_  = now;
    log_valid_ = true;

    const char* tag = "";
    switch (action_mode_) {
        case ActionMode::TAKEOFF:  tag = "[起飞]"; break;
        case ActionMode::MOVE_XY:  tag = "[平移]"; break;
        case ActionMode::MOVE_Z:   tag = "[升降]"; break;
        case ActionMode::MOVE_POSE: tag = "[位姿]"; break;
        case ActionMode::TURN_YAW: tag = "[转向]"; break;
        case ActionMode::CIRCLE:   tag = "[环绕]"; break;
        case ActionMode::EXTERNAL_VEL: tag = "[探索]"; break;
        case ActionMode::HOLD:     tag = "[悬停]"; break;
        case ActionMode::LAND:     tag = "[降落]"; break;
        case ActionMode::IDLE:     return;
    }
    RCLCPP_INFO(node_->get_logger(), "%s %s", tag, progress_string().c_str());
}

// ============================================================================
//   PD 控制
// ============================================================================
void DroneController::compute_velocity_command(
    double& vx, double& vy, double& vz, double& yaw_rate) const
{
    const double cx = current_pose_.pose.position.x;
    const double cy = current_pose_.pose.position.y;
    const double cz = current_pose_.pose.position.z;
    // 用"位置差分 + 低通"算出的速度估计当 D 项的反馈
    // 距离远 → P 项主导，速度被 MAX_SPEED 卡到上限
    // 距离近 → P 项变小，D 项与剩余速度抵消，平滑停在目标点
    const double cvx = v_est_x_;
    const double cvy = v_est_y_;
    const double cvz = v_est_z_;

    // 位置误差
    //   xy 用【有效目标 eff_gx_/eff_gy_】：无避障时=target_x_/y_；避障时=贝塞尔前瞻点(见 update_effective_goal)。
    //   z 仍用 target_z_(避障只改平移，不动升降)。
    const double ex = eff_gx_ - cx;
    const double ey = eff_gy_ - cy;
    const double ez = target_z_ - cz;

    // PD：v = Kp * 位置误差 - Kd * 当前速度
    double vx_raw = params::KP_XY * ex - params::KD_XY * cvx;
    double vy_raw = params::KP_XY * ey - params::KD_XY * cvy;
    double vz_raw = params::KP_Z  * ez - params::KD_Z  * cvz;

    // 水平做总速度上限裁剪（保持 vx/vy 比例不变），垂直单独裁剪
    // 环绕模式：水平限速 = 传入的环绕线速度 circle_speed_，并用 CIRCLE_MAX_SPEED_XY 安全封顶
    double max_speed_xy = params::MAX_SPEED_XY;
    if (action_mode_ == ActionMode::CIRCLE) {
        max_speed_xy = std::min(circle_speed_, params::CIRCLE_MAX_SPEED_XY);
    }
    double v_mag = std::sqrt(vx_raw*vx_raw + vy_raw*vy_raw);
    if (v_mag > max_speed_xy && v_mag > 1e-9) {
        const double s = max_speed_xy / v_mag;
        vx_raw *= s;
        vy_raw *= s;
    }
    vx = vx_raw;
    vy = vy_raw;

    // 垂直限速分两档：
    //   起飞 / 升降 / 降落 → 用大限速 MAX_SPEED_Z，正常爬升下降
    //   平飞（平移 / 转向 / 悬停 / 环绕）→ 用小限速 MAX_SPEED_Z_LEVEL，只允许微调，避免上下频繁矫正
    double z_limit = params::MAX_SPEED_Z;
    if (action_mode_ == ActionMode::MOVE_XY ||
        action_mode_ == ActionMode::TURN_YAW ||
        action_mode_ == ActionMode::CIRCLE ||
        action_mode_ == ActionMode::HOLD) {
        z_limit = params::MAX_SPEED_Z_LEVEL;
    }
    vz = clamp_abs(vz_raw, z_limit);

    // yaw：用 P 把误差变成 yaw_rate
    // 环绕模式用专用 yaw PD（CIRCLE_KP_YAW / CIRCLE_MAX_YAW_RATE），与通用转向分开调
    const double kp_yaw       = (action_mode_ == ActionMode::CIRCLE)
                                ? params::CIRCLE_KP_YAW : params::KP_YAW;
    const double max_yaw_rate = (action_mode_ == ActionMode::CIRCLE)
                                ? params::CIRCLE_MAX_YAW_RATE : params::MAX_YAW_RATE;
    const double yaw_err = wrap_pi(target_yaw_ - current_yaw());
    yaw_rate = clamp_abs(kp_yaw * yaw_err, max_yaw_rate);
}

// ============================================================================
//   圆周运动（闭环）：每拍用飞机实际位置算当前角，目标点放在前方一点的圆上，半径强制锁回 r，进度按实际转过的角度累计
// ============================================================================
void DroneController::update_circle_target()
{
    const double cx = current_pose_.pose.position.x;
    const double cy = current_pose_.pose.position.y;

    // 飞机相对圆心的实际角
    const double theta_cur = std::atan2(cy - circle_cy_, cx - circle_cx_);

    // 累计实际转过的角度（逆时针为正）：取本拍相对上一拍的增量，归一化到 [-π,π]
    double dtheta = wrap_pi(theta_cur - circle_theta_prev_);
    // 逆时针推进，只累加正向增量；反向抖动（dtheta<0）不倒扣，避免噪声让进度回退
    if (dtheta > 0.0) circle_progress_ += dtheta;
    circle_theta_prev_ = theta_cur;

    // 前瞻角 Δ：把目标点放在飞机前方一点，牵引它沿圆周走
    //   弧长前瞻 ≈ 0.5m 对应的角度（半径越大角越小），并夹在合理范围
    double lookahead = (circle_radius_ > 1e-6) ? (0.5 / circle_radius_) : 0.3;
    if (lookahead > 0.5) lookahead = 0.5;   // 约 28°，避免目标点跑太前导致切弦
    if (lookahead < 0.1) lookahead = 0.1;

    // 临近终点时收窄前瞻，让飞机稳稳停在 sweep 终点，不冲过头
    const double remain = circle_sweep_ - circle_progress_;
    if (remain < lookahead) lookahead = std::max(remain, 0.0);

    const double theta_target = theta_cur + lookahead;

    // 目标点：半径强制用 r（不是飞机实际半径）→ PD 把半径锁回 r
    target_x_ = circle_cx_ + circle_radius_ * std::cos(theta_target);
    target_y_ = circle_cy_ + circle_radius_ * std::sin(theta_target);

    // 机头朝向圆心：从目标点指向圆心的方向
    target_yaw_ = std::atan2(circle_cy_ - target_y_, circle_cx_ - target_x_);
}

// ============================================================================
//   setpoint 出口：把当前目标 + 当前位置 + 当前速度 → 速度命令（含限幅）
// ============================================================================
void DroneController::publish_setpoint(double vx, double vy, double vz, double yaw_rate)
{
    if (!std::isfinite(vx) || !std::isfinite(vy) ||
        !std::isfinite(vz) || !std::isfinite(yaw_rate)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
            "[setpoint] 非有限数 (vx=%f vy=%f vz=%f yr=%f)，强制清零",
            vx, vy, vz, yaw_rate);
        vx = vy = vz = yaw_rate = 0.0;
    }

    mavros_msgs::msg::PositionTarget msg;
    msg.header.stamp    = node_->now();
    msg.header.frame_id = "map";
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask =
        mavros_msgs::msg::PositionTarget::IGNORE_PX |
        mavros_msgs::msg::PositionTarget::IGNORE_PY |
        mavros_msgs::msg::PositionTarget::IGNORE_PZ |
        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
        mavros_msgs::msg::PositionTarget::IGNORE_YAW;
    msg.velocity.x = vx;
    msg.velocity.y = vy;
    msg.velocity.z = vz;
    msg.yaw_rate   = yaw_rate;
    setpoint_pub_->publish(msg);
}

// ============================================================================
//   ★位置 setpoint 出口★(起飞段 "打点上去" 用)
//   直接发目标位置 + 目标 yaw，屏蔽速度/加速度/yaw_rate(与速度环出口正好互补)，
//   由飞控内部位置环飞过去。本程序在这一段不算速度、不做 PD 矫正。
//   前提：飞控 local 位置估计与 SLAM/camera_init 同源(见 params 的开关注释)。
// ============================================================================
void DroneController::publish_position_setpoint(double x, double y, double z, double yaw)
{
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(z) || !std::isfinite(yaw)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
            "[位置setpoint] 非有限数 (x=%f y=%f z=%f yaw=%f)，改发当前位姿保持",
            x, y, z, yaw);
        // 退化成"保持当前位姿"，绝不把 NaN 发给飞控
        x = current_pose_.pose.position.x;
        y = current_pose_.pose.position.y;
        z = current_pose_.pose.position.z;
        yaw = current_yaw();
        if (!std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(z) || !std::isfinite(yaw)) return;
    }

    mavros_msgs::msg::PositionTarget msg;
    msg.header.stamp    = node_->now();
    msg.header.frame_id = "map";
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    // 只给位置 + yaw
    msg.type_mask =
        mavros_msgs::msg::PositionTarget::IGNORE_VX |
        mavros_msgs::msg::PositionTarget::IGNORE_VY |
        mavros_msgs::msg::PositionTarget::IGNORE_VZ |
        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
        mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    msg.yaw        = static_cast<float>(yaw);
    setpoint_pub_->publish(msg);
}

// ============================================================================
//   到位判定：根据 action_mode_ 
// ============================================================================
// 对外的默认到位判定：用正常飞行的水平容差 params::TOL_XY。
bool DroneController::is_reached() const
{
    return is_reached_tol(params::TOL_XY);
}

// 自定水平容差版(实现体)：is_reached() 与 is_reached_tol() 共用这一份逻辑，
//   唯一差别就是水平容差取传入的 tol_xy(z/yaw 容差与稳定计时完全不变)。
bool DroneController::is_reached_tol(double tol_xy) const
{
    if (!has_pose_.load()) return false;

    switch (action_mode_) {

    case ActionMode::IDLE:
        return false;

    case ActionMode::HOLD: {
        // wait_time：检查计时
        if (wait_active_) return node_->now() >= wait_until_;
        // 普通 HOLD（无计时）→ 永远 false（保持中，由上层切换 mode 退出）
        return false;
    }

    case ActionMode::LAND: {
        // 触底 + 已上锁
        return (current_pose_.pose.position.z - home_z_ < params::LAND_DONE_REL_HEIGHT)
               && !current_state_.armed;
    }

    case ActionMode::TAKEOFF:
    case ActionMode::MOVE_XY:
    case ActionMode::MOVE_Z: {
        // 位置类：xyz 全部在容差内 + 持续 SETTLE_DURATION 秒
        const double dx = current_pose_.pose.position.x - target_x_;
        const double dy = current_pose_.pose.position.y - target_y_;
        const double dz = current_pose_.pose.position.z - target_z_;
        const double xy = std::sqrt(dx*dx + dy*dy);
        const double z  = std::abs(dz);
        if (xy > tol_xy || z > params::TOL_Z) {   // ★水平容差用传入值★(默认=TOL_XY，找图传 FF_TOL_XY)
            settle_valid_ = false;
            return false;
        }
        if (!settle_valid_) {
            settle_start_ = node_->now();
            settle_valid_ = true;
            return false;
        }
        return (node_->now() - settle_start_).seconds() >= params::SETTLE_DURATION;
    }

    case ActionMode::MOVE_POSE: {
        // 一次到位：xyz 且 yaw 全部进容差 + 持续 SETTLE_DURATION 秒
        const double dx = current_pose_.pose.position.x - target_x_;
        const double dy = current_pose_.pose.position.y - target_y_;
        const double dz = current_pose_.pose.position.z - target_z_;
        const double xy = std::sqrt(dx*dx + dy*dy);
        const double z  = std::abs(dz);
        const double yaw_err = std::abs(wrap_pi(target_yaw_ - current_yaw()));
        if (xy > tol_xy || z > params::TOL_Z || yaw_err > params::TOL_YAW) {   // ★水平容差用传入值★
            settle_valid_ = false;
            return false;
        }
        if (!settle_valid_) {
            settle_start_ = node_->now();
            settle_valid_ = true;
            return false;
        }
        return (node_->now() - settle_start_).seconds() >= params::SETTLE_DURATION;
    }

    case ActionMode::CIRCLE: {
        // 圆周：已扫过的角度达到总扫角即完成
        return circle_active_ && (circle_progress_ >= circle_sweep_ - 1e-6);
    }

    case ActionMode::EXTERNAL_VEL:
        // 探索是否结束由算法的 /exploration/finished 决定，drone_ 自身不判定
        return false;

    case ActionMode::TURN_YAW: {
        // yaw 类：yaw 误差 + 持续 SETTLE_DURATION_YAW 秒
        const double err = std::abs(wrap_pi(target_yaw_ - current_yaw()));
        if (err > params::TOL_YAW) {
            settle_valid_ = false;
            return false;
        }
        if (!settle_valid_) {
            settle_start_ = node_->now();
            settle_valid_ = true;
            return false;
        }
        return (node_->now() - settle_start_).seconds() >= params::SETTLE_DURATION_YAW;
    }
    }
    return false;
}

// ============================================================================
//   当前动作的进度描述（用于打印日志）
// ============================================================================
std::string DroneController::progress_string() const
{
    if (!has_pose_.load()) return "（无雷达数据）";

    char buf[128];
    const double cx = current_pose_.pose.position.x;
    const double cy = current_pose_.pose.position.y;
    const double cz = current_pose_.pose.position.z;

    switch (action_mode_) {

    case ActionMode::IDLE:
        std::snprintf(buf, sizeof(buf), "空闲");
        break;

    case ActionMode::TAKEOFF: {
        const double rel_h = cz - home_z_;
        const double tgt_h = target_z_ - home_z_;
        std::snprintf(buf, sizeof(buf), "高度 %.2f / %.2fm", rel_h, tgt_h);
        break;
    }

    case ActionMode::MOVE_XY: {
        const double dx = target_x_ - cx;
        const double dy = target_y_ - cy;
        std::snprintf(buf, sizeof(buf), "x 剩余 %.2fm  y 剩余 %.2fm", dx, dy);
        break;
    }

    case ActionMode::MOVE_Z: {
        const double dz = target_z_ - cz;
        std::snprintf(buf, sizeof(buf), "剩余 %.2fm", dz);
        break;
    }

    case ActionMode::MOVE_POSE: {
        const double dx = target_x_ - cx, dy = target_y_ - cy, dz = target_z_ - cz;
        const double eyaw = wrap_pi(target_yaw_ - current_yaw()) * 180.0 / M_PI;
        std::snprintf(buf, sizeof(buf), "dx%.2f dy%.2f dz%.2f yaw误差%.1f°", dx, dy, dz, eyaw);
        break;
    }

    case ActionMode::TURN_YAW: {
        const double err_deg = wrap_pi(target_yaw_ - current_yaw()) * 180.0 / M_PI;
        std::snprintf(buf, sizeof(buf), "yaw 误差 %.1f°", err_deg);
        break;
    }

    case ActionMode::CIRCLE: {
        const double pct = (circle_sweep_ > 1e-6)
                           ? (circle_progress_ / circle_sweep_ * 100.0) : 100.0;
        std::snprintf(buf, sizeof(buf), "环绕 %.1f° / %.1f° (%.0f%%)",
                      circle_progress_ * 180.0 / M_PI,
                      circle_sweep_   * 180.0 / M_PI,
                      pct);
        break;
    }

    case ActionMode::EXTERNAL_VEL: {
        const double rel_h = cz - home_z_;
        std::snprintf(buf, sizeof(buf),
                      "v_fwd %.2f v_lat %.2f yr %.2f  高度 %.2fm",
                      ext_v_fwd_, ext_v_lat_, ext_yaw_rate_, rel_h);
        break;
    }

    case ActionMode::HOLD: {
        if (wait_active_) {
            const double remain = (wait_until_ - node_->now()).seconds();
            std::snprintf(buf, sizeof(buf), "剩余 %.2fs", remain);
        } else {
            std::snprintf(buf, sizeof(buf), "保持中");
        }
        break;
    }

    case ActionMode::LAND: {
        const double rel_h = cz - home_z_;
        std::snprintf(buf, sizeof(buf), "相对高度 %.2fm", rel_h);
        break;
    }
    }
    return std::string(buf);
}

// ============================================================================
//   运动 API 实现
// ============================================================================

namespace {
// 切到新 action mode 时的统一处理：清掉旧的稳定计时
void reset_for_new_action(bool& settle_valid, bool& wait_active)
{
    settle_valid = false;
    wait_active  = false;
}
}  // namespace

void DroneController::capture_home()
{
    if (!has_pose_.load()) {
        RCLCPP_WARN(node_->get_logger(), "[home] 没有雷达数据，无法记录 home！");
        return;
    }
    home_x_   = current_pose_.pose.position.x;
    home_y_   = current_pose_.pose.position.y;
    home_z_   = current_pose_.pose.position.z;
    home_yaw_ = current_yaw();
    home_captured_ = true;

    // 起飞前先把目标 = 当前位置（避免 mode 切换瞬间目标是 0,0,0）
    target_x_   = home_x_;
    target_y_   = home_y_;
    target_z_   = home_z_;
    target_yaw_ = home_yaw_;
    RCLCPP_INFO(node_->get_logger(), "[home] 已记录 home (%.2f, %.2f, %.2f, yaw=%.2f°)",
                home_x_, home_y_, home_z_, home_yaw_ * 180.0 / M_PI);
}

// ============================================================================
//   主动停止：任务中止 / 完成时调，让 tick 不再跑 PD，只发零速度占位
// ============================================================================
void DroneController::stop()
{
    action_mode_  = ActionMode::IDLE;
    wait_active_  = false;
    settle_valid_ = false;
    circle_active_ = false;
    takeoff_pos_mode_ = false;   // 起飞途中中止：退出位置环(IDLE 分支自己会按开关发占位)
    // 复位绕杆子状态机(停采集、回 IDLE)，避免下次 description_circle_right 续用旧态
    pole_phase_ = PoleCirclePhase::IDLE;
    { std::lock_guard<std::mutex> lk(pole_mtx_); pole_collecting_ = false; }
}

// ============================================================================
//   请求解锁：每秒最多发一次，最多重试 5 次。已解锁则直接返回成功。
//   返回 true = 还在尝试或已成功；false = 已放弃
// ============================================================================
bool DroneController::request_arm()
{
    if (current_state_.armed) return true;          // 已经解锁
    if (arm_giveup_) return false;                  // 之前已放弃

    constexpr int    MAX_RETRY        = 5;
    constexpr double RETRY_INTERVAL_S = 1.0;

    const auto now = node_->now();
    if (arm_time_valid_ &&
        (now - arm_last_try_).seconds() < RETRY_INTERVAL_S) {
        return true;                                // 间隔不到 1s，先不重复发
    }

    if (arm_retry_count_ >= MAX_RETRY) {
        RCLCPP_ERROR(node_->get_logger(),
            "[解锁] 重试 %d 次仍失败，放弃", MAX_RETRY);
        arm_giveup_ = true;
        return false;
    }

    arm_last_try_  = now;
    arm_time_valid_ = true;
    arm_retry_count_++;

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    arming_client_->async_send_request(req,
        [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                if (resp->success) {
                    RCLCPP_INFO(node_->get_logger(), "[解锁] 飞控已确认解锁");
                } else {
                    RCLCPP_WARN(node_->get_logger(),
                        "[解锁] 飞控拒绝（result=%u），将重试", resp->result);
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(),
                    "[解锁] 服务异常：%s", e.what());
            }
        });
    RCLCPP_INFO(node_->get_logger(),
        "[解锁] 发送解锁请求（第 %d/%d 次）", arm_retry_count_, MAX_RETRY);
    return true;
}

void DroneController::takeoff(double altitude_relative_home)
{
    if (action_mode_ != ActionMode::TAKEOFF) {
        reset_for_new_action(settle_valid_, wait_active_);
        action_mode_ = ActionMode::TAKEOFF;
        // ★进起飞段位置环★("打点上去")：tick() 从此发位置 setpoint。
        //   由状态机在起飞后悬停稳定时调 exit_takeoff_position_mode() 退出。
        if (params::TAKEOFF_POSITION_MODE) takeoff_pos_mode_ = true;
    }
    target_x_   = home_x_;
    target_y_   = home_y_;
    target_z_   = home_z_ + altitude_relative_home;
    target_yaw_ = home_yaw_;
}

// 退出起飞段位置环 → 之后所有动作恢复速度环 PD。幂等。
void DroneController::exit_takeoff_position_mode()
{
    if (!takeoff_pos_mode_) return;      // 幂等：没在位置环(或开关关着)直接返回
    takeoff_pos_mode_ = false;
    // ★清掉 PD 的速度估计残留★：起飞段没跑 PD，v_est_* 仍是 odom 回调一直在算的值，
    //   本身是连续的，所以不用清；但稳定计时要清——切换瞬间重新起算，避免用位置环
    //   阶段攒下的 settle 直接判"已到位"。
    settle_valid_ = false;
    RCLCPP_INFO(node_->get_logger(),
        "[起飞] 退出位置环 → 之后改用速度环 PD(走航点/找图等照常)");
}

void DroneController::land()
{
    if (action_mode_ != ActionMode::LAND) {
        reset_for_new_action(settle_valid_, wait_active_);
        action_mode_    = ActionMode::LAND;
        land_requested_ = false;
    }
}

void DroneController::target_xy_slam(double x_slam, double y_slam)
{
    if (action_mode_ != ActionMode::MOVE_XY ||
        std::abs(target_x_ - x_slam) > 1e-6 || std::abs(target_y_ - y_slam) > 1e-6) {
        reset_for_new_action(settle_valid_, wait_active_);
        action_mode_ = ActionMode::MOVE_XY;
    }
    target_x_ = x_slam;
    target_y_ = y_slam;
    // z / yaw 保持调用瞬间值（target_z_、target_yaw_ 不改）
}

void DroneController::target_xy_body(double dx_body, double dy_body)
{
    
    if (action_mode_ == ActionMode::MOVE_XY &&
        std::abs(body_dx_cmd_ - dx_body) < 1e-9 &&
        std::abs(body_dy_cmd_ - dy_body) < 1e-9) {
        return;
    }
    body_dx_cmd_ = dx_body;
    body_dy_cmd_ = dy_body;

    double x_slam, y_slam;
    body_to_slam_xy(dx_body, dy_body, x_slam, y_slam);
    target_xy_slam(x_slam, y_slam);
}

void DroneController::target_z_slam(double z_slam)
{
    if (action_mode_ != ActionMode::MOVE_Z ||
        std::abs(target_z_ - z_slam) > 1e-6) {
        reset_for_new_action(settle_valid_, wait_active_);
        action_mode_ = ActionMode::MOVE_Z;
    }
    target_z_ = z_slam;
}

void DroneController::target_z_body(double dz_body)
{
    if (action_mode_ == ActionMode::MOVE_Z &&
        std::abs(body_dz_cmd_ - dz_body) < 1e-9) {
        return;
    }
    body_dz_cmd_ = dz_body;
    target_z_slam(current_pose_.pose.position.z + dz_body);
}

void DroneController::target_yaw_slam(double yaw_slam_deg)
{
    const double new_yaw = wrap_pi(yaw_slam_deg * M_PI / 180.0);
    if (action_mode_ != ActionMode::TURN_YAW ||
        std::abs(wrap_pi(target_yaw_ - new_yaw)) > 1e-6) {
        reset_for_new_action(settle_valid_, wait_active_);
        action_mode_ = ActionMode::TURN_YAW;
        // 切到 TURN_YAW 瞬间锁 xy/z 为当前位置（避免飘）
        target_x_ = current_pose_.pose.position.x;
        target_y_ = current_pose_.pose.position.y;
        target_z_ = current_pose_.pose.position.z;
    }
    target_yaw_ = new_yaw;
}

// 一次到位：同时设 x/y/z + yaw 目标，切 MOVE_POSE。PD 同拍控制平移/升降/转向(边转边走升高)。
void DroneController::target_pose_slam(double x_slam, double y_slam, double z_slam, double yaw_slam_deg)
{
    const double new_yaw = wrap_pi(yaw_slam_deg * M_PI / 180.0);
    if (action_mode_ != ActionMode::MOVE_POSE) {
        reset_for_new_action(settle_valid_, wait_active_);
        action_mode_ = ActionMode::MOVE_POSE;
    }
    target_x_   = x_slam;
    target_y_   = y_slam;
    target_z_   = z_slam;
    target_yaw_ = new_yaw;
}

void DroneController::target_yaw_body(double dyaw_body_deg)
{
    if (action_mode_ == ActionMode::TURN_YAW &&
        std::abs(body_dyaw_cmd_ - dyaw_body_deg) < 1e-9) {
        return;
    }
    body_dyaw_cmd_ = dyaw_body_deg;
    // 当前 yaw 是弧度，dyaw 入参是度 → 加完之后传给 slam 版 yaw API 时再转回弧度
    const double current_deg = current_yaw() * 180.0 / M_PI;
    target_yaw_slam(current_deg + dyaw_body_deg);
}

// ============================================================================
//   绕杆环绕（无参）：有状态原语，内部子状态机 采集→接近→环绕。
//   状态机每拍调它一次即可；用 is_reached() 判整个流程是否完成。
// ============================================================================
void DroneController::description_circle_right()
{
    // 首次进入(或从别的动作切进来)：复位子状态机到 COLLECT，开始悬停攒帧。
    if (pole_phase_ == PoleCirclePhase::IDLE ||
        pole_phase_ == PoleCirclePhase::DONE ||
        pole_phase_ == PoleCirclePhase::FAIL) {
        pole_phase_ = PoleCirclePhase::COLLECT;
        pole_z_lock_ = current_pose_.pose.position.z;   // 锁高：全程只飞 xy
        pole_collect_time_valid_ = false;
        {
            std::lock_guard<std::mutex> lk(pole_mtx_);
            pole_sx_.clear(); pole_sy_.clear(); pole_sr_.clear();
            pole_collecting_ = true;                     // 从下一拍起接收样本
        }
    }
    pole_circle_tick();
}

// 内部：绕杆子状态机推进。返回 true = 整个流程完成(DONE)。FAIL 也结束(返回 false，由上层判 phase)。
bool DroneController::pole_circle_tick()
{
    switch (pole_phase_) {

    case PoleCirclePhase::COLLECT: {
        // 悬停锁位攒帧
        if (action_mode_ != ActionMode::HOLD) {
            action_mode_ = ActionMode::HOLD;
            target_x_ = current_pose_.pose.position.x;
            target_y_ = current_pose_.pose.position.y;
            target_z_ = pole_z_lock_;
            target_yaw_ = current_yaw();
            wait_active_ = false;      // 用帧数/超时推进，不用 wait_time 计时
        }
        if (!pole_collect_time_valid_) {
            pole_collect_start_ = node_->now();
            pole_collect_time_valid_ = true;
        }
        size_t n; double sx = 0, sy = 0, sr = 0;
        {
            std::lock_guard<std::mutex> lk(pole_mtx_);
            n = pole_sx_.size();
            for (size_t i = 0; i < n; ++i) { sx += pole_sx_[i]; sy += pole_sy_[i]; sr += pole_sr_[i]; }
        }
        const double elapsed = (node_->now() - pole_collect_start_).seconds();
        const bool enough  = (n >= static_cast<size_t>(params::POLE_CIRCLE_COLLECT_FRAMES));
        const bool timeout = (elapsed >= params::POLE_CIRCLE_COLLECT_TIMEOUT_S);

        if (!enough && !timeout) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "[绕杆] 采集中 %zu/%d 帧…", n, params::POLE_CIRCLE_COLLECT_FRAMES);
            return false;
        }
        if (!enough && n < static_cast<size_t>(params::POLE_CIRCLE_MIN_FRAMES)) {
            { std::lock_guard<std::mutex> lk(pole_mtx_); pole_collecting_ = false; }
            RCLCPP_WARN(node_->get_logger(),
                "[绕杆] 采集 %.1fs 仅 %zu 帧(<%d)，没看到杆 → 放弃(悬停)",
                elapsed, n, params::POLE_CIRCLE_MIN_FRAMES);
            pole_phase_ = PoleCirclePhase::FAIL;
            return false;
        }
        // 攒够 或 超时但够最小帧数 → 定杆心 + 接近半径
        { std::lock_guard<std::mutex> lk(pole_mtx_); pole_collecting_ = false; }
        pole_cx_ = sx / static_cast<double>(n);
        pole_cy_ = sy / static_cast<double>(n);
        const double pole_r = sr / static_cast<double>(n);            // 平均杆半径
        pole_approach_r_ = pole_r + params::POLE_CIRCLE_STANDOFF_M;   // 离杆表面 standoff → 离杆心 = 半径+standoff
        RCLCPP_INFO(node_->get_logger(),
            "[绕杆] 用 %zu 帧：杆心(%.2f,%.2f) 半径≈%.3fm → 接近半径 %.2fm(离杆表面%.2fm)",
            n, pole_cx_, pole_cy_, pole_r, pole_approach_r_, params::POLE_CIRCLE_STANDOFF_M);
        pole_phase_ = PoleCirclePhase::APPROACH;
        return false;
    }

    case PoleCirclePhase::APPROACH: {
        // 接近点：从杆心朝【当前飞机方向】退到接近半径处(就近切入圆)；机头朝杆；z 锁定。
        const double dx = current_pose_.pose.position.x - pole_cx_;
        const double dy = current_pose_.pose.position.y - pole_cy_;
        double d = std::hypot(dx, dy);
        double ux, uy;
        if (d > 1e-3) { ux = dx / d; uy = dy / d; }
        else { ux = 1.0; uy = 0.0; }                 // 恰在杆心上(不该发生)：任取一方向
        const double ax = pole_cx_ + pole_approach_r_ * ux;
        const double ay = pole_cy_ + pole_approach_r_ * uy;
        const double face_yaw_deg = std::atan2(pole_cy_ - ay, pole_cx_ - ax) * 180.0 / M_PI;  // 机头朝杆

        // 复用 MOVE_POSE：边转边动、只飞 xy(z 给锁定高度)。target_pose_slam 内部会切 MOVE_POSE。
        target_pose_slam(ax, ay, pole_z_lock_, face_yaw_deg);

        if (is_reached()) {   // MOVE_POSE 到位(xyz+yaw 进容差且稳定)
            RCLCPP_INFO(node_->get_logger(), "[绕杆] 已到杆前接近点、对准 → 开始绕杆");
            // 初始化圆周闭环：圆心=杆心，半径=接近半径，线速度=POLE_CIRCLE_SPEED，扫 sweep。
            reset_for_new_action(settle_valid_, wait_active_);
            action_mode_  = ActionMode::CIRCLE;
            circle_cx_    = pole_cx_;
            circle_cy_    = pole_cy_;
            circle_radius_ = pole_approach_r_;
            circle_speed_  = params::POLE_CIRCLE_SPEED;
            circle_sweep_  = params::POLE_CIRCLE_SWEEP_DEG * M_PI / 180.0;
            circle_theta_prev_ = std::atan2(current_pose_.pose.position.y - circle_cy_,
                                            current_pose_.pose.position.x - circle_cx_);
            circle_progress_ = 0.0;
            circle_active_   = true;
            target_z_        = pole_z_lock_;   // 环绕高度锁定(只飞 xy)
            pole_phase_      = PoleCirclePhase::CIRCLE;
        }
        return false;
    }

    case PoleCirclePhase::CIRCLE: {
        // 环绕由 tick() 的 CIRCLE 分支每拍推进(update_circle_target)；这里只判完成。
        if (circle_active_ && circle_progress_ >= circle_sweep_ - 1e-6) {
            RCLCPP_INFO(node_->get_logger(), "[绕杆] 绕杆完成 (%.0f°)",
                        params::POLE_CIRCLE_SWEEP_DEG);
            pole_phase_ = PoleCirclePhase::DONE;
            return true;
        }
        return false;
    }

    case PoleCirclePhase::DONE:
        return true;
    case PoleCirclePhase::FAIL:
    case PoleCirclePhase::IDLE:
        return false;
    }
    return false;
}

void DroneController::wait_time(double seconds)
{
    if (action_mode_ != ActionMode::HOLD || !wait_active_) {
        // 切到 HOLD：锁定当前位置 + 当前 yaw + 启动计时
        action_mode_ = ActionMode::HOLD;
        target_x_    = current_pose_.pose.position.x;
        target_y_    = current_pose_.pose.position.y;
        target_z_    = current_pose_.pose.position.z;
        target_yaw_  = current_yaw();
        wait_until_  = node_->now() + rclcpp::Duration::from_seconds(seconds);
        wait_active_ = true;
        settle_valid_ = false;
    }
    // 重复调用：不重置计时（幂等）
}

// ============================================================================
//   外部速度（探索）：进入探索 + 缓存机体系速度命令
// ============================================================================
void DroneController::enter_exploration()
{
    // 锁定当前高度为探索期间的保持高度
    explore_z_ = current_pose_.pose.position.z;
    ext_v_fwd_ = ext_v_lat_ = ext_yaw_rate_ = 0.0;
    ext_valid_ = false;                       // 还没收到算法速度 → 看门狗先保高
    reset_for_new_action(settle_valid_, wait_active_);
    action_mode_ = ActionMode::EXTERNAL_VEL;
}

void DroneController::set_velocity_body(double v_fwd, double v_lat, double yaw_rate)
{
    if (action_mode_ != ActionMode::EXTERNAL_VEL) {
        reset_for_new_action(settle_valid_, wait_active_);
        action_mode_ = ActionMode::EXTERNAL_VEL;
    }
    ext_v_fwd_    = v_fwd;
    ext_v_lat_    = v_lat;
    ext_yaw_rate_ = yaw_rate;
    ext_cmd_time_ = node_->now();
    ext_valid_    = true;
}

// ============================================================================
//   函数：欧拉角转换、坐标变换
// ============================================================================
double DroneController::current_yaw() const
{
    const auto& q = current_pose_.pose.orientation;
    const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (n < 1e-9) return 0.0;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tq);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void DroneController::body_to_slam_xy(double dx_body, double dy_body,
                                      double& out_x_slam, double& out_y_slam) const
{
    // "body" = 调用瞬间的飞机当前位置 + 当前 yaw 朝向
    // dx_body 前进、dy_body 左移
    const double yaw = current_yaw();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    out_x_slam = current_pose_.pose.position.x + c * dx_body - s * dy_body;
    out_y_slam = current_pose_.pose.position.y + s * dx_body + c * dy_body;
}

}  // namespace fly_mission
