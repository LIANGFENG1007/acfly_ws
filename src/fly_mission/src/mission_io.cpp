// ============================================================================
//  mission_io.cpp —— 外部 IO：Arduino 串口 / 视觉 shm 信箱 / 跨机 UDP / 遥测
//  ---------------------------------------------------------------------------
//  这些都是"与外界交换数据"，不含任何飞行决策：状态机只读它们留下的成员标志
//  (start_recv_ / lock_dx_ / lock_tgt_* …)，不直接碰底层收发。
// ============================================================================

#include "fly_mission/fly_mission_node.hpp"

#include <nlohmann/json.hpp>
#include <cmath>       // std::hypot / std::cos / std::sin / std::isfinite

namespace fly_mission {

// ════════════════════════════════════════════════════════════════════════════
//  Arduino 串口
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::arduino_send(const std::string& text, int times)
{
    if (arduino_.send(text, times, params::ARDUINO_SEND_GAP_MS)) {
        RCLCPP_INFO(get_logger(), "[Arduino] 已发 \"%s\" ×%d", text.c_str(), times);
    } else {
        RCLCPP_WARN(get_logger(), "[Arduino] 串口发送失败(设备 %s 没插/名字不对?)，任务继续",
                    params::ARDUINO_DEV);
    }
}

void FlyMissionNode::arduino_send_async(const std::string& text, int times)
{
    if (arduino_.queue(text, times)) {
        RCLCPP_INFO(get_logger(), "[Arduino] 已排队 \"%s\" ×%d(非阻塞，本拍不等发送完成)",
                    text.c_str(), times);
    } else {
        RCLCPP_WARN(get_logger(),
            "[Arduino] 发送队列满，\"%s\" 被丢弃(串口 %s 不通? 待发 %zu 字节)",
            text.c_str(), params::ARDUINO_DEV, arduino_.pending());
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  跨机命令 UDP 收包
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::poll_udp_cmd()
{
    if (!cmd_rx_) return;
    if (!cmd_rx_->poll()) return;       // 本拍没有【新】命令(重复发的不算)

    if (cmd_rx_->got(udp_cmd::CMD_START) && !start_recv_) {
        start_recv_ = true;
        RCLCPP_INFO(get_logger(),
            "[启动] 收到启动指令(UDP seq=%u，累计收包 %u)（已锁定，后续重复发将忽略）",
            cmd_rx_->last_seq(), cmd_rx_->rx_count());
    }

    // ★命令 2(第二段任务触发)★
    //   ★只在 WAIT_TRIGGER(地面待机)状态才接受★——这一条很关键：
    //   本函数每拍都跑，不看状态。若对方的发送脚本在第一段还在飞的时候就已经在发
    //   命令 2(提前开了脚本 / 上一轮的进程没关)，trigger_recv_ 会被提前锁存；
    //   等第一段降落进 WAIT_TRIGGER 时标志已是 true → ★不等指令直接起飞★，
    //   人还没准备好桨就转了。
    //   所以这里加状态门：不在待机状态收到的命令 2 一律忽略(只提示)。
    //   UdpCmdReceiver 内部仍然锁存，所以要用 reset() 清掉，
    //   否则进入 WAIT_TRIGGER 后 got() 立刻为真，等于门没加。
    if (cmd_rx_->got(udp_cmd::CMD_TAKEOFF_AGAIN) && !trigger_recv_) {
        if (state_ == MissionState::WAIT_TRIGGER) {
            trigger_recv_ = true;
            RCLCPP_INFO(get_logger(),
                "[二段] 收到命令 2(UDP)（已锁定，后续重复发将忽略）");
        } else {
            cmd_rx_->reset(udp_cmd::CMD_TAKEOFF_AGAIN);   // 丢弃，别提前锁存
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "[二段] ★收到命令 2 但当前不在地面待机(状态=%s)，已忽略★"
                "——请等飞机第一段降落上锁后再发",
                state_name(state_));
        }
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  视觉 shm 信箱消费
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::consume_shm_cv()
{
    if (!params::USE_SHM_CV) return;

    shm::CvFrame f;
    if (!shm_cv_.read(f) || f.seq == shm_last_seq_) return;   // 无新帧：不重复喂同一帧
    shm_last_seq_ = f.seq;

    static const char* kColor[] = { "red", "green", "yellow", "blue" };
    static const char* kShape[] = { "triangle", "square", "round", "unknow" };
    nlohmann::json j;
    j["line_x"] = f.line_x;               // 寻线字段(finding 程序恒0；blackline 接入后有值)
    j["line_y"] = f.line_y;
    auto arr = nlohmann::json::array();
    for (int i = 0; i < f.n_targets; ++i) {
        const auto& t = f.targets[i];
        if (t.color < 0 || t.color > 3 || t.shape < 0 || t.shape > 3) continue;  // 未知编码丢弃
        arr.push_back({ {"id", t.id}, {"color", kColor[t.color]},
                        {"shape", kShape[t.shape]}, {"x", t.x}, {"y", t.y} });
    }
    j["targets"] = arr;
    find_.ingest(j);
    lf_.ingest(j);

    // ★锁定投掷(LOCK_DROP)用的 dx/dy★：取【第一个目标】的 x/y 当机体系偏移(前+/左+)。
    //   与找图共用同一份 shm 帧，只是找图看 color/shape，这里看相对量。
    //   不经 kColor/kShape 过滤，直接取原始 t.x/t.y。
    //
    //   ★★★ 坐标系约定(已与视觉端逐项核对) ★★★
    //   配套视觉程序 = ws_opencv/github_vison-master/down_vision
    //     · params.yaml: shm_world_coords: false  → ★写机体系 dx/dy★
    //     · x = dx 向前为正、y = dy 向左为正(视觉端 shm_swap_xy: false)
    //     · dx 由 (K_.cy - py)*height/K_.fy 算出，"前为正"，与本段一致
    //     · id/color/shape 恒 0(该程序不发类别)，本段不用这三个字段
    //     · 该程序每帧都写(没目标时 n_targets=0)，所以 seq 会持续推进
    //   ★注意 shm 的 x/y 没有字段标明坐标系★：另一个视觉程序 race_2025 的
    //   shm_world_coords=true 写的是 SLAM 世界绝对坐标，语义相反。
    //   ⇒ ★同时只能跑一个视觉程序★。跑错了的症状：dx/dy 是"飞机在场地里的
    //     绝对坐标"那种数值(几米、且不随对准而减小) → 越飞越偏、永不投掷。
    //   地面自检：手持飞机对着目标移动，看 [锁定] 日志的 dx/dy 是否趋近 0。
    //
    // ★视觉进程"活着"的心跳★：视觉端每拍都写(没看到目标也写 n_targets=0)，
    //   所以只要读到新 seq 就说明视觉在跑。
    //   ★与"看到目标"分开记★：否则"视觉活着但暂时没看到目标"和"视觉挂了"
    //   在主控这边表现完全一样(都是 dx/dy 不更新)，日志会误导排查方向。
    lock_cv_alive_time_  = now();
    lock_cv_alive_valid_ = true;

    if (f.n_targets > 0) {
        const double dx = f.targets[0].x;
        const double dy = f.targets[0].y;
        if (std::isfinite(dx) && std::isfinite(dy)) {
            lock_dx_ = dx;
            lock_dy_ = dy;
            lock_cv_time_  = now();     // ★本机时钟★：判视觉数据新鲜度
            lock_cv_valid_ = true;
            // ★收到新帧就立刻把机体系偏移换算成 SLAM 绝对目标点并冻结★
            //   ——绝不能在状态机里每拍用"当前位置+同一个旧 dx/dy"重算：
            //   视觉写 50Hz、状态机读 50Hz —— ★同频但不同步★，相位漂移会让
            //   某些拍读不到新帧。那些拍若用【已经移动过的当前位置】加同一个
            //   旧偏移重算，就会把目标点一路往前推 → 正反馈，飞机越冲越远。
            //   在收帧这一刻算，用的就是"看到目标时飞机在哪"，物理上正确。
            if (drone_.has_pose()) {
                const double yaw = drone_.current_yaw_deg() * M_PI / 180.0;
                const double c = std::cos(yaw), s = std::sin(yaw);
                lock_tgt_x_ = drone_.current_x() + c * dx - s * dy;
                lock_tgt_y_ = drone_.current_y() + s * dx + c * dy;
                lock_tgt_valid_ = true;
            }
        }
    }

    const double age_ms = (shm::ShmMailboxReader::now_mono() - f.stamp) * 1000.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[shm] 消费 seq=%lu 数据年龄=%.1fms targets=%d",
        static_cast<unsigned long>(f.seq), age_ms, f.n_targets);
}

// ════════════════════════════════════════════════════════════════════════════
//  遥测上报
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::telemetry_tick()
{
    if (!tlm_tx_) return;

    const double now_s = udp_tlm::UdpTelemetrySender::now_mono();
    if (params::TLM_RATE_HZ > 0.0 &&
        (now_s - tlm_last_send_) < (1.0 / params::TLM_RATE_HZ)) return;
    tlm_last_send_ = now_s;

    // 小车坐标要发【原始 B 系值】(未加 CAR_ORIGIN 平移)——标定错了原始值仍然对，
    //   排查时更有用。监控端要画在同一张图上就自己加 CAR_ORIGIN_X/Y。
    double raw_x = 0.0, raw_y = 0.0;
    const bool car_ok = car_.latest_raw(raw_x, raw_y);

    // 视觉数据是否新鲜(与 LOCK_DROP 里同一口径)
    const bool cv_ok = lock_cv_valid_ && lock_tgt_valid_ &&
                       (now() - lock_cv_time_).seconds() <= params::LOCK_CV_TIMEOUT_S;

    // ★飞机位姿有效位★：current_x/y/z 在无位姿时返回 0(见 drone_controller 注释)，
    //   不带这个位的话监控端会把 (0,0,0) 当成"飞机真的在原点"画出来 —— 而实际是
    //   Point-LIO 没起/挂了、坐标毫无意义。这个区分对排查很关键。
    const bool pose_ok = drone_.has_pose();

    int32_t flags = 0;
    if (car_ok)  flags |= udp_tlm::F_CAR_OK;
    if (cv_ok)   flags |= udp_tlm::F_CV_OK;
    if (pose_ok) flags |= udp_tlm::F_POSE_OK;

    const bool ok = tlm_tx_->send(
        drone_.current_x(), drone_.current_y(), drone_.current_z(),
        raw_x, raw_y, telemetry_status(state_), flags);

    if (ok) {
        tlm_fail_run_ = 0;                   // 成功即清零连续失败计数
        return;
    }
    // ★失败才打印，且红色 + 节流★(需求：正常时终端不打印)
    ++tlm_fail_run_;
    if ((now_s - tlm_last_warn_) >= params::TLM_WARN_PERIOD_S) {
        tlm_last_warn_ = now_s;
        RCLCPP_ERROR(get_logger(),
            "[遥测] ★发送失败★ 连续 %u 次(累计成功 %u / 失败 %u) → %s:%d"
            "（网线/WiFi 断了? 目标不可达? 路由不通?）本机飞行不受影响",
            tlm_fail_run_, tlm_tx_->sent(), tlm_tx_->failed(),
            params::TLM_DEST_IP, params::TLM_DEST_PORT);
    }
}

}  // namespace fly_mission
