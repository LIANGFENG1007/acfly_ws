// 离线跟踪台架：真实 TrajectoryTracker + 拟合的 PX4 响应滞后 + SLAM 噪声，秒级评估控制律。
//
// 编译：tools/sim/bench/build.sh    运行：tracker_bench [--real] [--noise σp σyaw] [--seed N] [--csv DIR]
//   默认模型取自 r01 仿真拟合：角速度 延迟0.16s+τ0.22s；速度 延迟0.38s+τ0.30s；SLAM 20Hz 无噪声。
//   --real 使用更保守的"实机"模型：延迟/τ 各放大约 1.3 倍，SLAM 10Hz，位置噪声 1.5cm、航向 0.6°，
//   并叠加低频风扰。这不是实机辨识结果，只用来检验控制律对滞后和噪声是否敏感。
//
// 场景的参考轨迹走与节点相同的 smooth_catmull_rom(ds=ARC_SAMPLE_DS)。
#include "exploration_planner/bezier.hpp"
#include "motion_estimator.hpp"   // 未采用的卡尔曼估计实验，仅台架 --kf 对照
#include "exploration_planner/params.hpp"
#include "exploration_planner/trajectory_tracker.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace exploration;
namespace p = exploration::params;

namespace {

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct Model {
    double delay_v = 0.38, tau_v = 0.30, delay_yaw = 0.16, tau_yaw = 0.22;
    double slam_hz = 20.0, slam_delay = 0.03, sigma_p = 0.0, sigma_yaw = 0.0;
    double wind = 0.0;  // 风扰速度幅值(m/s)，一阶低通随机游走
    bool kf = false;    // 用 MotionEstimator(卡尔曼) 替代差分+EMA
    bool imu = false;   // 角速度用陀螺(真值+0.01 rad/s 噪声, 延迟 10 ms)
    bool kf_pose = false; // 控制输入也用滤波后的位置/航向
    double predict_lead = -1; // >=0 时控制输入用外推位姿(外推到 now+lead)
    double kf_acc = 1.5, kf_pos = 0.01; // 卡尔曼过程噪声(m/s²)/测量噪声(m)
};

TrackerGains make_gains()
{
    TrackerGains g{};
    const double d2r = M_PI / 180.0;
    g.v_max = p::V_MAX; g.v_min = p::V_MIN; g.k_curv = p::CARLIKE_K_CURV;
    g.lookahead = p::CARLIKE_LOOKAHEAD; g.endpoint_slow_r = p::ENDPOINT_SLOW_R;
    g.kp_yaw = p::KP_YAW; g.kd_yaw = p::CARLIKE_KD_YAW; g.max_yaw_rate = p::CARLIKE_MAX_YAW_RATE;
    g.kp_lat = p::KP_LAT; g.kd_lat = p::KD_LAT; g.max_v_lat = p::MAX_V_LAT;
    g.heading_gate_rad = p::CARLIKE_HEADING_GATE_DEG * d2r;
    g.forward_only = true; g.compensate_lateral_drift = true;
    g.drift_velocity_gain = p::CARLIKE_DRIFT_KD;
    g.max_accel = p::CARLIKE_MAX_ACCEL; g.max_decel = p::CARLIKE_MAX_DECEL;
    g.curvature_feedforward = p::CARLIKE_CURVATURE_FF; g.creep_speed = p::CARLIKE_CREEP_SPEED;
    g.max_yaw_accel = p::CARLIKE_MAX_YAW_ACCEL; g.yaw_filter_tau = p::CARLIKE_YAW_FILTER_S;
    g.prediction_time = p::CARLIKE_PREDICTION_S; g.lateral_prediction_time = p::CARLIKE_LATERAL_PREDICTION_S;
    g.align_resume_rad = p::CARLIKE_ALIGN_RESUME_DEG * d2r; g.align_stop_speed = p::CARLIKE_ALIGN_STOP_SPEED;
    g.align_stop_yaw_rate = p::CARLIKE_ALIGN_STOP_YAW_RATE; g.align_settle_s = p::CARLIKE_ALIGN_SETTLE_S;
    g.stop_align_rad = p::CARLIKE_STOP_ALIGN_DEG * d2r; g.corner_stop_rad = p::CARLIKE_CORNER_STOP_DEG * d2r;
    g.max_lateral_accel = p::CARLIKE_MAX_LATERAL_ACCEL;
    g.smith_time = p::CARLIKE_SMITH_S; g.smith_yaw_time = p::CARLIKE_SMITH_YAW_S;
    g.command_filter_tau = p::CARLIKE_COMMAND_FILTER_S;
    g.lateral_slow_gain = p::CARLIKE_LATERAL_SLOW_GAIN; g.lateral_slow_deadband = p::CARLIKE_LATERAL_SLOW_DEADBAND;
    g.lateral_slow_tau = p::CARLIKE_LATERAL_SLOW_TAU;
    g.stop_anchor_latency = p::CARLIKE_STOP_ANCHOR_LATENCY_S; g.stop_anchor_max_m = p::CARLIKE_STOP_ANCHOR_MAX_M;
    g.dt = p::TIMER_PERIOD_MS / 1000.0;
    return g;
}

struct Scenario {
    std::string name;
    Path2 waypoints;
    double switch_x = NAN;   // 走到 x>=此值时换路
    Path2 switch_to;         // 新路线(不含起点；起点=当前位置，用机头方向 Bezier 接入)
};

// 与节点 blend_explore_transition 相同的起步接入(简化：不做安全校验)
Path2 blend(const Vec2& p0, double yaw, const Path2& rest)
{
    const Vec2 p3 = rest.front();
    const double d03 = std::hypot(p3.x - p0.x, p3.y - p0.y);
    {   // 与节点一致：夹角超出 [TURN_BLEND_ANGLE, TURN_BLEND_MAX_ANGLE] 不构造接入曲线
        const double angle = std::abs(wrap(std::atan2(p3.y - p0.y, p3.x - p0.x) - yaw));
        if (angle < p::CARLIKE_TURN_BLEND_ANGLE_DEG * M_PI / 180 || angle > p::TURN_BLEND_MAX_ANGLE_DEG * M_PI / 180) {
            Path2 out{p0};
            out.insert(out.end(), rest.begin(), rest.end());
            return out;
        }
    }
    const double handle = std::min(p::CARLIKE_TURN_BLEND_M, d03 * 0.45);
    const Vec2 nd{(p3.x - p0.x) / d03, (p3.y - p0.y) / d03};
    const Vec2 c1{p0.x + std::cos(yaw) * handle, p0.y + std::sin(yaw) * handle};
    const Vec2 c2{p3.x - nd.x * handle, p3.y - nd.y * handle};
    Path2 out{p0};
    const int n = 24;
    for (int k = 1; k <= n; ++k) {
        const double t = double(k) / n, u = 1 - t;
        out.push_back({u*u*u*p0.x + 3*u*u*t*c1.x + 3*u*t*t*c2.x + t*t*t*p3.x,
                       u*u*u*p0.y + 3*u*u*t*c1.y + 3*u*t*t*c2.y + t*t*t*p3.y});
    }
    for (size_t i = 1; i < rest.size(); ++i) out.push_back(rest[i]);
    return out;
}

double path_distance(const Trajectory& t, double x, double y)
{
    double best = 1e9;
    for (size_t i = 1; i < t.size(); ++i) {
        const Vec2 a = t[i - 1].p, b = t[i].p;
        const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
        const double u = l2 > 1e-12 ? std::clamp(((x - a.x) * dx + (y - a.y) * dy) / l2, 0.0, 1.0) : 0.0;
        best = std::min(best, std::hypot(x - a.x - u * dx, y - a.y - u * dy));
    }
    return best;
}

struct Metrics {
    double time = 0, err_mean = 0, err_max = 0, d_yaw_rms = 0, d_v_rms = 0, acc_rms = 0, jerk_rms = 0;
    int yaw_flips = 0, lat_flips = 0, fwd_rev = 0, aligns = 0;
    double yaw_peak = 0, vlat_err = 0, min_fwd = 0;
    bool done = false;
};

Metrics run(const Scenario& sc, const Model& m, const TrackerGains& g, unsigned seed, const char* csv)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> n01(0, 1);
    const double dt = 0.002;                 // 物理步长
    const int ctrl_every = int(std::lround(g.dt / dt));
    const int slam_every = int(std::lround(1.0 / m.slam_hz / dt));
    TrajectoryTracker tr(g);
    Trajectory traj = smooth_catmull_rom(sc.waypoints, p::ARC_SAMPLE_DS);
    tr.set_trajectory(traj);
    const double yaw0 = traj.size() > 1 ? traj[1].theta : 0.0;
    // 真值
    double x = sc.waypoints.front().x, y = sc.waypoints.front().y, yaw = yaw0, vx = 0, vy = 0, r = 0;
    double wx = 0, wy = 0;
    // 指令延迟队列(世界系速度目标 + 角速度)
    std::deque<std::pair<double, Vec2>> vq; std::deque<std::pair<double, double>> rq;
    Vec2 v_target{}; double r_target = 0;
    // 节点侧估计
    struct Obs { double t, x, y, yaw; };
    std::deque<Obs> slam_q;
    bool have_prev = false; double px = x, py = y, pyaw = yaw, prev_x = x, prev_y = y, prev_yaw = yaw, prev_t = 0;
    double v_fwd_est = 0, v_lat_est = 0, yaw_rate_est = 0;
    const double alpha = p::V_EST_ALPHA;
    MotionEstimatorConfig ecfg; ecfg.accel_sigma = m.kf_acc; ecfg.pos_sigma = m.kf_pos;
    MotionEstimator est(ecfg);
    double vle_sum = 0; int vle_n = 0;
    std::deque<std::pair<double, double>> gyro_q;
    bool switched = false;
    Metrics out;
    std::vector<VelCmd> cmds;
    std::vector<Vec2> vel_hist;
    double err_sum = 0; int err_n = 0;
    bool prev_aligning = false;
    FILE* f = csv ? std::fopen(csv, "w") : nullptr;
    if (f) std::fprintf(f, "t,x,y,yaw,v_fwd_cmd,v_lat_cmd,yaw_cmd,err,heading_error,geo,lead,p,d,ff,desired,v_lat_est,v_lat_true\n");
    const double t_end = 60.0;
    int step = 0;
    for (double t = 0; t < t_end; t += dt, ++step) {
        // SLAM 采样(带噪声)，按 slam_delay 送达
        if (step % slam_every == 0)
            slam_q.push_back({t + m.slam_delay, x + m.sigma_p * n01(rng), y + m.sigma_p * n01(rng),
                              yaw + m.sigma_yaw * n01(rng)});
        while (!slam_q.empty() && slam_q.front().t <= t) {
            const Obs o = slam_q.front(); slam_q.pop_front();
            if (m.kf) {
                est.observe_pose(o.x, o.y, o.yaw, have_prev ? 1.0 / m.slam_hz : 0.0, t);
                v_fwd_est = est.v_fwd(); v_lat_est = est.v_lat(); yaw_rate_est = est.yaw_rate(t);
            } else if (have_prev) {
                const double sdt = 1.0 / m.slam_hz;
                const double ovx = (o.x - prev_x) / sdt, ovy = (o.y - prev_y) / sdt;
                const double c = std::cos(o.yaw), s = std::sin(o.yaw);
                v_fwd_est = alpha * (c * ovx + s * ovy) + (1 - alpha) * v_fwd_est;
                v_lat_est = alpha * (-s * ovx + c * ovy) + (1 - alpha) * v_lat_est;
                yaw_rate_est = alpha * wrap(o.yaw - prev_yaw) / sdt + (1 - alpha) * yaw_rate_est;
            }
            prev_x = o.x; prev_y = o.y; prev_yaw = o.yaw; have_prev = true; prev_t = o.t;
            px = o.x; py = o.y; pyaw = o.yaw;
            if (m.kf && m.kf_pose) { px = est.x(); py = est.y(); pyaw = est.yaw(); }
        }
        if (m.imu && step % 5 == 0) gyro_q.push_back({t + 0.01, r + 0.01 * n01(rng)});
        while (!gyro_q.empty() && gyro_q.front().first <= t) {
            est.observe_gyro(gyro_q.front().second, t);
            if (!m.kf) yaw_rate_est = gyro_q.front().second;
            gyro_q.pop_front();
        }
        if (m.kf) yaw_rate_est = est.yaw_rate(t);
        if (step % ctrl_every == 0) {
            if (!switched && std::isfinite(sc.switch_x) && px >= sc.switch_x) {
                switched = true;
                traj = smooth_catmull_rom(blend({px, py}, pyaw, sc.switch_to), p::ARC_SAMPLE_DS);
                tr.set_trajectory(traj);
            }
            double cx = px, cy = py, cyaw = pyaw;
            if (m.predict_lead >= 0) {
                cx = est.predict_x(t, m.predict_lead, 0.3); cy = est.predict_y(t, m.predict_lead, 0.3);
                cyaw = est.predict_yaw(t, m.predict_lead, 0.3);
            }
            VelCmd c = tr.update(cx, cy, cyaw, v_fwd_est, v_lat_est, p::GOAL_TOL_XY, yaw_rate_est);
            if (tr.reorienting() && !prev_aligning) ++out.aligns;
            prev_aligning = tr.reorienting();
            if (c.at_goal) { out.done = true; out.time = t; break; }
            if (c.needs_replan) { out.time = t; break; }
            cmds.push_back(c);
            // 主控：机体→世界，用 SLAM 航向
            const double cc = std::cos(cyaw), ss = std::sin(cyaw);
            vq.push_back({t + m.delay_v, {cc * c.v_fwd - ss * c.v_lat, ss * c.v_fwd + cc * c.v_lat}});
            rq.push_back({t + m.delay_yaw, c.yaw_rate});
            {   // 横向速度估计误差(机体系)
                const double ts = std::sin(yaw), tc = std::cos(yaw);
                vle_sum += std::pow(v_lat_est - (-ts * vx + tc * vy), 2); ++vle_n;
            }
            const double e = path_distance(traj, x, y);
            err_sum += e; ++err_n; out.err_max = std::max(out.err_max, e);
            vel_hist.push_back({vx, vy});
            if (f) {
                const auto& d = tr.debug();
                std::fprintf(f, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", t, x, y, yaw,
                             c.v_fwd, c.v_lat, c.yaw_rate, e, tr.heading_error(), d.geo_error, d.lead_error, d.p_term,
                             d.d_term, d.ff_term, d.desired, v_lat_est, -std::sin(yaw) * vx + std::cos(yaw) * vy);
            }
        }
        while (!vq.empty() && vq.front().first <= t) { v_target = vq.front().second; vq.pop_front(); }
        while (!rq.empty() && rq.front().first <= t) { r_target = rq.front().second; rq.pop_front(); }
        if (m.wind > 0) {
            wx += (-wx * 0.5 + m.wind * 1.2 * n01(rng)) * dt;
            wy += (-wy * 0.5 + m.wind * 1.2 * n01(rng)) * dt;
        }
        vx += (v_target.x + wx - vx) * dt / m.tau_v;
        vy += (v_target.y + wy - vy) * dt / m.tau_v;
        r += (r_target - r) * dt / m.tau_yaw;
        x += vx * dt; y += vy * dt; yaw = wrap(yaw + r * dt);
    }
    if (!out.done && out.time == 0) out.time = t_end;
    out.vlat_err = vle_n ? std::sqrt(vle_sum / vle_n) : 0;
    if (f) std::fclose(f);
    out.err_mean = err_n ? err_sum / err_n : 0;
    // 指令统计
    auto sign_flips = [&](auto get, double thr) {
        int n = 0, last = 0;
        for (const auto& c : cmds) {
            const double v = get(c);
            const int s = std::abs(v) > thr ? (v > 0 ? 1 : -1) : 0;
            if (s && last && s != last) ++n;
            if (s) last = s;
        }
        return n;
    };
    out.yaw_flips = sign_flips([](const VelCmd& c) { return c.yaw_rate; }, 0.05);
    out.lat_flips = sign_flips([](const VelCmd& c) { return c.v_lat; }, 0.02);
    double sy = 0, sv = 0;
    for (size_t i = 1; i < cmds.size(); ++i) {
        sy += std::pow(cmds[i].yaw_rate - cmds[i - 1].yaw_rate, 2);
        sv += std::pow(cmds[i].v_fwd - cmds[i - 1].v_fwd, 2) + std::pow(cmds[i].v_lat - cmds[i - 1].v_lat, 2);
        out.yaw_peak = std::max(out.yaw_peak, std::abs(cmds[i].yaw_rate));
        out.min_fwd = std::min(out.min_fwd, cmds[i].v_fwd);
    }
    if (cmds.size() > 1) { out.d_yaw_rms = std::sqrt(sy / (cmds.size() - 1)); out.d_v_rms = std::sqrt(sv / (cmds.size() - 1)); }
    {  // 前进指令加减速反转
        int dir = 0; double ext = cmds.empty() ? 0 : cmds[0].v_fwd;
        for (const auto& c : cmds) {
            const double v = c.v_fwd;
            if (dir >= 0 && v < ext - 0.03) { if (dir > 0) ++out.fwd_rev; dir = -1; ext = v; }
            else if (dir <= 0 && v > ext + 0.03) { if (dir < 0) ++out.fwd_rev; dir = 1; ext = v; }
            else if ((dir >= 0 && v > ext) || (dir <= 0 && v < ext)) ext = v;
        }
    }
    double sa = 0, sj = 0; int na = 0;
    Vec2 pa{};
    for (size_t i = 1; i < vel_hist.size(); ++i) {
        const Vec2 a{(vel_hist[i].x - vel_hist[i - 1].x) / g.dt, (vel_hist[i].y - vel_hist[i - 1].y) / g.dt};
        sa += a.x * a.x + a.y * a.y;
        if (i > 1) sj += std::pow((a.x - pa.x) / g.dt, 2) + std::pow((a.y - pa.y) / g.dt, 2);
        pa = a; ++na;
    }
    if (na > 1) { out.acc_rms = std::sqrt(sa / na); out.jerk_rms = std::sqrt(sj / (na - 1)); }
    return out;
}

}  // namespace

// --set name=value 覆盖 TrackerGains 字段(台架调参用)
bool set_gain(TrackerGains& g, const std::string& kv)
{
    const auto eq = kv.find('=');
    if (eq == std::string::npos) return false;
    const std::string k = kv.substr(0, eq);
    const double v = std::atof(kv.c_str() + eq + 1);
    struct F { const char* n; double TrackerGains::* m; };
    static const F fields[] = {
        {"v_max", &TrackerGains::v_max}, {"k_curv", &TrackerGains::k_curv}, {"lookahead", &TrackerGains::lookahead},
        {"endpoint_slow_r", &TrackerGains::endpoint_slow_r}, {"kp_yaw", &TrackerGains::kp_yaw},
        {"kd_yaw", &TrackerGains::kd_yaw}, {"max_yaw_rate", &TrackerGains::max_yaw_rate},
        {"kp_lat", &TrackerGains::kp_lat}, {"max_v_lat", &TrackerGains::max_v_lat},
        {"heading_gate_rad", &TrackerGains::heading_gate_rad}, {"drift_velocity_gain", &TrackerGains::drift_velocity_gain},
        {"max_accel", &TrackerGains::max_accel}, {"max_decel", &TrackerGains::max_decel},
        {"curvature_feedforward", &TrackerGains::curvature_feedforward}, {"creep_speed", &TrackerGains::creep_speed},
        {"max_yaw_accel", &TrackerGains::max_yaw_accel}, {"prediction_time", &TrackerGains::prediction_time},
        {"lateral_prediction_time", &TrackerGains::lateral_prediction_time},
        {"max_lateral_accel", &TrackerGains::max_lateral_accel}, {"stop_align_rad", &TrackerGains::stop_align_rad},
        {"ydw", &TrackerGains::yaw_damping_command_weight}, {"cft", &TrackerGains::command_filter_tau},
        {"lsg", &TrackerGains::lateral_slow_gain}, {"lsd", &TrackerGains::lateral_slow_deadband}, {"lst", &TrackerGains::lateral_slow_tau},
        {"sal", &TrackerGains::stop_anchor_latency}, {"sam", &TrackerGains::stop_anchor_max_m},
        {"smith", &TrackerGains::smith_time}, {"smith_yaw", &TrackerGains::smith_yaw_time},
    };
    for (const auto& f : fields) if (k == f.n) { g.*(f.m) = v; return true; }
    if (k == "compensate_lateral_drift") { g.compensate_lateral_drift = v != 0; return true; }
    return false;
}

int main(int argc, char** argv)
{
    TrackerGains g = make_gains();
    Model m;
    unsigned seed = 1;
    const char* csv_dir = nullptr;
    int repeats = 1;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--real")) {
            m.delay_v = 0.50; m.tau_v = 0.40; m.delay_yaw = 0.22; m.tau_yaw = 0.30;
            m.slam_hz = 10.0; m.slam_delay = 0.05; m.sigma_p = 0.015; m.sigma_yaw = 0.6 * M_PI / 180.0; m.wind = 0.10;
            repeats = 5;
        } else if (!std::strcmp(argv[i], "--noise") && i + 2 < argc) {
            m.sigma_p = std::atof(argv[++i]); m.sigma_yaw = std::atof(argv[++i]) * M_PI / 180.0; repeats = 5;
        } else if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) seed = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--csv") && i + 1 < argc) csv_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--set") && i + 1 < argc) {
            if (!set_gain(g, argv[++i])) { std::fprintf(stderr, "unknown gain %s\n", argv[i]); return 1; }
        }
        else if (!std::strcmp(argv[i], "--predict") && i + 1 < argc) { m.kf = true; m.predict_lead = std::atof(argv[++i]); }
        else if (!std::strcmp(argv[i], "--wind") && i + 1 < argc) m.wind = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--kfacc") && i + 1 < argc) { m.kf = true; m.kf_acc = std::atof(argv[++i]); }
        else if (!std::strcmp(argv[i], "--kfpos") && i + 1 < argc) m.kf_pos = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--legacy")) {   // 2026-09-23 调优前的跟踪参数(对照)
            g.smith_time = g.smith_yaw_time = g.command_filter_tau = 0.0;
            g.drift_velocity_gain = 0.80; g.lateral_slow_gain = 3.0; g.lateral_slow_deadband = g.lateral_slow_tau = 0.0;
        }
        else if (!std::strcmp(argv[i], "--lag") && i + 1 < argc) {   // 放大全部响应滞后(延迟与时间常数)
            const double k = std::atof(argv[++i]);
            m.delay_v *= k; m.tau_v *= k; m.delay_yaw *= k; m.tau_yaw *= k;
        }
        else if (!std::strcmp(argv[i], "--kf")) m.kf = true;
        else if (!std::strcmp(argv[i], "--imu")) m.imu = true;
        else if (!std::strcmp(argv[i], "--kfpose")) { m.kf = true; m.kf_pose = true; }
    }
    const std::vector<Scenario> scenarios = {
        {"turn90", {{0, 0}, {2.5, 0}, {2.5, 2.5}}},
        {"s_bend", {{0, 0}, {1.5, 0}, {2.3, 0.7}, {3.3, 0.7}, {4.1, 0}, {5.5, 0}}},
        {"kink45", {{0, 0}, {2.0, 0}, {4.0, 2.0}}},
        {"straight", {{0, 0}, {5.0, 0}}},
        {"switch40", {{0, 0}, {5, 0}}, 1.5, {{3.2, 1.2}, {4.0, 2.8}}},
        {"switch70", {{0, 0}, {5, 0}}, 1.5, {{2.4, 1.6}, {2.6, 3.2}}},
        {"switch110", {{0, 0}, {5, 0}}, 1.5, {{0.9, 1.6}, {0.2, 3.0}}},
        {"switch85", {{0, 0}, {5, 0}}, 1.5, {{1.8, 1.8}, {1.8, 3.5}}},
    };
    std::printf("%-9s %6s %6s %6s %5s %5s %5s %6s %6s %6s %6s %6s %3s\n", "scene", "time", "errAvg", "errMax",
                "yawFl", "latFl", "fwdRv", "dYaw", "dV", "yawPk", "acc", "jerk", "al");
    Metrics total{};
    for (const auto& sc : scenarios) {
        Metrics avg{};
        for (int k = 0; k < repeats; ++k) {
            std::string path = csv_dir ? std::string(csv_dir) + "/" + sc.name + ".csv" : "";
            Metrics r = run(sc, m, g, seed + k, csv_dir && k == 0 ? path.c_str() : nullptr);
            avg.time += r.time / repeats; avg.err_mean += r.err_mean / repeats; avg.err_max = std::max(avg.err_max, r.err_max);
            avg.yaw_flips += r.yaw_flips; avg.lat_flips += r.lat_flips; avg.fwd_rev += r.fwd_rev; avg.aligns += r.aligns;
            avg.d_yaw_rms += r.d_yaw_rms / repeats; avg.d_v_rms += r.d_v_rms / repeats;
            avg.yaw_peak = std::max(avg.yaw_peak, r.yaw_peak);
            avg.acc_rms += r.acc_rms / repeats; avg.jerk_rms += r.jerk_rms / repeats;
            avg.done = r.done; avg.vlat_err += r.vlat_err / repeats; avg.min_fwd = std::min(avg.min_fwd, r.min_fwd);
        }
        std::printf("%-9s %6.2f %6.3f %6.3f %5.1f %5.1f %5.1f %6.4f %6.4f %6.3f %6.3f %6.2f %3.1f%s\n", sc.name.c_str(),
                    avg.time, avg.err_mean, avg.err_max, double(avg.yaw_flips) / repeats, double(avg.lat_flips) / repeats,
                    double(avg.fwd_rev) / repeats, avg.d_yaw_rms, avg.d_v_rms, avg.yaw_peak, avg.acc_rms, avg.jerk_rms,
                    double(avg.aligns) / repeats, avg.done ? "" : " (未完成)");
        if (avg.aligns > 0) std::printf("          最小前进指令 %.3f\n", avg.min_fwd);
        total.time += avg.time; total.err_mean += avg.err_mean / scenarios.size();
        total.yaw_flips += avg.yaw_flips; total.lat_flips += avg.lat_flips; total.fwd_rev += avg.fwd_rev;
        total.d_yaw_rms += avg.d_yaw_rms / scenarios.size(); total.jerk_rms += avg.jerk_rms / scenarios.size();
        total.err_max = std::max(total.err_max, avg.err_max); total.vlat_err += avg.vlat_err / scenarios.size();
    }
    std::printf("TOTAL     %6.2f %6.3f %6.3f %5.1f %5.1f %5.1f %6.4f %6s %6s %6s %6.2f vlatErr=%.3f\n", total.time, total.err_mean,
                total.err_max, double(total.yaw_flips) / repeats, double(total.lat_flips) / repeats,
                double(total.fwd_rev) / repeats, total.d_yaw_rms, "", "", "", total.jerk_rms, total.vlat_err);
    return 0;
}
