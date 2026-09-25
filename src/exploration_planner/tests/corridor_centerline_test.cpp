// 按宽度对中模式闭环测试：探索终点到隧道的距离不固定、H 的 x 偏离隧道中线、门偏心、
// 场地里有另一对平行墙干扰、门间距短；横向涡流扰动 + 0.1 s 指令延迟。
// 参数取 params.hpp(与节点一致)，只验证逻辑：Y 锁定横移到中线、门外保持中线、
// 门前 CORRIDOR_GATE_ALIGN_M 内才对准门、前进不慢、全程不碰墙、到 H。
#include "exploration_planner/corridor_controller.hpp"
#include "exploration_planner/params.hpp"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

using namespace exploration;
namespace p = exploration::params;

namespace {

struct Box { double x0, x1, y0, y1; };

struct Scene {
    std::string name;
    double center_x = 9.60;   // 真实隧道中线 x(控制器不知道)
    double width = 1.30;      // 两墙内表面间距
    double wall = 0.05;
    double top = 4.6, bottom = -2.3;   // 隧道北/南端内表面
    double lock_y = 4.0;      // 西墙侧门开口中心 = 探索终点 y
    double opening = 1.2;
    double red_distance = 1.9;  // 探索终点在隧道中线西侧多远
    double h_bias = 0.0;        // H 的 x 相对中线的偏差
    double h_y = -1.4;
    double gap = 0.85, door_depth = 0.04;
    std::vector<std::pair<double, double>> doors;  // (门 y, 门洞中心相对中线的偏移)
    std::vector<Box> extra;
    double gust_scale = 1.0;   // 涡流强度倍率(仿真同款场景对应 Gazebo，扰动弱)

    std::vector<Box> boxes() const {
        const double west = center_x - width / 2, east = center_x + width / 2;
        std::vector<Box> b{
            {west - wall, west, bottom, lock_y - opening / 2},
            {west - wall, west, lock_y + opening / 2, top},
            {east, east + wall, bottom, top},
            {west - wall, east + wall, top, top + wall},
            {west - wall, east + wall, bottom - wall, bottom},
        };
        for (const auto& [y, offset] : doors) {
            const double gl = center_x + offset - gap / 2, gr = center_x + offset + gap / 2;
            b.push_back({west, gl, y - door_depth / 2, y + door_depth / 2});
            b.push_back({gr, east, y - door_depth / 2, y + door_depth / 2});
        }
        b.insert(b.end(), extra.begin(), extra.end());
        return b;
    }
};

// 水平首次命中激光：720 线，0.1~6 m，带测距噪声
Path2 scan(const std::vector<Box>& boxes, const Vec2& o, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, 0.01);
    Path2 points;
    for (int i = 0; i < 720; ++i) {
        const double a = -M_PI + 2.0 * M_PI * i / 720.0;
        const double dx = std::cos(a), dy = std::sin(a);
        double best = 6.0;
        bool hit = false;
        for (const auto& b : boxes) {
            double t0 = -std::numeric_limits<double>::infinity(), t1 = std::numeric_limits<double>::infinity();
            const double lo[2]{b.x0, b.y0}, hi[2]{b.x1, b.y1}, org[2]{o.x, o.y}, dir[2]{dx, dy};
            bool miss = false;
            for (int k = 0; k < 2; ++k) {
                if (std::abs(dir[k]) < 1e-12) {
                    if (org[k] < lo[k] || org[k] > hi[k]) miss = true;
                    continue;
                }
                double a0 = (lo[k] - org[k]) / dir[k], a1 = (hi[k] - org[k]) / dir[k];
                if (a0 > a1) std::swap(a0, a1);
                t0 = std::max(t0, a0);
                t1 = std::min(t1, a1);
            }
            if (miss || t0 > t1 || t0 < 0.0) continue;
            if (t0 < best) { best = t0; hit = true; }
        }
        if (!hit || best < 0.1) continue;
        points.push_back({o.x + dx * best + noise(rng), o.y + dy * best + noise(rng)});
    }
    return points;
}

double clearance(const std::vector<Box>& boxes, const Vec2& q) {
    double d = std::numeric_limits<double>::infinity();
    for (const auto& b : boxes) {
        const double dx = std::max({b.x0 - q.x, 0.0, q.x - b.x1});
        const double dy = std::max({b.y0 - q.y, 0.0, q.y - b.y1});
        d = std::min(d, std::hypot(dx, dy));
    }
    return d;
}

CorridorConfig makeConfig() {
    const double d2r = M_PI / 180.0;
    CorridorConfig c;
    auto& pc = c.perception;
    pc.corridor_width = p::CORRIDOR_WIDTH;
    pc.robot_width = p::CORRIDOR_ROBOT_WIDTH;
    pc.minimum_gap_extra = p::CORRIDOR_MIN_GAP_EXTRA;
    pc.cell_size = p::CORRIDOR_CELL;
    pc.lookahead = p::CORRIDOR_SENSOR_RANGE;
    pc.lookbehind = p::CORRIDOR_LOOKBEHIND;
    pc.wall_search_tolerance = p::CORRIDOR_WALL_SEARCH_M;
    pc.wall_exclusion_band = p::CORRIDOR_WALL_EXCLUSION_M;
    pc.wall_min_span = p::CORRIDOR_WALL_MIN_SPAN;
    pc.wall_min_points = p::CORRIDOR_WALL_MIN_POINTS;
    pc.max_wall_gap = p::CORRIDOR_MAX_WALL_GAP;
    pc.gate_min_span = p::CORRIDOR_GATE_MIN_SPAN;
    pc.gate_cluster_depth = p::CORRIDOR_GATE_DEPTH_CLUSTER;
    pc.gate_max_depth = p::CORRIDOR_GATE_MAX_DEPTH;
    pc.surface_sample_gap = p::CORRIDOR_SURFACE_SAMPLE_GAP;
    pc.gate_min_points = p::CORRIDOR_GATE_MIN_POINTS;
    pc.gate_min_clear_rays = p::CORRIDOR_GATE_MIN_CLEAR_RAYS;
    pc.endpoint_exclusion = p::CORRIDOR_ENDPOINT_EXCLUSION;
    pc.min_observed_ahead = p::CORRIDOR_MIN_OBSERVED_AHEAD;
    pc.width_tolerance = p::CORRIDOR_WIDTH_TOL_M;
    pc.center_search_range = p::CORRIDOR_CENTER_SEARCH_M;
    c.initial_yaw = p::CORRIDOR_INITIAL_YAW_DEG * d2r;
    c.entry_speed = p::CORRIDOR_ENTRY_SPEED;
    c.cruise_speed = p::CORRIDOR_CRUISE_SPEED;
    c.gate_speed = p::CORRIDOR_GATE_SPEED;
    c.align_speed = p::CORRIDOR_ALIGN_SPEED;
    c.yaw_kp = p::CORRIDOR_YAW_KP;
    c.max_yaw_rate = p::CORRIDOR_MAX_YAW_RATE;
    c.yaw_accel = p::CORRIDOR_YAW_ACCEL;
    c.acceleration = p::CORRIDOR_ACCEL;
    c.position_kp = p::CORRIDOR_POSITION_KP;
    c.velocity_kd = p::CORRIDOR_VELOCITY_KD;
    c.velocity_filter_tau = p::CORRIDOR_VELOCITY_FILTER_S;
    c.arrival_hysteresis = p::CORRIDOR_ARRIVAL_HYSTERESIS;
    c.heading_tolerance = p::CORRIDOR_YAW_TOL_DEG * d2r;
    c.heading_stop = p::CORRIDOR_HEADING_STOP_DEG * d2r;
    c.point_tolerance = p::CORRIDOR_POINT_TOL;
    c.settle_time = p::CORRIDOR_SETTLE_S;
    c.lookahead = p::CORRIDOR_LOOKAHEAD;
    c.moving_lookahead = p::CORRIDOR_MOVING_LOOKAHEAD;
    c.continuous_time_margin = p::CORRIDOR_CONTINUOUS_TIME_MARGIN;
    c.gate_association = p::CORRIDOR_GATE_ASSOC_M;
    c.confirm_frames = p::CORRIDOR_CONFIRM_FRAMES;
    c.cloud_timeout = p::CORRIDOR_CLOUD_TIMEOUT_S;
    c.cloud_window = p::CORRIDOR_CLOUD_WINDOW_S;
    c.centerline_mode = true;
    c.gate_align_distance = p::CORRIDOR_GATE_ALIGN_M;
    // SWEEP_KP/SWEEP_KD/SWEEP_SEED 环境变量只供增益扫描，平时用 params 值
    c.center_kp = std::getenv("SWEEP_KP") ? std::atof(std::getenv("SWEEP_KP")) : p::CORRIDOR_CENTER_KP;
    c.center_kd = std::getenv("SWEEP_KD") ? std::atof(std::getenv("SWEEP_KD")) : p::CORRIDOR_CENTER_KD;
    c.lateral_max_speed = p::CORRIDOR_LATERAL_MAX_SPEED;
    c.lateral_acceleration = p::CORRIDOR_LATERAL_ACCEL;
    c.lateral_latency = p::CORRIDOR_LATERAL_LATENCY_S;
    c.lateral_brake_accel = p::CORRIDOR_LATERAL_BRAKE_ACCEL;
    c.smith_time = std::getenv("SWEEP_SMITH") ? std::atof(std::getenv("SWEEP_SMITH")) : p::CORRIDOR_SMITH_S;
    if (std::getenv("SWEEP_LATENCY")) c.lateral_latency = std::atof(std::getenv("SWEEP_LATENCY"));
    if (std::getenv("SWEEP_BRAKE")) c.lateral_brake_accel = std::atof(std::getenv("SWEEP_BRAKE"));
    c.wall_latency = std::getenv("SWEEP_WALL_T") ? std::atof(std::getenv("SWEEP_WALL_T")) : p::CORRIDOR_WALL_LATENCY_S;
    c.wall_brake_accel = std::getenv("SWEEP_WALL_A") ? std::atof(std::getenv("SWEEP_WALL_A")) : p::CORRIDOR_WALL_BRAKE_ACCEL;
    c.min_forward_speed = p::CORRIDOR_MIN_FORWARD_SPEED;
    c.center_filter_time = p::CORRIDOR_CENTER_FILTER_S;
    c.entry_center_tolerance = p::CORRIDOR_ENTRY_CENTER_TOL;
    return c;
}

bool g_verbose = false;

// 飞机响应模型：乐观(仿真量级) / 保守(实机量级：延迟、响应更慢，涡流更大)
// accel_limit>0：飞控速度环的加速度上限(m/s²)，PX4 Gazebo 横向实测约 0.3(0→0.25 m/s 约 1 s，刹车同样)
struct Dynamics { const char* name; int delay_steps; double tau; double gust; double accel_limit = 0.0; };

bool run(const Scene& scene, const Dynamics& dyn, unsigned seed) {
    const auto boxes = scene.boxes();
    CorridorConfig config = makeConfig();
    config.perception.corridor_width = scene.width;   // 参数里填的就是实际隧道宽度
    CorridorController controller(config);
    const Vec2 red{scene.center_x - scene.red_distance, scene.lock_y};
    const Vec2 h{scene.center_x + scene.h_bias, scene.h_y};
    if (!controller.configure(red, h)) { std::cerr << scene.name << " FAIL: configure\n"; return false; }
    controller.start(red, 0.0);
    std::mt19937 rng(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    const double dt = 0.02, radius = p::CORRIDOR_ROBOT_WIDTH * 0.5;
    Vec2 pos = red;
    double yaw = p::CORRIDOR_INITIAL_YAW_DEG * M_PI / 180.0, vx = 0, vy = 0, yaw_rate = 0, gust = 0;
    std::deque<CorridorCommand> delay(dyn.delay_steps);
    double min_clear = 1e9, next_cloud = 0.0;
    bool finished = false, entry_done = false, ok = true;
    Vec2 entry_pos{};
    double center_sq = 0.0, fwd_sum = 0.0; int center_n = 0;
    double max_center_estimate_error = 0.0;
    std::vector<double> align_start(scene.doors.size(), -1.0), cross_error(scene.doors.size(), -1.0);
    int aligning_outside_zone = 0;
    CorridorCommand cmd;
    int step = 0;
    for (; step < 8000; ++step) {
        const double now = step * dt;
        if (now + 1e-9 >= next_cloud) { controller.observe(scan(boxes, pos, rng), now, pos); next_cloud += 0.1; }
        const double vf = std::cos(yaw) * vx + std::sin(yaw) * vy;
        const double vl = -std::sin(yaw) * vx + std::cos(yaw) * vy;
        cmd = controller.update(pos, yaw, vf, vl, now, true);
        if (cmd.finished) { finished = true; break; }
        const auto phase = controller.phase();
        if (g_verbose && step % 5 == 0 && pos.y < 3.4 && pos.y > 1.8)
            std::cout << "  t=" << now << " pos=(" << pos.x << ',' << pos.y << ") v=(" << vx << ',' << vy
                      << ") cmd f=" << cmd.forward << " l=" << cmd.lateral << " tgt=(" << cmd.target.x << ','
                      << cmd.target.y << ") " << cmd.status << " | obs=" << controller.observation().reason
                      << " gate_s=" << controller.observation().gate_s << " pass=" << controller.observation().gate_passable
                      << " locked=" << controller.gateLocked() << " door_s=" << controller.gate().gate_s
                      << " door_reason=" << controller.gate().reason << '\n';
        if (!entry_done && phase == CorridorPhase::Search) { entry_done = true; entry_pos = pos; }
        if (phase == CorridorPhase::Search && controller.centerLocked()) {
            max_center_estimate_error = std::max(max_center_estimate_error,
                std::abs(controller.centerWorld(pos).x - scene.center_x));
            // 离所有门都超过 align+0.2 m：必须保持中线
            bool far = true;
            for (std::size_t i = 0; i < scene.doors.size(); ++i) {
                const double y = scene.doors[i].first;
                if (pos.y < y + scene.door_depth / 2 + p::CORRIDOR_GATE_ALIGN_M + 0.4 &&
                    pos.y > y - scene.door_depth / 2 - p::CORRIDOR_GATE_ALIGN_M - 0.4) far = false;
            }
            if (far && pos.y < scene.lock_y - 0.5 && pos.y > scene.h_y + 0.6) {
                center_sq += std::pow(pos.x - scene.center_x, 2);
                fwd_sum += -vy;
                ++center_n;
                if (cmd.status == "Aligning to door center") ++aligning_outside_zone;
            }
        }
        for (std::size_t i = 0; i < scene.doors.size(); ++i) {
            const double front = scene.doors[i].first + scene.door_depth / 2;
            if (align_start[i] < 0.0 && cmd.status == "Aligning to door center" &&
                pos.y > front - 0.05 && pos.y - front < 1.5 && controller.gatesPassed() == static_cast<int>(i))
                align_start[i] = pos.y - front;
            if (cross_error[i] < 0.0 && pos.y <= scene.doors[i].first)
                cross_error[i] = std::abs(pos.x - (scene.center_x + scene.doors[i].second));
        }
        // 动力学：指令延迟 → 一阶响应；横向涡流(世界 x 向低频扰动 ~0.05 m/s)
        delay.push_back(cmd);
        const auto applied = delay.front();
        delay.pop_front();
        gust += (-gust / 0.8) * dt + dyn.gust * scene.gust_scale * std::sqrt(dt) * n01(rng);
        const double k = 1.0 - std::exp(-dt / dyn.tau);
        const double tvx = std::cos(yaw) * applied.forward - std::sin(yaw) * applied.lateral + gust;
        const double tvy = std::sin(yaw) * applied.forward + std::cos(yaw) * applied.lateral;
        double ax = k * (tvx - vx) / dt, ay = k * (tvy - vy) / dt;
        if (dyn.accel_limit > 0.0) {
            const double a = std::hypot(ax, ay);
            if (a > dyn.accel_limit) { ax *= dyn.accel_limit / a; ay *= dyn.accel_limit / a; }
        }
        vx += ax * dt;
        vy += ay * dt;
        yaw_rate += k * (applied.yaw_rate - yaw_rate);
        pos.x += vx * dt;
        pos.y += vy * dt;
        yaw += yaw_rate * dt;
        min_clear = std::min(min_clear, clearance(boxes, pos));
        if (clearance(boxes, pos) <= radius - 0.02) {
            std::cerr << scene.name << " FAIL: body hits wall at (" << pos.x << ',' << pos.y << ") phase="
                      << static_cast<int>(phase) << " status=" << cmd.status << '\n';
            return false;
        }
    }
    const double center_rms = center_n ? std::sqrt(center_sq / center_n) : 1e9;
    const double fwd_mean = center_n ? fwd_sum / center_n : 0.0;
    std::cout << scene.name << "[" << dyn.name << "]: finished=" << finished << " t=" << step * dt << "s gates=" << controller.gatesPassed()
              << " entry_end=(" << entry_pos.x << ',' << entry_pos.y << ") 中线=" << scene.center_x
              << " 中线估计最大误差=" << max_center_estimate_error << " 门外中线RMS=" << center_rms
              << " 门外平均前进=" << fwd_mean << " 最小净距=" << min_clear - radius
              << " end=(" << pos.x << ',' << pos.y << ") status=" << cmd.status << '\n';
    for (std::size_t i = 0; i < scene.doors.size(); ++i)
        std::cout << "   门" << i + 1 << " 开始对准时离门前沿=" << align_start[i] << "m 过门横向误差=" << cross_error[i] << "m\n";
    auto check = [&](bool condition, const std::string& what) {
        if (!condition) { std::cerr << scene.name << "[" << dyn.name << "] FAIL: " << what << '\n'; ok = false; }
    };
    check(finished, "未到达 H");
    check(controller.gatesPassed() == static_cast<int>(scene.doors.size()), "过门数不对");
    check(entry_done && std::abs(entry_pos.x - scene.center_x) <= 0.07, "横移结束时不在隧道中线");
    check(entry_done && std::abs(entry_pos.y - scene.lock_y) <= 0.06, "横移时 Y 没锁住");
    check(max_center_estimate_error <= 0.06, "雷达中线估计误差过大");
    check(center_rms <= 0.10, "门外没有保持中线");   // 强涡流+0.4 s 延迟下的实际水平留余量
    check(fwd_mean >= 0.22, "门外前进太慢");
    check(aligning_outside_zone == 0, "离门还远就在对准门");
    check(std::abs(pos.y - scene.h_y) <= 0.12, "没停在 H 的纵向位置");
    for (std::size_t i = 0; i < scene.doors.size(); ++i) {
        const double door_margin = 0.5 * (scene.gap - p::CORRIDOR_ROBOT_WIDTH);
        check(cross_error[i] >= 0.0 && cross_error[i] <= std::min(0.09, door_margin), "过门时没对准门" + std::to_string(i + 1));
        check(align_start[i] >= 0.0 && align_start[i] <= p::CORRIDOR_GATE_ALIGN_M + 0.1, "对准门太早" + std::to_string(i + 1));
    }
    return ok;
}

}  // namespace

int main(int argc, char** argv) {
    g_verbose = argc > 1;
    std::vector<Scene> scenes;
    Scene base;
    base.name = "基本(终点离中线1.9m)";
    base.doors = {{2.5, 0.10}, {0.3, -0.10}};
    base.h_bias = 0.07;
    scenes.push_back(base);

    Scene far = base;
    far.name = "终点离中线3.2m+H偏0.25m";
    far.red_distance = 3.2;
    far.h_bias = -0.25;
    scenes.push_back(far);

    Scene near = base;
    near.name = "终点贴近隧道(1.3m)";
    near.red_distance = 1.3;
    scenes.push_back(near);

    Scene decoy = base;
    decoy.name = "西侧有另一对平行墙干扰";
    {
        const double west = base.center_x - base.width / 2;
        decoy.extra = {{west - 2.05, west - 2.0, -2.0, 4.5}, {west - 3.40, west - 3.35, -2.0, 4.5}};
        decoy.red_distance = 1.9;
    }
    scenes.push_back(decoy);

    Scene close_doors = base;
    close_doors.name = "两门间距1.0m";
    close_doors.doors = {{2.2, 0.10}, {1.2, -0.08}};
    scenes.push_back(close_doors);

    Scene wide_offset = base;
    wide_offset.name = "门偏心0.18m";
    wide_offset.doors = {{2.5, 0.18}, {0.3, -0.18}};
    scenes.push_back(wide_offset);

    // 仿真 arena 同款：宽 1.40、门洞 0.75(机身 0.64，每侧仅 5.5 cm)、S 形偏心 ±0.325，门洞一侧就是隧道墙
    Scene arena = base;
    arena.name = "仿真同款S门(门洞0.75偏心0.325)";
    arena.width = 1.40;
    arena.gap = 0.75;
    arena.doors = {{2.5, 0.325}, {0.0, -0.325}};
    arena.h_bias = -0.15;
    arena.gust_scale = 0.3;
    scenes.push_back(arena);

    bool ok = true;
    unsigned seed = std::getenv("SWEEP_SEED") ? std::atoi(std::getenv("SWEEP_SEED")) : 7;
    const Dynamics models[] = {{"乐观", 5, 0.15, 0.06}, {"保守", 20, 0.30, 0.09}, {"PX4横向", 10, 0.20, 0.06, 0.30}};
    for (const auto& dyn : models) {
        for (const auto& scene : scenes) {
            if (argc > 1 && scene.name.find(argv[1]) == std::string::npos) { ++seed; continue; }
            ok = run(scene, dyn, seed++) && ok;
        }
    }
    std::cout << (ok ? "ALL PASS" : "SOME FAILED") << '\n';
    return ok ? 0 : 1;
}
