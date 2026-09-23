// 离线探索台架：真实 GridMap(视野/遮挡) + select_observation_target(选点) + plan_global_path(机头锥 A*)，
// 运动理想化，几秒跑完整个探索，用来比较【选点策略】而不是控制律。
//
// 编译：tools/sim/bench/build.sh   运行：explore_bench [--set 名=值 ...] layouts/L1.json [L2.json ...]
// 时间模型(按 r01~r20 仿真粗标)：巡航 0.42 m/s；路径折角 <45° 不额外计时；45°~75° 计 角度/0.8×0.5 s 减速；
//   ≥75° 视为停车转向：刹车+起步 1.5 s + 角度/0.7 rad/s。途中每 0.5 m 检查目标是否已无未扫格(view_exhausted)。
// 障碍全部已知(真实系统逐步发现)，所以只看相对差异。总时间含归航到红点(7,4.25)。
// 2026-09-23 结论：终点引导(后期选点加 w·到红点距离)最多 -3% 且对权重敏感、增加停车转向，未采用；
//   "先远后近"(w·(覆盖率-pivot)·到红点距离)同样最多 -3.3%，pivot 稍变即转负，未采用。
#include "exploration_planner/coverage_planner.hpp"
#include "exploration_planner/global_planner.hpp"
#include "exploration_planner/grid_map.hpp"
#include "exploration_planner/params.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

using namespace exploration;
namespace p = exploration::params;

namespace {

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

Obstacles load_layout(const std::string& path)
{
    std::ifstream in(path);
    std::stringstream ss; ss << in.rdbuf();
    const std::string text = ss.str();
    const auto start = text.find("\"obstacles\"");
    Obstacles out;
    if (start == std::string::npos) return out;
    const auto end = text.find(']', start);
    const std::string block = text.substr(start, end - start);
    std::regex obj(R"(\{[^}]*\})");
    std::regex num_x(R"("x"\s*:\s*(-?[0-9.]+))"), num_y(R"("y"\s*:\s*(-?[0-9.]+))"), num_r(R"("r"\s*:\s*(-?[0-9.]+))");
    for (auto it = std::sregex_iterator(block.begin(), block.end(), obj); it != std::sregex_iterator(); ++it) {
        const std::string o = it->str();
        std::smatch m;
        Obstacle ob{};
        if (std::regex_search(o, m, num_x)) ob.cx = std::stod(m[1]);
        if (std::regex_search(o, m, num_y)) ob.cy = std::stod(m[1]);
        ob.r = std::regex_search(o, m, num_r) ? std::stod(m[1]) : 0.2;
        ob.r += p::OBS_R_INFLATE;   // 与节点障碍拟合充气一致
        out.push_back(ob);
    }
    return out;
}

struct Result { double time = 0, explore_time = 0, length = 0, home_length = 0, stop_turns = 0, selects = 0, exhausted = 0, coverage = 0; bool done = false; };

Result run(const Obstacles& obs, FrontierConfig fcfg, GlobalConfig gcfg, GridConfig grid_cfg,
           double coverage_goal, bool verbose)
{
    GridMap grid(grid_cfg);
    Vec2 pos{0, 0};
    double yaw = 0, t = 0;
    int band = -1;
    std::vector<Vec2> unreachable;
    Result r;
    const double v = 0.42, step = 0.05;
    grid.mark_scan(pos.x, pos.y, yaw, obs);
    grid.fill_obstacle_cells(obs);
    double route_heading = std::numeric_limits<double>::quiet_NaN();
    int stuck = 0;
    while (grid.coverage_ratio() < coverage_goal && t < 600) {
        auto snap = grid.snapshot();
        int cand_band = band;
        FrontierSelection sel = select_observation_target(snap, fcfg, pos, yaw, cand_band, obs,
                                                          &unreachable, p::UNREACH_BLOCK_R, route_heading);
        if (!sel.valid) {
            if (unreachable.empty() || ++stuck > 3) break;
            unreachable.clear();
            continue;
        }
        ++r.selects;
        GlobalConfig cfg = gcfg;
        cfg.validate_complete_path = true;
        GlobalResult gr = plan_global_path(pos, sel.point, obs, cfg, yaw);
        if (!gr.ok) gr = plan_global_path(pos, sel.point, obs, cfg);
        if (!gr.ok || gr.path.size() < 2) { unreachable.push_back(sel.point); continue; }
        band = cand_band;
        // 沿折线飞：每个顶点按折角计时
        double heading = yaw;
        bool abandoned = false;
        double since_check = 0;
        for (size_t i = 1; i < gr.path.size() && !abandoned; ++i) {
            const Vec2 a = gr.path[i - 1], b = gr.path[i];
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            if (len < 1e-6) continue;
            const double seg = std::atan2(b.y - a.y, b.x - a.x);
            const double turn = std::abs(wrap(seg - heading));
            if (turn >= 75 * M_PI / 180) { t += 1.5 + turn / 0.7; ++r.stop_turns; }
            else if (turn >= 45 * M_PI / 180) t += turn / 0.8 * 0.5;
            heading = seg;
            for (double s = step; s <= len + 1e-9; s += step) {
                pos = {a.x + (b.x - a.x) * s / len, a.y + (b.y - a.y) * s / len};
                t += step / v; r.length += step; since_check += step;
                grid.mark_scan(pos.x, pos.y, heading, obs);
                if (since_check >= 0.5) {
                    since_check = 0;
                    auto s2 = grid.snapshot();
                    if (visible_unknown_count(s2, sel.point, sel.view_heading, obs, fcfg) == 0 &&
                        std::hypot(sel.point.x - pos.x, sel.point.y - pos.y) > 0.3) {
                        ++r.exhausted; abandoned = true; break;
                    }
                }
                if (grid.coverage_ratio() >= coverage_goal) { abandoned = true; break; }
            }
        }
        yaw = heading;
        route_heading = abandoned ? heading : std::numeric_limits<double>::quiet_NaN();
        if (!abandoned) unreachable.push_back(sel.point);   // 与节点一致：观察过的点不再选
        if (verbose) std::printf("  t=%6.1f 目标(%5.2f,%5.2f) 覆盖%.2f %s\n", t, sel.point.x, sel.point.y,
                                 grid.coverage_ratio(), abandoned ? "途中换" : "到达");
    }
    r.explore_time = t;
    // 归航：沿机头锥 A* 飞到红点，同样按折角计时
    {
        GlobalConfig cfg = gcfg;
        cfg.validate_complete_path = true;
        const Vec2 goal{7.0, 4.25};   // fly_mission 红点(10m 场地)
        GlobalResult gr = plan_global_path(pos, goal, obs, cfg, yaw);
        if (!gr.ok) gr = plan_global_path(pos, goal, obs, cfg);
        double heading = yaw;
        for (size_t i = 1; gr.ok && i < gr.path.size(); ++i) {
            const Vec2 a = gr.path[i - 1], b = gr.path[i];
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            if (len < 1e-6) continue;
            const double seg = std::atan2(b.y - a.y, b.x - a.x);
            const double turn = std::abs(wrap(seg - heading));
            if (turn >= 75 * M_PI / 180) { t += 1.5 + turn / 0.7; ++r.stop_turns; }
            else if (turn >= 45 * M_PI / 180) t += turn / 0.8 * 0.5;
            heading = seg; t += len / 0.45; r.home_length += len;
        }
    }
    r.time = t; r.coverage = grid.coverage_ratio(); r.done = r.coverage >= coverage_goal;
    return r;
}

bool set_field(FrontierConfig& f, const std::string& kv)
{
    const auto eq = kv.find('=');
    if (eq == std::string::npos) return false;
    const std::string k = kv.substr(0, eq);
    const double v = std::atof(kv.c_str() + eq + 1);
    struct F { const char* n; double FrontierConfig::* m; };
    static const F fields[] = {
        {"near_w", &FrontierConfig::near_weight}, {"turn_pen", &FrontierConfig::turn_penalty},
        {"cluster_w", &FrontierConfig::cluster_weight}, {"along", &FrontierConfig::along_bonus},
        {"band_width", &FrontierConfig::band_width}, {"band_tol", &FrontierConfig::band_tol},
        {"gain_w", &FrontierConfig::gain_weight}, {"clear_w", &FrontierConfig::clearance_weight},
        {"standoff", &FrontierConfig::observation_standoff}, {"min_dist", &FrontierConfig::observation_min_distance},
        {"cont_pen", &FrontierConfig::continuity_turn_penalty}, {"small_pen", &FrontierConfig::small_region_penalty},
        {"detour_w", &FrontierConfig::goal_detour_weight}, {"detour_start", &FrontierConfig::goal_detour_start},
    };
    for (const auto& fd : fields) if (k == fd.n) { f.*(fd.m) = v; return true; }
    if (k == "band_clear") { f.band_clear_cnt = static_cast<int>(v); return true; }
    if (k == "cluster_cap") { f.cluster_cap = static_cast<int>(v); return true; }
    return false;
}

}  // namespace

int main(int argc, char** argv)
{
    GridConfig gc{p::FIELD_MIN_X, p::FIELD_MIN_Y, p::FIELD_MAX_X, p::FIELD_MAX_Y,
                  p::BIG_CELL, p::SMALL_CELL, p::COVERAGE_THRESH, p::FOV_DEG, p::FOV_RANGE};
    FrontierConfig f{};
    f.min_x = gc.min_x; f.min_y = gc.min_y; f.max_x = gc.max_x; f.max_y = gc.max_y;
    f.big_cell = gc.big_cell; f.margin = p::WALL_MARGIN;
    f.near_weight = p::FRONTIER_NEAR_W; f.turn_penalty = p::FRONTIER_TURN_PEN; f.cluster_weight = p::FRONTIER_CLUSTER_W;
    f.cluster_cap = p::FRONTIER_CLUSTER_CAP; f.horizon = p::EXPLORE_HORIZON_M; f.chain_gap = p::CHAIN_GAP_M;
    f.band_width = p::BAND_WIDTH; f.band_tol = p::BAND_TOL; f.along_bonus = p::ALONG_BONUS; f.band_clear_cnt = p::BAND_CLEAR_CNT;
    f.small_region_cells = p::FRONTIER_SMALL_REGION_CELLS; f.small_region_penalty = p::FRONTIER_SMALL_REGION_PENALTY;
    f.observation_standoff = p::FRONTIER_OBSERVATION_STANDOFF; f.preferred_clearance = p::FRONTIER_PREFERRED_CLEARANCE;
    f.gain_weight = p::FRONTIER_GAIN_WEIGHT; f.clearance_weight = p::FRONTIER_CLEARANCE_WEIGHT;
    f.observation_min_distance = p::FRONTIER_OBSERVATION_MIN_DISTANCE; f.continuity_turn_penalty = p::FRONTIER_CONTINUITY_TURN_PEN;
    GlobalConfig g{};
    g.min_x = gc.min_x; g.min_y = gc.min_y; g.max_x = gc.max_x; g.max_y = gc.max_y;
    g.cell = p::GLOBAL_CELL; g.robot_radius = p::ROBOT_RADIUS; g.inflate = p::GLOBAL_MARGIN;
    g.wall_margin = p::GLOBAL_WALL_MARGIN; g.head_cone_half = p::GLOBAL_HEADING_CONE_DEG * M_PI / 180;
    g.head_cone_radius = p::GLOBAL_HEADING_CONE_RADIUS;
    f.minimum_clearance = g.robot_radius + g.inflate;
    f.goal = {7.0, 4.25}; f.goal_valid = true;
    f.goal_detour_weight = p::FRONTIER_GOAL_DETOUR_W; f.goal_detour_start = p::FRONTIER_GOAL_DETOUR_START;
    bool verbose = false;
    std::vector<std::string> layouts;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--set") && i + 1 < argc) {
            if (!set_field(f, argv[++i])) { std::fprintf(stderr, "unknown %s\n", argv[i]); return 1; }
        } else if (!std::strcmp(argv[i], "-v")) verbose = true;
        else layouts.push_back(argv[i]);
    }
    Result sum;
    for (const auto& path : layouts) {
        const Obstacles obs = load_layout(path);
        const Result r = run(obs, f, g, gc, p::DONE_COVERAGE, verbose);
        const std::string name = path.substr(path.find_last_of('/') + 1);
        std::printf("%-20s t=%6.1f 探索=%5.1f 归航=%4.1fm 路径=%5.1fm 停车转向=%2.0f 选点=%2.0f 途中换=%2.0f 覆盖=%.2f%s\n", name.c_str(),
                    r.time, r.explore_time, r.home_length, r.length, r.stop_turns, r.selects, r.exhausted, r.coverage, r.done ? "" : " (未完成)");
        sum.time += r.time; sum.explore_time += r.explore_time; sum.home_length += r.home_length;
        sum.length += r.length; sum.stop_turns += r.stop_turns; sum.selects += r.selects;
    }
    std::printf("TOTAL                t=%6.1f 探索=%6.1f 归航=%5.1fm 路径=%5.1fm 停车转向=%2.0f 选点=%2.0f\n", sum.time, sum.explore_time,
                sum.home_length, sum.length, sum.stop_turns, sum.selects);
    return 0;
}
