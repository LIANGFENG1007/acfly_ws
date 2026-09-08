#include "exploration_planner/visualizer.hpp"

#include <cmath>
#include <algorithm>
#include <array>
#include <cstdio>

namespace exploration {
namespace {

bool finite_point(const Vec2& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y);
}

std::array<Vec2, 4> corridor_corners(const CorridorVisualState& corridor)
{
    const double dx = corridor.h.x - corridor.entry.x;
    const double dy = corridor.h.y - corridor.entry.y;
    const double length = std::hypot(dx, dy);
    const double half_width = std::isfinite(corridor.width)
        ? std::max(0.05, corridor.width * 0.5) : 1.0;
    const Vec2 normal = length > 1e-6
        ? Vec2{-dy / length * half_width, dx / length * half_width}
        : Vec2{half_width, 0.0};
    return {{{corridor.entry.x + normal.x, corridor.entry.y + normal.y},
             {corridor.h.x + normal.x, corridor.h.y + normal.y},
             {corridor.h.x - normal.x, corridor.h.y - normal.y},
             {corridor.entry.x - normal.x, corridor.entry.y - normal.y}}};
}

void dashed_line(cv::Mat& img, cv::Point from, cv::Point to,
                 const cv::Scalar& color)
{
    const cv::Point2d delta = cv::Point2d(to) - cv::Point2d(from);
    const double length = cv::norm(delta);
    if (length < 1.0) return;
    for (double d = 0; d < length; d += 13.0) {
        const cv::Point a = cv::Point2d(from) + delta * (d / length);
        const cv::Point b = cv::Point2d(from) + delta * (std::min(d + 7.0, length) / length);
        cv::line(img, a, b, color, 1, cv::LINE_AA);
    }
}

void status_line(cv::Mat& img, const std::string& text, int y,
                 const cv::Scalar& color)
{
    std::string visible = text;
    const double font_scale = 0.46;
    int baseline = 0;
    if (cv::getTextSize(visible, cv::FONT_HERSHEY_SIMPLEX,
                        font_scale, 1, &baseline).width > img.cols - 16) {
        while (!visible.empty() &&
               cv::getTextSize(visible + "...", cv::FONT_HERSHEY_SIMPLEX,
                               font_scale, 1, &baseline).width > img.cols - 16) {
            visible.pop_back();
        }
        visible += "...";
    }
    cv::putText(img, visible, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX,
                font_scale, color, 1, cv::LINE_AA);
}

}  // namespace

Visualizer::Visualizer(const GridConfig& grid_cfg, int canvas_px)
    : cfg_(grid_cfg), canvas_px_(std::max(96, canvas_px))
{
    update_bounds({});
}

void Visualizer::update_bounds(const CorridorVisualState& corridor)
{
    double min_x = cfg_.min_x, max_x = cfg_.max_x;
    double min_y = cfg_.min_y, max_y = cfg_.max_y;
    const bool configured = corridor.configured && finite_point(corridor.entry) &&
        finite_point(corridor.h);
    if (configured) {
        // Display bounds are independent of coverage-grid indices and do not
        // follow noisy cloud extrema, so the map remains stable during flight.
        for (const Vec2& p : corridor_corners(corridor)) {
            min_x = std::min(min_x, p.x);
            max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y);
            max_y = std::max(max_y, p.y);
        }
    }
    constexpr double margin = 0.45;
    min_x -= margin;
    min_y -= margin;
    max_x += margin;
    max_y += margin;
    view_min_x_ = min_x;
    view_max_y_ = max_y;
    field_w_ = std::max(1e-6, max_x - min_x);
    field_h_ = std::max(1e-6, max_y - min_y);
    scale_ = std::min((canvas_px_ - 1) / field_w_,
                      (canvas_px_ - 1 - status_height_) / field_h_);

    const int map_width = std::max(1, static_cast<int>(std::lround(field_w_ * scale_)) + 1);
    W_ = std::max(std::min(400, canvas_px_), map_width);
    H_ = std::max(1, static_cast<int>(std::lround(field_h_ * scale_)) + 1 + status_height_);
    offset_x_ = (W_ - map_width) * 0.5;
}

cv::Point Visualizer::to_px(double x, double y) const
{
    const int px = static_cast<int>(std::lround((x - view_min_x_) * scale_ + offset_x_));
    // 图像 y 轴向下：场地上方(y大)对应像素上方(行小)
    const int py = static_cast<int>(std::lround((view_max_y_ - y) * scale_)) + status_height_;
    return cv::Point(px, py);
}

cv::Mat Visualizer::render(const GridSnapshot& grid,
                           const Trajectory& traj,
                           const Vec2& goal, bool goal_valid,
                           double px, double py, double yaw, bool pose_valid,
                           const Vec2& lookahead, bool look_valid,
                           const std::vector<Vec2>& pois,
                           const Obstacles& obstacles,
                           bool turning, int turn_dir,
                           bool unreachable_valid, const Vec2& unreachable_pos,
                           const CorridorVisualState& corridor)
{
    update_bounds(corridor);
    cv::Mat img(H_, W_, CV_8UC3, cv::Scalar(40, 40, 40));   // 深灰底
    const bool corridor_configured = corridor.configured && finite_point(corridor.entry) &&
        finite_point(corridor.h);
    if (corridor_configured) {
        std::vector<cv::Point> reference;
        for (const Vec2& p : corridor_corners(corridor)) {
            reference.push_back(to_px(p.x, p.y));
        }
        cv::fillConvexPoly(img, reference, cv::Scalar(62, 48, 48));
        cv::rectangle(img, to_px(cfg_.min_x, cfg_.max_y),
                      to_px(cfg_.max_x, cfg_.min_y), cv::Scalar(40, 40, 40), cv::FILLED);
        for (size_t i = 0; i < reference.size(); ++i) {
            dashed_line(img, reference[i], reference[(i + 1) % reference.size()],
                        cv::Scalar(136, 119, 101));
        }
    }

    const int cpb = grid.cells_per_big();
    const double sc = grid.config().small_cell;

    // ---- 1) 栅格填充：已探索大格整格填绿；未满大格逐【小格】填(只填扫到的) ----
    //   小格细线、大格粗线，对比出层次。逐小格只对"未满大格"做，已探索大格一刀整填省开销。
    for (int bi = 0; bi < grid.big_nx(); ++bi) {
        for (int bj = 0; bj < grid.big_ny(); ++bj) {
            // 大格世界范围
            const double x0 = cfg_.min_x + bi * cpb * sc;
            const double y0 = cfg_.min_y + bj * cpb * sc;
            const double x1 = x0 + cpb * sc;
            const double y1 = y0 + cpb * sc;
            cv::Point p_tl = to_px(x0, y1);   // 左上(世界 y 大)
            cv::Point p_br = to_px(x1, y0);   // 右下

            if (grid.big_explored(bi, bj)) {
                cv::rectangle(img, p_tl, p_br, cv::Scalar(60, 180, 70), cv::FILLED);  // 已探索整格绿
            } else {
                // 逐小格：扫到的填浅绿，没扫到的留底色 → 障碍/墙根被填的小格也能精确看到
                const int gi_lo = bi * cpb, gj_lo = bj * cpb;
                const int gi_hi = std::min(gi_lo + cpb, grid.small_nx());
                const int gj_hi = std::min(gj_lo + cpb, grid.small_ny());
                for (int gi = gi_lo; gi < gi_hi; ++gi) {
                    for (int gj = gj_lo; gj < gj_hi; ++gj) {
                        if (!grid.small_scanned(gi, gj)) continue;
                        const double sx0 = cfg_.min_x + gi * sc;
                        const double sy0 = cfg_.min_y + gj * sc;
                        cv::rectangle(img, to_px(sx0, sy0 + sc), to_px(sx0 + sc, sy0),
                                      cv::Scalar(70, 140, 80), cv::FILLED);   // 已扫小格浅绿
                    }
                }
            }
        }
    }

    // ---- 1a) 小格细网格线（细、暗，仅勾勒）----
    //   仅当每小格 ≥3px 时才画，否则线太密会糊成一片实色反而盖住填充。
    if (sc * scale_ >= 3.0) {
        const int snx = grid.small_nx(), sny = grid.small_ny();
        const cv::Scalar fine(70, 70, 70);   // 暗灰细线
        for (int gi = 0; gi <= snx; ++gi) {
            const double x = cfg_.min_x + gi * sc;
            cv::line(img, to_px(x, cfg_.min_y), to_px(x, cfg_.max_y), fine, 1);
        }
        for (int gj = 0; gj <= sny; ++gj) {
            const double y = cfg_.min_y + gj * sc;
            cv::line(img, to_px(cfg_.min_x, y), to_px(cfg_.max_x, y), fine, 1);
        }
    }

    // ---- 1b) 大格网格线（微粗、稍亮，盖在小格线上区分层次）----
    {
        const cv::Scalar coarse(120, 120, 120);
        for (int bi = 0; bi <= grid.big_nx(); ++bi) {
            const double x = cfg_.min_x + std::min(bi * cpb, grid.small_nx()) * sc;
            cv::line(img, to_px(x, cfg_.min_y), to_px(x, cfg_.max_y), coarse, 2);
        }
        for (int bj = 0; bj <= grid.big_ny(); ++bj) {
            const double y = cfg_.min_y + std::min(bj * cpb, grid.small_ny()) * sc;
            cv::line(img, to_px(cfg_.min_x, y), to_px(cfg_.max_x, y), coarse, 2);
        }
    }

    // ---- 1c) 雷达障碍物：绿色实心圆（半径按 scale_ 等比缩放到正确位置）----
    for (const auto& o : obstacles) {
        cv::Point oc = to_px(o.cx, o.cy);
        const int rpx = std::max(2, static_cast<int>(std::lround(o.r * scale_)));
        cv::circle(img, oc, rpx, cv::Scalar(0, 220, 0), cv::FILLED);   // 纯绿实心(BGR)
        cv::circle(img, oc, rpx, cv::Scalar(0, 100, 0), 2);           // 深绿描边
    }

    // ---- 2) 规划轨迹 ----
    if (!corridor.active && traj.size() >= 2) {
        for (size_t i = 1; i < traj.size(); ++i) {
            cv::line(img, to_px(traj[i - 1].p.x, traj[i - 1].p.y),
                          to_px(traj[i].p.x, traj[i].p.y),
                     cv::Scalar(255, 185, 80), 2, cv::LINE_AA);
        }
    }

    // ---- 3) 终点 ----
    if (goal_valid) {
        cv::Point gp = to_px(goal.x, goal.y);
        cv::drawMarker(img, gp, cv::Scalar(0, 0, 255), cv::MARKER_STAR, 16, 2);
    }

    // ---- 3b) 插点(目的地)：蓝色圆点 ----
    for (const auto& q : pois) {
        cv::Point qp = to_px(q.x, q.y);
        cv::circle(img, qp, 6, cv::Scalar(255, 0, 0), cv::FILLED);   // 蓝实心(BGR)
        cv::circle(img, qp, 6, cv::Scalar(255, 255, 255), 1);        // 白描边更醒目
    }

    if (corridor_configured) {
        for (const Vec2& p : corridor.points) {
            if (!finite_point(p)) continue;
            cv::circle(img, to_px(p.x, p.y), 1, cv::Scalar(225, 225, 225), cv::FILLED);
        }
        for (size_t i = 1; corridor.active && i < corridor.route.size(); ++i) {
            if (!finite_point(corridor.route[i - 1]) || !finite_point(corridor.route[i])) continue;
            cv::line(img, to_px(corridor.route[i - 1].x, corridor.route[i - 1].y),
                          to_px(corridor.route[i].x, corridor.route[i].y),
                     cv::Scalar(255, 185, 80), 2, cv::LINE_AA);
        }
        if (corridor.gate_valid && finite_point(corridor.gate_left) &&
            finite_point(corridor.gate_right) && finite_point(corridor.gate_center)) {
            const cv::Point left = to_px(corridor.gate_left.x, corridor.gate_left.y);
            const cv::Point right = to_px(corridor.gate_right.x, corridor.gate_right.y);
            const cv::Point center = to_px(corridor.gate_center.x, corridor.gate_center.y);
            const cv::Scalar opening_color(225, 90, 240);
            cv::line(img, left, right, opening_color, 2, cv::LINE_AA);
            cv::circle(img, left, 4, opening_color, 1, cv::LINE_AA);
            cv::circle(img, right, 4, opening_color, 1, cv::LINE_AA);
            cv::drawMarker(img, center, opening_color, cv::MARKER_CROSS, 14, 2, cv::LINE_AA);
        }
        const cv::Point entry = to_px(corridor.entry.x, corridor.entry.y);
        const cv::Point h = to_px(corridor.h.x, corridor.h.y);
        cv::circle(img, entry, 9, cv::Scalar(0, 210, 140), cv::FILLED, cv::LINE_AA);
        cv::putText(img, "E", entry + cv::Point(-5, 5), cv::FONT_HERSHEY_SIMPLEX,
                    0.45, cv::Scalar(20, 35, 25), 1, cv::LINE_AA);
        cv::circle(img, h, 12, cv::Scalar(245, 245, 245), 2, cv::LINE_AA);
        cv::putText(img, "H", h + cv::Point(-6, 6), cv::FONT_HERSHEY_SIMPLEX,
                    0.55, cv::Scalar(245, 245, 245), 2, cv::LINE_AA);
    }

    // ---- 4) 飞机：位置 + 朝向箭头 + FOV_DEG 扇形视野 ----
    if (pose_valid) {
        cv::Point pp = to_px(px, py);

        // 视野扇形轮廓
        const double half = (cfg_.fov_deg * 0.5) * M_PI / 180.0;
        const double r = cfg_.fov_range;
        std::vector<cv::Point> fan;
        fan.push_back(pp);
        const int seg = 24;
        for (int k = 0; k <= seg; ++k) {
            const double a = yaw - half + (2.0 * half) * k / seg;
            fan.push_back(to_px(px + r * std::cos(a), py + r * std::sin(a)));
        }
        const cv::Point* pts = fan.data();
        const int npts = static_cast<int>(fan.size());
        cv::polylines(img, &pts, &npts, 1, true, cv::Scalar(80, 160, 255), 1);

        // 朝向箭头
        cv::Point head = to_px(px + 0.5 * std::cos(yaw), py + 0.5 * std::sin(yaw));
        cv::arrowedLine(img, pp, head, cv::Scalar(0, 255, 255), 2, cv::LINE_AA, 0, 0.3);

        // 机体点
        cv::circle(img, pp, 4, cv::Scalar(0, 255, 255), cv::FILLED);
    }

    // ---- 5) 前瞻点 ----
    if (look_valid && !corridor.active) {
        cv::circle(img, to_px(lookahead.x, lookahead.y), 3, cv::Scalar(255, 0, 255), cv::FILLED);
    }

    // ---- 5a) 原地转身找解标志：飞机处画带缺口圆弧 + 切向箭头(橙黄)，表示"正在原地旋转找路" ----
    if (turning && pose_valid) {
        cv::Point c = to_px(px, py);
        const int rr = std::max(10, static_cast<int>(std::lround(0.55 * scale_)));
        // 缺口圆弧(40°~320°)，留缺口更像"旋转中"图标
        cv::ellipse(img, c, cv::Size(rr, rr), 0, 40, 320, cv::Scalar(0, 180, 255), 2, cv::LINE_AA);
        // 弧端点处加一个切向箭头指示转向(turn_dir +1 逆时针 / -1 顺时针)
        const double ang_deg = (turn_dir >= 0) ? 320.0 : 40.0;
        const double ae = ang_deg * M_PI / 180.0;
        cv::Point tip(c.x + static_cast<int>(rr * std::cos(ae)),
                      c.y + static_cast<int>(rr * std::sin(ae)));
        const double td = ae + (turn_dir >= 0 ? M_PI / 2.0 : -M_PI / 2.0);   // 切线方向
        cv::Point tail(tip.x - static_cast<int>(14 * std::cos(td)),
                       tip.y - static_cast<int>(14 * std::sin(td)));
        cv::arrowedLine(img, tail, tip, cv::Scalar(0, 180, 255), 2, cv::LINE_AA, 0, 0.5);
    }

    // ---- 5b) 真无解红叉：目标被围死(转一圈仍无解)→ 在该位置画红色 X ----
    if (unreachable_valid) {
        cv::Point u = to_px(unreachable_pos.x, unreachable_pos.y);
        const int d = std::max(7, static_cast<int>(std::lround(0.25 * scale_)));
        cv::line(img, cv::Point(u.x - d, u.y - d), cv::Point(u.x + d, u.y + d), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        cv::line(img, cv::Point(u.x - d, u.y + d), cv::Point(u.x + d, u.y - d), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }

    // ---- 6) 覆盖率文本 ----
    char buf[128];
    const char* stage = corridor.active ? "CORRIDOR" : (pose_valid ? "EXPLORATION" : "WAITING");
    std::snprintf(buf, sizeof(buf), "Coverage: %.1f%% | %s", grid.coverage_ratio() * 100.0, stage);
    cv::rectangle(img, cv::Rect(0, 0, W_, status_height_), cv::Scalar(28, 28, 28), cv::FILLED);
    status_line(img, buf, 17, cv::Scalar(190, 220, 190));
    if (corridor_configured) {
        const std::string phase = corridor.active
            ? (corridor.phase.empty() ? "ACTIVE" : corridor.phase) : "READY";
        status_line(img, "Corridor: " + phase + " | Passed: " + std::to_string(corridor.gates_passed),
                    37, cv::Scalar(255, 220, 165));
        if (corridor.gate_valid && finite_point(corridor.gate_left) &&
            finite_point(corridor.gate_right)) {
            const double width = std::hypot(corridor.gate_left.x - corridor.gate_right.x,
                                            corridor.gate_left.y - corridor.gate_right.y);
            std::snprintf(buf, sizeof(buf), "Gap: %.2f m | Cloud: %zu | Corridor ref: %.2f m",
                          width, corridor.points.size(), corridor.width);
        } else {
            std::snprintf(buf, sizeof(buf), "Gap: -- | Cloud: %zu | Corridor ref: %.2f m",
                          corridor.points.size(), corridor.width);
        }
        status_line(img, buf, 57, cv::Scalar(225, 195, 230));
    } else {
        status_line(img, "Corridor: -- | Passed: --", 37, cv::Scalar(255, 220, 165));
        status_line(img, "Gap: -- | Cloud: 0 | Corridor ref: --", 57, cv::Scalar(225, 195, 230));
    }

    return img;
}

}  // namespace exploration
