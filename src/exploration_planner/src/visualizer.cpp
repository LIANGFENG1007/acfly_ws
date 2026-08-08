#include "exploration_planner/visualizer.hpp"

#include <cmath>
#include <algorithm>

namespace exploration {

Visualizer::Visualizer(const GridConfig& grid_cfg, int canvas_px)
    : cfg_(grid_cfg)
{
    field_w_ = cfg_.max_x - cfg_.min_x;
    field_h_ = cfg_.max_y - cfg_.min_y;

    // 按场地长边等比缩放到 canvas_px
    const double longer = std::max(field_w_, field_h_);
    scale_ = (longer > 1e-6) ? (canvas_px / longer) : 1.0;

    W_ = std::max(1, static_cast<int>(std::lround(field_w_ * scale_)));
    H_ = std::max(1, static_cast<int>(std::lround(field_h_ * scale_)));
}

cv::Point Visualizer::to_px(double x, double y) const
{
    const int px = static_cast<int>(std::lround((x - cfg_.min_x) * scale_));
    // 图像 y 轴向下：场地上方(y大)对应像素上方(行小)
    const int py = static_cast<int>(std::lround((cfg_.max_y - y) * scale_));
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
                           bool unreachable_valid, const Vec2& unreachable_pos)
{
    cv::Mat img(H_, W_, CV_8UC3, cv::Scalar(40, 40, 40));   // 深灰底

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
    if (traj.size() >= 2) {
        for (size_t i = 1; i < traj.size(); ++i) {
            cv::line(img, to_px(traj[i - 1].p.x, traj[i - 1].p.y),
                          to_px(traj[i].p.x, traj[i].p.y),
                     cv::Scalar(230, 200, 60), 2);   // 青黄色折线
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
    if (look_valid) {
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
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Coverage: %.1f%%", grid.coverage_ratio() * 100.0);
    cv::putText(img, buf, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);

    return img;
}

}  // namespace exploration
