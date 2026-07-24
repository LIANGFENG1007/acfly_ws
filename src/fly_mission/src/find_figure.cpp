// ============================================================================
//  find_figure.cpp  ── 机载视觉找图 实现（详见同名 .hpp）
// ============================================================================

#include "fly_mission/find_figure.hpp"
#include "fly_mission/params.hpp"

#include <nlohmann/json.hpp>

#include <cmath>

namespace fly_mission {

namespace {
inline double dist(double ax, double ay, double bx, double by)
{
    return std::hypot(ax - bx, ay - by);
}
}  // namespace

FindFigure::FindFigure(rclcpp::Node* node)
    : node_(node)
{
    // 参数：编译期默认值集中在 fly_mission/params.hpp 的 ★视觉找图★ 段，
    //   这里 declare_parameter 引用它们，运行期 -p ff_* 可覆盖(与其余模块一致)。
    auto dd = [this](const std::string& n, double v) { return node_->declare_parameter<double>(n, v); };
    confirm_frames_ = static_cast<int>(std::lround(
                        dd("ff_confirm_frames", static_cast<double>(params::FF_CONFIRM_FRAMES))));
    assoc_dist_    = dd("ff_assoc_dist",    params::FF_ASSOC_DIST);
    black_radius_  = dd("ff_black_radius",  params::FF_BLACK_RADIUS);
    hover_sec_     = dd("ff_hover_sec",     params::FF_HOVER_SEC);
    timeout_sec_   = dd("ff_timeout_sec",   params::FF_TIMEOUT_SEC);
    frame_gap_sec_ = dd("ff_frame_gap_sec", params::FF_FRAME_GAP_SEC);

    // 不再自己订阅 /cv/target_info：改由主控订阅一次、只 parse 一次后 ingest() 喂进来
    //   (原来 FindFigure/LineFollower 各订各 parse 同一条消息，重复解析拖慢节点→起飞/走线卡顿)。
    RCLCPP_INFO(node_->get_logger(),
        "[找图] FindFigure 已启动 (确认 %d 帧, 关联 %.2fm, 拉黑半径 %.2fm, 超时 %.1fs)",
        confirm_frames_, assoc_dist_, black_radius_, timeout_sec_);
}

void FindFigure::reset_accum()
{
    acc_active_ = false;
    acc_color_.clear();
    acc_shape_.clear();
    acc_cx_ = acc_cy_ = 0.0;
    acc_n_ = 0;
}

void FindFigure::enable()
{
    std::lock_guard<std::mutex> lk(mtx_);
    enabled_ = true;
    reset_accum();   // 清掉启用前可能残留的半截累计，从干净状态开始收帧
    RCLCPP_INFO(node_->get_logger(), "[找图] 已启用(起飞稳定完成)，开始监听图形");
}

std::vector<FindFigure::Target> FindFigure::parse_targets(const nlohmann::json& j) const
{
    std::vector<Target> out;
    try {
        const auto it = j.find("targets");
        if (it == j.end() || !it->is_array()) return out;
        for (const auto& e : *it) {
            if (!e.is_object()) continue;
            // 容错取值：字段缺失/类型不对 → 跳过该对象，不整体失败
            if (!e.contains("x") || !e.contains("y")) continue;
            if (!e["x"].is_number() || !e["y"].is_number()) continue;
            Target t;
            t.x = e["x"].get<double>();
            t.y = e["y"].get<double>();
            t.color = e.value("color", std::string{});
            t.shape = e.value("shape", std::string{});
            if (!std::isfinite(t.x) || !std::isfinite(t.y)) continue;
            out.push_back(std::move(t));
        }
    } catch (const std::exception& ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[找图] /cv/target_info JSON 解析失败: %s", ex.what());
    }
    return out;
}

bool FindFigure::in_blacklist(const P& p) const
{
    for (const auto& v : visited_)
        if (dist(p.x, p.y, v.x, v.y) <= black_radius_) return true;
    return false;
}

// 喂入一帧已解析 JSON(主控 parse 一次后调用)：解析目标 → 坐标约束累计 → 满帧入队
void FindFigure::ingest(const nlohmann::json& j)
{
    const std::vector<Target> dets = parse_targets(j);

    std::lock_guard<std::mutex> lk(mtx_);
    // ★未启用(起飞未稳)★：直接丢弃，不累计/不入队——避免起飞途中错坐标污染 + 拉黑起飞点。
    if (!enabled_) return;

    const rclcpp::Time now = node_->now();

    // 断帧：距上次命中累计超过 frame_gap_sec_ → 目标暂时消失，累计清零重来
    if (acc_active_ && acc_time_valid_ &&
        (now - acc_last_time_).seconds() > frame_gap_sec_) {
        reset_accum();
    }

    if (dets.empty()) return;

    // 1) 若正在累计：在本帧里找与当前累计 (color,shape,坐标≈) 匹配的检测
    const Target* hit = nullptr;
    if (acc_active_) {
        for (const auto& d : dets) {
            if (d.color == acc_color_ && d.shape == acc_shape_ &&
                dist(d.x, d.y, acc_cx_, acc_cy_) < assoc_dist_) {
                hit = &d;
                break;
            }
        }
    }

    if (hit) {
        // 命中 → 增量更新算术平均中心，帧数+1
        acc_cx_ = (acc_cx_ * acc_n_ + hit->x) / (acc_n_ + 1);
        acc_cy_ = (acc_cy_ * acc_n_ + hit->y) / (acc_n_ + 1);
        acc_n_ += 1;
    } else {
        // 没在累计 / 当前累计目标本帧没出现 → 用本帧第一个目标重开一段累计
        const Target& d0 = dets.front();
        acc_active_ = true;
        acc_color_  = d0.color;
        acc_shape_  = d0.shape;
        acc_cx_ = d0.x; acc_cy_ = d0.y;
        acc_n_  = 1;
    }
    acc_last_time_  = now;
    acc_time_valid_ = true;

    // 2) 累计满 → 确认一个图形
    if (acc_n_ >= confirm_frames_) {
        const P center{acc_cx_, acc_cy_};

        // 已拉黑 → 直接丢弃、不入队(不再飞过去)
        if (in_blacklist(center)) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[找图] 确认 %s %s @ (%.2f,%.2f) 落在已拉黑区域 → 忽略",
                acc_color_.c_str(), acc_shape_.c_str(), center.x, center.y);
            reset_accum();
            return;
        }

        // 队列去重：已有非常接近的同类目标在排队 → 不重复入队
        bool dup = false;
        for (const auto& q : queue_) {
            if (q.color == acc_color_ && q.shape == acc_shape_ &&
                dist(q.x, q.y, center.x, center.y) < black_radius_) { dup = true; break; }
        }
        if (!dup) {
            Target t;
            t.color = acc_color_; t.shape = acc_shape_;
            t.x = center.x; t.y = center.y;
            queue_.push_back(t);
            RCLCPP_INFO(node_->get_logger(),
                "[找图] 确认目标 #%zu: %s %s @ (%.2f,%.2f)，入队",
                queue_.size(), t.color.c_str(), t.shape.c_str(), t.x, t.y);
        }
        reset_accum();   // 准备累计下一个
    }
}

bool FindFigure::has_pending() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return !queue_.empty();
}

void FindFigure::begin()
{
    std::lock_guard<std::mutex> lk(mtx_);
    hover_active_    = false;
    find_start_time_ = node_->now();
    find_time_valid_ = true;
}

FindFigure::Step FindFigure::tick(bool reached_current_target, double& out_x, double& out_y)
{
    std::lock_guard<std::mutex> lk(mtx_);

    if (queue_.empty()) return Step::DONE;   // 防御：无目标

    Target& t = queue_.front();

    // ★实时刷新中心★：若后台仍在累计【同一个】目标(同 color+shape 且坐标≈队首)，
    //   用其更新后的算术平均中心，实现"没到位就用最新中心继续飞"。否则用入队时中心。
    if (acc_active_ && acc_color_ == t.color && acc_shape_ == t.shape &&
        dist(acc_cx_, acc_cy_, t.x, t.y) < black_radius_) {
        t.x = acc_cx_;
        t.y = acc_cy_;
    }
    out_x = t.x;
    out_y = t.y;

    const rclcpp::Time now = node_->now();

    // 结束本次找图的统一收尾：把中心拉黑、弹出队首、清悬停/计时态。
    auto finish = [&](const char* why) {
        visited_.push_back(P{t.x, t.y});
        RCLCPP_INFO(node_->get_logger(),
            "[找图] 目标 %s %s @ (%.2f,%.2f) %s → 拉黑(半径%.2fm)，回原状态",
            t.color.c_str(), t.shape.c_str(), t.x, t.y, why, black_radius_);
        queue_.pop_front();
        hover_active_    = false;
        find_time_valid_ = false;
    };

    // 超时：整个找图过程超 timeout_sec_ → 放弃本次(但仍拉黑)
    if (find_time_valid_ && (now - find_start_time_).seconds() > timeout_sec_) {
        finish("超时放弃");
        return Step::DONE;
    }

    // 悬停计时中：等满 hover_sec_ → 完成
    if (hover_active_) {
        if (now >= hover_until_) {
            finish("已到点并悬停完成");
            return Step::DONE;
        }
        return Step::FLYING;   // 悬停期间保持在中心上方(主控继续 target_xy_slam)
    }

    // 未悬停：飞到中心上方(reach_tol_ 内，由主控到位判定给出) → 进入悬停
    if (reached_current_target) {
        hover_active_ = true;
        hover_until_  = now + rclcpp::Duration::from_seconds(hover_sec_);
        RCLCPP_INFO(node_->get_logger(),
            "[找图] 已到 %s %s 上方 (%.2f,%.2f)，悬停 %.1fs",
            t.color.c_str(), t.shape.c_str(), t.x, t.y, hover_sec_);
        return Step::FLYING;
    }

    return Step::FLYING;   // 仍在飞向中心
}

}  // namespace fly_mission
