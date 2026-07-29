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
// 目标的可读标签(日志用)：有类名用类名，否则显示编号；都没有回退 color+shape。
std::string label_of(int id, const std::string& color, const std::string& shape)
{
    if (id != 0) {
        const char* n = params::ff_class_name(id);
        if (n[0] != '\0') return std::string(n) + "(id" + std::to_string(id) + ")";
        return "id" + std::to_string(id);
    }
    return color + " " + shape;
}
}  // namespace

double FindFigure::dist_xy(double ax, double ay, double bx, double by)
{
    return std::hypot(ax - bx, ay - by);
}

FindFigure::FindFigure(rclcpp::Node* node)
    : node_(node)
{
    // 参数：编译期默认值集中在 fly_mission/params.hpp 的 ★视觉找图★ 段，
    //   这里 declare_parameter 引用它们，运行期 -p ff_* 可覆盖(与其余模块一致)。
    auto dd = [this](const std::string& n, double v) { return node_->declare_parameter<double>(n, v); };
    confirm_frames_ = static_cast<int>(std::lround(
                        dd("ff_confirm_frames", static_cast<double>(params::FF_CONFIRM_FRAMES))));
    assoc_dist_    = dd("ff_assoc_dist",    params::FF_ASSOC_DIST);
    max_accum_     = static_cast<int>(std::lround(
                        dd("ff_max_accum", static_cast<double>(params::FF_MAX_ACCUM))));
    if (max_accum_ < 1) max_accum_ = 1;
    black_radius_  = dd("ff_black_radius",  params::FF_BLACK_RADIUS);
    hover_sec_     = dd("ff_hover_sec",     params::FF_HOVER_SEC);
    timeout_sec_   = dd("ff_timeout_sec",   params::FF_TIMEOUT_SEC);
    frame_gap_sec_ = dd("ff_frame_gap_sec", params::FF_FRAME_GAP_SEC);

    // 不再自己订阅 /cv/target_info：改由主控订阅一次、只 parse 一次后 ingest() 喂进来
    //   (原来 FindFigure/LineFollower 各订各 parse 同一条消息，重复解析拖慢节点→起飞/走线卡顿)。
    RCLCPP_INFO(node_->get_logger(),
        "[找图] FindFigure 已启动 (确认 %d 帧, 关联 %.2fm, 拉黑半径 %.2fm, 超时 %.1fs, "
        "并行累计上限 %d 个; 身份判据=视觉id，同种不同只靠坐标差>%.2fm 区分)",
        confirm_frames_, assoc_dist_, black_radius_, timeout_sec_, max_accum_, assoc_dist_);
}

// 清掉全部并行累计器(启用时、以及需要从干净状态重来时用)。
void FindFigure::reset_accum()
{
    accums_.clear();
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
            // ★取类别编号 id★(区分不同动物的唯一依据；视觉 shm_class_ids 给)。
            //   缺字段/非数字 → 0，此时下面 same_kind() 自动回退比 color+shape(老视觉兼容)。
            if (e.contains("id") && e["id"].is_number_integer())
                t.id = e["id"].get<int>();
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

// 喂入一帧已解析 JSON(主控 parse 一次后调用)：
//   解析目标 → ★每个物体各自并行累计★(按 种类id + 坐标 分桶) → 谁攒满谁入队。
void FindFigure::ingest(const nlohmann::json& j)
{
    const std::vector<Target> dets = parse_targets(j);

    std::lock_guard<std::mutex> lk(mtx_);
    // ★未启用(起飞未稳)★：直接丢弃，不累计/不入队——避免起飞途中错坐标污染 + 拉黑起飞点。
    if (!enabled_) return;

    const rclcpp::Time now = node_->now();

    // 1) 断帧清理：★逐个累计器单独判★——某个目标消失只作废它自己，
    //    其他仍在视野里的目标继续累计(原实现是一刀清空，一个目标闪一下全部重来)。
    for (size_t i = 0; i < accums_.size(); ) {
        if (accums_[i].time_valid &&
            (now - accums_[i].last_time).seconds() > frame_gap_sec_) {
            accums_.erase(accums_.begin() + static_cast<long>(i));
        } else {
            ++i;
        }
    }

    if (dets.empty()) return;

    // 2) 本帧每条检测各自找归属累计器；找不到就新开一个。
    //    ★一个累计器本帧最多吃一条检测★(matched)，否则同种类两只挨得近时可能被同一个
    //    累计器连吃两条、把中心拉到两只中间。
    std::vector<char> used(accums_.size(), 0);
    for (const auto& d : dets) {
        // 已拉黑的位置：连累计都不开(不再飞过去)，省掉无用计算与日志
        if (in_blacklist(P{d.x, d.y})) continue;

        int hit = -1;
        double best = assoc_dist_;
        for (size_t k = 0; k < accums_.size(); ++k) {
            if (used[k]) continue;
            if (!d.same_kind(accums_[k].t)) continue;              // 种类不同 → 不是它
            const double dd = dist(d.x, d.y, accums_[k].t.x, accums_[k].t.y);
            if (dd < best) { best = dd; hit = static_cast<int>(k); }  // 取最近的那个桶
        }

        if (hit >= 0) {
            // 命中：增量更新算术平均中心，帧数+1
            Accum& a = accums_[static_cast<size_t>(hit)];
            a.t.x = (a.t.x * a.n + d.x) / (a.n + 1);
            a.t.y = (a.t.y * a.n + d.y) / (a.n + 1);
            if (a.t.id == 0 && d.id != 0) a.t.id = d.id;   // 补上后来才给的 id
            a.n += 1;
            a.last_time  = now;
            a.time_valid = true;
            used[static_cast<size_t>(hit)] = 1;
        } else {
            // 没有归属 → 这是个新物体(不同种类，或同种类的另一只) → 新开一个累计器
            if (static_cast<int>(accums_.size()) >= max_accum_) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                    "[找图] 并行累计已达上限 %d 个，本帧新目标 %s @(%.2f,%.2f) 丢弃"
                    "(可调大 ff_max_accum)",
                    max_accum_, label_of(d.id, d.color, d.shape).c_str(), d.x, d.y);
                continue;
            }
            Accum a;
            a.t = d;
            a.n = 1;
            a.last_time  = now;
            a.time_valid = true;
            accums_.push_back(a);
            used.push_back(1);
        }
    }

    // 3) 检查每个累计器是否攒满 → 确认入队(满了的移除，其余继续攒)
    for (size_t i = 0; i < accums_.size(); ) {
        Accum& a = accums_[i];
        if (a.n < confirm_frames_) { ++i; continue; }

        const P center{a.t.x, a.t.y};
        const std::string tag = label_of(a.t.id, a.t.color, a.t.shape);

        // 已拉黑 → 丢弃不入队(飞行途中该点可能刚被拉黑)
        if (in_blacklist(center)) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[找图] 确认 %s @ (%.2f,%.2f) 落在已拉黑区域 → 忽略",
                tag.c_str(), center.x, center.y);
            accums_.erase(accums_.begin() + static_cast<long>(i));
            continue;
        }

        // 队列去重：★同种类★且非常接近的目标已在排队 → 不重复入队。
        //   注意仍按 same_kind 比，所以"同位置附近的不同动物"不会被误去重。
        bool dup = false;
        for (const auto& q : queue_) {
            if (q.same_kind(a.t) &&
                dist(q.x, q.y, center.x, center.y) < black_radius_) { dup = true; break; }
        }
        if (!dup) {
            Target t = a.t;
            t.x = center.x; t.y = center.y;
            queue_.push_back(t);
            RCLCPP_INFO(node_->get_logger(),
                "[找图] 确认目标 #%zu: %s @ (%.2f,%.2f)，入队(队列 %zu 个，并行累计中 %zu 个)",
                queue_.size(), tag.c_str(), t.x, t.y, queue_.size(), accums_.size() - 1);
        }
        accums_.erase(accums_.begin() + static_cast<long>(i));
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

    // ★实时刷新中心★：若后台某个累计器仍在跟【同一个】物体(同种类 且 坐标≈队首)，
    //   用它更新后的算术平均中心，实现"没到位就用最新中心继续飞"。否则用入队时中心。
    //   ★取最近的那个★：同种类可能有多只在累计，必须挑离队首目标最近的，
    //   否则会被同种的另一只把飞行目标拽走。
    {
        int best_k = -1;
        double best_d = black_radius_;      // 超过拉黑半径就不算同一个物体了
        for (size_t k = 0; k < accums_.size(); ++k) {
            if (!accums_[k].t.same_kind(t)) continue;
            const double dd = dist(accums_[k].t.x, accums_[k].t.y, t.x, t.y);
            if (dd < best_d) { best_d = dd; best_k = static_cast<int>(k); }
        }
        if (best_k >= 0) {
            t.x = accums_[static_cast<size_t>(best_k)].t.x;
            t.y = accums_[static_cast<size_t>(best_k)].t.y;
        }
    }
    out_x = t.x;
    out_y = t.y;

    const rclcpp::Time now = node_->now();

    // 结束本次找图的统一收尾：把中心拉黑、弹出队首、清悬停/计时态。
    auto finish = [&](const char* why) {
        // ★先按值拷出来★：下面 queue_.pop_front() 会销毁 t 指向的元素，
        //   之后再读 t.x/t.y 就是悬垂引用(t 是 queue_.front() 的引用)。
        const double fx = t.x, fy = t.y;
        const std::string tag = label_of(t.id, t.color, t.shape);

        visited_.push_back(P{fx, fy});
        RCLCPP_INFO(node_->get_logger(),
            "[找图] 目标 %s @ (%.2f,%.2f) %s → 拉黑(半径%.2fm)，回原状态",
            tag.c_str(), fx, fy, why, black_radius_);
        queue_.pop_front();
        hover_active_    = false;
        find_time_valid_ = false;
        // ★丢掉仍在跟这个位置的累计器★：不然它已攒了大半，刚拉黑就又攒满一次。
        //   虽然 ingest 里 in_blacklist 会拦住入队，但那要等它攒满、白转一圈还刷日志。
        //   ★同种类的另一只(离得远)不受影响★——只清落在拉黑圆内的。
        for (size_t k = 0; k < accums_.size(); ) {
            if (dist(accums_[k].t.x, accums_[k].t.y, fx, fy) <= black_radius_)
                accums_.erase(accums_.begin() + static_cast<long>(k));
            else
                ++k;
        }
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
