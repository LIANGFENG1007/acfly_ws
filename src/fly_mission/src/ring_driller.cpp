#include "fly_mission/ring_driller.hpp"
#include "fly_mission/params.hpp"

#include <cmath>

namespace fly_mission {

RingDriller::RingDriller(rclcpp::Node* node)
    : node_(node)
{
    // 独立 Reentrant 组：后台收 /ring_detector/center 攒帧，不阻塞 20Hz 状态机 timer
    cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = cbg_;
    sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/ring_detector/center", rclcpp::QoS(10),
        std::bind(&RingDriller::on_center, this, std::placeholders::_1), opt);
}

void RingDriller::begin()
{
    std::lock_guard<std::mutex> lk(mtx_);
    sx_.clear(); sy_.clear(); sz_.clear(); scos_.clear(); ssin_.clear();
    collecting_        = true;      // 从下一拍起接收样本
    phase_             = Phase::COLLECT;
    collect_time_valid_ = false;    // 首拍再起采集计时
}

// 后台线程：仅采集态记录。data 至少 5 个 [x,y,z,yaw_err,ring_yaw_deg]。
void RingDriller::on_center(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 5) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[钻圈] /ring_detector/center 只有 %zu 个数(需≥5含 ring_yaw_deg)，本条忽略",
            msg->data.size());
        return;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    if (!collecting_) return;
    const double x = msg->data[0];
    const double y = msg->data[1];
    const double z = msg->data[2];
    const double ring_yaw = msg->data[4] * M_PI / 180.0;   // 环面绝对朝向
    sx_.push_back(x);
    sy_.push_back(y);
    sz_.push_back(z);
    scos_.push_back(std::cos(ring_yaw));
    ssin_.push_back(std::sin(ring_yaw));
}

// 取平均算停靠位姿：环心 = xyz 均值；环面朝向 = atan2(sin均值, cos均值)；
// 停靠点 = 环心沿法向后退 standoff 米；目标 yaw = 环面朝向。
bool RingDriller::compute_target()
{
    double sx = 0, sy = 0, sz = 0, sc = 0, ss = 0;
    size_t n = 0;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        n = sx_.size();
        if (n == 0) return false;
        for (size_t i = 0; i < n; ++i) {
            sx += sx_[i]; sy += sy_[i]; sz += sz_[i];
            sc += scos_[i]; ss += ssin_[i];
        }
    }
    const double cx = sx / static_cast<double>(n);
    const double cy = sy / static_cast<double>(n);
    const double cz = sz / static_cast<double>(n);
    const double ring_yaw = std::atan2(ss, sc);            // (cos,sin) 平均后取角，天然归一
    const double yaw_deg  = ring_yaw * 180.0 / M_PI;       // 正对环面(环前/环后同一朝向)

    const double D  = params::DRILL_STANDOFF_M;
    const double nx = std::cos(ring_yaw);                  // 环面法向(=穿过方向)单位向量
    const double ny = std::sin(ring_yaw);
    // 目标高度 = 环心高度 + 抬升偏移：SLAM 位姿点(雷达)在机身中心上方，直接对准环心机身会偏低，
    //   抬高此偏移让机身中心穿过环心(见 params::DRILL_Z_OFFSET_M)。
    const double tz = cz + params::DRILL_Z_OFFSET_M;

    // 环前：环心沿法向后退 D 米(飞机所在这侧)；环后：沿法向前进 D 米(穿过去那侧)。二者同高 tz。
    front_x_ = cx - D * nx;  front_y_ = cy - D * ny;  front_z_ = tz;  front_yaw_deg_ = yaw_deg;
    back_x_  = cx + D * nx;  back_y_  = cy + D * ny;  back_z_  = tz;  back_yaw_deg_  = yaw_deg;

    RCLCPP_INFO(node_->get_logger(),
        "[钻圈] 用 %zu 帧：环心(%.2f,%.2f,%.2f) 朝向 %.1f° → 环前(%.2f,%.2f) 环后(%.2f,%.2f) 目标z=%.2f(环心%.2f+偏移%.2f)",
        n, cx, cy, cz, yaw_deg, front_x_, front_y_, back_x_, back_y_, tz, cz, params::DRILL_Z_OFFSET_M);
    return true;
}

RingDriller::Step RingDriller::tick(bool reached,
    double& out_x, double& out_y, double& out_z, double& out_yaw_deg)
{
    switch (phase_) {

    case Phase::COLLECT: {
        if (!collect_time_valid_) {
            collect_start_      = node_->now();
            collect_time_valid_ = true;
        }
        size_t n;
        { std::lock_guard<std::mutex> lk(mtx_); n = sx_.size(); }

        const bool enough  = (n >= static_cast<size_t>(params::DRILL_COLLECT_FRAMES));
        const double elapsed = (node_->now() - collect_start_).seconds();
        const bool timeout = (elapsed >= params::DRILL_COLLECT_TIMEOUT_S);

        if (!enough && !timeout) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "[钻圈] 采集中 %zu/%d 帧…", n, params::DRILL_COLLECT_FRAMES);
            return Step::COLLECTING;
        }
        if (!enough && timeout &&
            n < static_cast<size_t>(params::DRILL_MIN_FRAMES)) {
            { std::lock_guard<std::mutex> lk(mtx_); collecting_ = false; }
            RCLCPP_WARN(node_->get_logger(),
                "[钻圈] 采集 %.1fs 仅 %zu 帧(<%d)，判定没看到圆环 → 放弃",
                elapsed, n, params::DRILL_MIN_FRAMES);
            phase_ = Phase::FAIL;
            return Step::FAILED;
        }
        if (timeout && !enough) {
            RCLCPP_WARN(node_->get_logger(),
                "[钻圈] 采集超时(%.1fs)，用现有 %zu 帧继续", elapsed, n);
        }
        // 攒够 或 超时但够最小帧数 → 算环前/环后位姿，进入 APPROACH_FRONT
        { std::lock_guard<std::mutex> lk(mtx_); collecting_ = false; }
        if (!compute_target()) {
            phase_ = Phase::FAIL;
            return Step::FAILED;
        }
        phase_ = Phase::APPROACH_FRONT;
        out_x = front_x_; out_y = front_y_; out_z = front_z_; out_yaw_deg = front_yaw_deg_;
        return Step::APPROACHING;
    }

    case Phase::APPROACH_FRONT:
        out_x = front_x_; out_y = front_y_; out_z = front_z_; out_yaw_deg = front_yaw_deg_;
        if (reached) {                       // MOVE_POSE 到位(xyz+yaw 全进容差且稳定)
            RCLCPP_INFO(node_->get_logger(), "[钻圈] 已到环前对准位姿 → 悬停");
            phase_ = Phase::HOVER_FRONT;
            return Step::HOVERING;
        }
        return Step::APPROACHING;

    case Phase::HOVER_FRONT:
        if (reached) {                       // 环前悬停(wait_time)到点 → 穿过圈飞环后
            RCLCPP_INFO(node_->get_logger(), "[钻圈] 环前悬停完 → 穿过圈飞向环后");
            phase_ = Phase::THROUGH;
            out_x = back_x_; out_y = back_y_; out_z = back_z_; out_yaw_deg = back_yaw_deg_;
            return Step::APPROACHING;
        }
        return Step::HOVERING;

    case Phase::THROUGH:
        out_x = back_x_; out_y = back_y_; out_z = back_z_; out_yaw_deg = back_yaw_deg_;
        if (reached) {                       // 已飞到环后位姿(穿过圈)
            RCLCPP_INFO(node_->get_logger(), "[钻圈] 已钻到环后 → 悬停");
            phase_ = Phase::HOVER_BACK;
            return Step::HOVERING;
        }
        return Step::APPROACHING;

    case Phase::HOVER_BACK:
        if (reached) {                       // 环后悬停到点 → 完成
            phase_ = Phase::DONE;
            return Step::DONE;
        }
        return Step::HOVERING;

    case Phase::DONE:
        return Step::DONE;

    case Phase::FAIL:
        return Step::FAILED;
    }
    return Step::FAILED;
}

}  // namespace fly_mission
