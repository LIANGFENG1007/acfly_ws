#pragma once

#include <algorithm>
#include <cmath>
#include "exploration_planner/types.hpp"

namespace exploration {

// Fixed world-frame anchor for an in-place turn. Slew-limit in world coordinates
// so rotating the aircraft does not rotate the correction or relatch its target.
class PositionHold {
public:
    // initial_velocity：进入定点时【已发出】的世界系速度指令。默认零(旧行为)会让巡航中进入
    //   停车转向时指令一拍从巡航速跌到 0，飞机点头；从当前指令起步再按限幅刹停则连续。
    //
    // ★锚点跟随滑行★：带速进入时，只要实测速度仍沿进入方向(> kSlideStop)，就把锚点推到当前位置在该方向上的投影，
    //   滑行停下后锚点固定。惯性滑行物理上必然发生，按预测锚点回拉只会多一次反向机动(r49：-0.19 m/s、点头 -2.9°)；
    //   滑停预测依赖 PX4 滞后，实机不确定，跟随不依赖它。
    //   allow_slide=false：只从已发指令起步，不跟随滑行、允许反推(必达点保持原行为)。
    //   max_slide：锚点因跟随滑行累计前移的上限(m)，超过即固定锚点并恢复反推。
    void begin(Vec2 anchor, Vec2 initial_velocity = {}, bool allow_slide = true, double max_slide = 0.30)
    {
        anchor_ = anchor; velocity_ = initial_velocity;
        const double speed = std::hypot(initial_velocity.x, initial_velocity.y);
        sliding_ = allow_slide && speed > kSlideStop;
        slide_dir_ = sliding_ ? Vec2{initial_velocity.x / speed, initial_velocity.y / speed} : Vec2{};
        slide_left_ = std::max(0.0, max_slide);
    }
    // 调用方发现实测滑行方向不安全时立即结束滑行：锚点固定、恢复阻尼反推。
    void end_slide() { sliding_ = false; }
    bool sliding() const { return sliding_; }
    // 调用方改写了本拍实际发出的指令(守卫裁剪/置零)时同步内部斜坡状态，避免约束解除后跳回旧值。
    void set_velocity(Vec2 world_velocity) { velocity_ = world_velocity; }
    void stop() { velocity_ = {}; }
    Vec2 anchor() const { return anchor_; }
    Vec2 update(Vec2 position, double yaw, double vf, double vl,
                double kp, double kd, double max_speed, double acceleration, double dt)
    {
        const double c = std::cos(yaw), s = std::sin(yaw);
        const Vec2 measured{c * vf - s * vl, s * vf + c * vl};
        if (sliding_) {
            if (measured.x * slide_dir_.x + measured.y * slide_dir_.y > kSlideStop) {
                const double ahead = std::min(slide_left_,
                    (position.x - anchor_.x) * slide_dir_.x + (position.y - anchor_.y) * slide_dir_.y);
                if (ahead > 0.0) {
                    anchor_.x += ahead * slide_dir_.x; anchor_.y += ahead * slide_dir_.y;
                    slide_left_ -= ahead;
                    if (slide_left_ <= 1e-6) sliding_ = false;
                }
            } else {
                sliding_ = false;
            }
        }
        Vec2 desired{kp * (anchor_.x - position.x) - kd * measured.x,
                     kp * (anchor_.y - position.y) - kd * measured.y};
        if (sliding_) {
            // 滑行中不反推：阻尼项 -kd·v 会要求反向速度(0.3 m/s 时约 -0.1)，就是点头的来源。
            //   沿滑行方向只允许减到 0，由指令斜坡与 PX4 自身刹停；横向纠偏保留。
            const double along = desired.x * slide_dir_.x + desired.y * slide_dir_.y;
            if (along < 0.0) { desired.x -= along * slide_dir_.x; desired.y -= along * slide_dir_.y; }
        }
        const double speed = std::hypot(desired.x, desired.y);
        if (speed > max_speed && speed > 0.0) {
            desired.x *= max_speed / speed; desired.y *= max_speed / speed;
        }
        const Vec2 delta{desired.x - velocity_.x, desired.y - velocity_.y};
        const double change = std::hypot(delta.x, delta.y);
        const double scale = change > 0.0 ? std::min(1.0, acceleration * dt / change) : 1.0;
        velocity_.x += scale * delta.x;
        velocity_.y += scale * delta.y;
        return {c * velocity_.x + s * velocity_.y, -s * velocity_.x + c * velocity_.y};
    }
private:
    static constexpr double kSlideStop = 0.05;  // m/s
    Vec2 anchor_, velocity_, slide_dir_;
    bool sliding_ = false;
    double slide_left_ = 0.0;
};

}  // namespace exploration
