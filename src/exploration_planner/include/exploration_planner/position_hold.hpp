#pragma once

#include <algorithm>
#include <cmath>
#include "exploration_planner/types.hpp"

namespace exploration {

// Fixed world-frame anchor for an in-place turn. Slew-limit in world coordinates
// so rotating the aircraft does not rotate the correction or relatch its target.
class PositionHold {
public:
    void begin(Vec2 anchor) { anchor_ = anchor; velocity_ = {}; }
    void stop() { velocity_ = {}; }
    Vec2 anchor() const { return anchor_; }
    Vec2 update(Vec2 position, double yaw, double vf, double vl,
                double kp, double kd, double max_speed, double acceleration, double dt)
    {
        const double c = std::cos(yaw), s = std::sin(yaw);
        const Vec2 measured{c * vf - s * vl, s * vf + c * vl};
        Vec2 desired{kp * (anchor_.x - position.x) - kd * measured.x,
                     kp * (anchor_.y - position.y) - kd * measured.y};
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
    Vec2 anchor_, velocity_;
};

}  // namespace exploration
