#pragma once

#include <algorithm>
#include <cmath>
#include "exploration_planner/types.hpp"

namespace exploration {

// Slew the command, never the measured aircraft velocity. A change opposing
// measured motion may brake faster than ordinary acceleration.
inline Vec2 slew_velocity(Vec2 previous, Vec2 desired, Vec2 measured,
                          double acceleration, double braking, double dt)
{
    const Vec2 delta{desired.x - previous.x, desired.y - previous.y};
    const bool opposing_motion = delta.x * measured.x + delta.y * measured.y < 0.0;
    const double rate = opposing_motion ? std::max(acceleration, braking) : acceleration;
    const double length = std::hypot(delta.x, delta.y);
    const double scale = length > 1e-9 ? std::min(1.0, std::max(0.0, rate * dt) / length) : 1.0;
    return {previous.x + scale * delta.x, previous.y + scale * delta.y};
}

}  // namespace exploration
