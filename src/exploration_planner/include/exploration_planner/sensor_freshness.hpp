#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace exploration {

// Source stamps establish frame progression, not age against another clock.
// Reception times and watchdog queries must use the same local steady clock.
class SensorFreshness {
public:
    bool observe(std::int64_t source_stamp, double received_at)
    {
        if (source_stamp < 0 || !std::isfinite(received_at)) return false;
        interval_ = 0.0;
        if (!seen_) {
            seen_ = true;
        } else if (source_stamp == source_stamp_) {
            return false;
        } else if (source_stamp < source_stamp_) {
            source_stamp_ = source_stamp;
            accepted_at_ = invalid_time();
            return false;  // Require a progressing frame after a source restart.
        } else if (std::isfinite(accepted_at_)) {
            interval_ = static_cast<double>(source_stamp - source_stamp_) * 1e-9;
        }
        source_stamp_ = source_stamp;
        accepted_at_ = received_at;
        return true;
    }

    double age(double now) const
    {
        return std::isfinite(accepted_at_) ? now - accepted_at_
                                          : std::numeric_limits<double>::infinity();
    }
    bool fresh(double now, double timeout) const
    {
        const double elapsed = age(now);
        return std::isfinite(elapsed) && elapsed >= 0.0 && elapsed <= timeout;
    }
    double source_interval() const { return interval_; }

private:
    static double invalid_time() { return std::numeric_limits<double>::quiet_NaN(); }
    bool seen_ = false;
    std::int64_t source_stamp_ = 0;
    double accepted_at_ = invalid_time();
    double interval_ = 0.0;
};

}  // namespace exploration
