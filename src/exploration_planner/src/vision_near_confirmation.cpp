#include "exploration_planner/vision_near_confirmation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace exploration {
namespace {
double distance(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
bool finite(const Vec2& point) { return std::isfinite(point.x) && std::isfinite(point.y); }
}

VisionNearConfirmation::VisionNearConfirmation(const VisionNearConfig& config) : cfg_(config)
{
    if (cfg_.confirm_frames < 1 || !std::isfinite(cfg_.near_distance) || cfg_.near_distance <= 0 ||
        !std::isfinite(cfg_.speed_cap) || cfg_.speed_cap <= 0 ||
        !std::isfinite(cfg_.association_radius) || cfg_.association_radius <= 0 ||
        !std::isfinite(cfg_.weight_min_distance) || cfg_.weight_min_distance <= 0 ||
        !std::isfinite(cfg_.frame_gap_timeout) || cfg_.frame_gap_timeout <= 0 ||
        !std::isfinite(cfg_.confirmation_timeout) || cfg_.confirmation_timeout <= 0)
        throw std::invalid_argument("Invalid near-range confirmation parameters");
}

void VisionNearConfirmation::reset()
{
    track_id_ = 0;
    near_entered_ = have_frame_ = false;
    entered_at_ = last_seen_ = exclude_through_stamp_ = last_stamp_ = 0;
    last_seq_ = last_epoch_ = 0;
    result_.reset();
    interrupt();
}

void VisionNearConfirmation::begin(const VisionVisitTarget& target, double now, double exclude_stamp)
{
    if (!target.track_id || !finite(target.position) || !std::isfinite(now) || !std::isfinite(exclude_stamp))
        throw std::invalid_argument("Invalid active target for near confirmation");
    reset();
    track_id_ = target.track_id;
    anchor_ = target.position;
    exclude_through_stamp_ = exclude_stamp; // Do not count the far-confirmation image again.
}

void VisionNearConfirmation::interrupt()
{
    if (result_) return;
    count_ = 0;
    votes_.fill(0);
    total_weight_ = 0;
    weighted_position_ = {};
}

void VisionNearConfirmation::position(const Vec2& aircraft, double now)
{
    if (!track_id_ || !finite(aircraft) || !std::isfinite(now)) { interrupt(); return; }
    if (!near_entered_ && distance(aircraft, anchor_) < cfg_.near_distance) {
        near_entered_ = true;
        entered_at_ = now;
        interrupt();
    }
    if (!result_ && (distance(aircraft, anchor_) >= cfg_.near_distance ||
        (count_ && now - last_seen_ > cfg_.frame_gap_timeout))) interrupt();
    // Once near-range control is entered, the lower cap stays latched for this
    // target; small excursions across 2m must not switch speed back and forth.
}

bool VisionNearConfirmation::timed_out(double now) const
{
    return track_id_ && near_entered_ && !result_ && std::isfinite(now) &&
        now - entered_at_ >= cfg_.confirmation_timeout;
}

double VisionNearConfirmation::speed_cap(double normal_speed) const
{
    return near_entered_ ? std::min(normal_speed, cfg_.speed_cap) : normal_speed;
}

int VisionNearConfirmation::winner() const
{
    int best = 0, maximum = 0;
    bool tie = false;
    for (int id = 1; id <= 6; ++id) {
        if (votes_[id] > maximum) { maximum = votes_[id]; best = id; tie = false; }
        else if (votes_[id] == maximum && maximum > 0) { tie = true; }
    }
    return tie ? 0 : best;
}

std::optional<VisionRefinement> VisionNearConfirmation::update(
    const VisionFrame& frame, const Vec2& aircraft, double now, const std::vector<Vec2>& others)
{
    position(aircraft, now);
    if (!track_id_ || result_ || timed_out(now)) return std::nullopt;
    if (!finite(aircraft) || !std::isfinite(now) || !std::isfinite(frame.stamp) ||
        frame.stamp <= 0 || !frame.seq || (frame.seq & 1) || frame.count > frame.targets.size()) {
        interrupt(); return std::nullopt;
    }
    if (frame.stamp <= exclude_through_stamp_) return std::nullopt;
    if (have_frame_ && frame.epoch == last_epoch_ &&
        (frame.seq == last_seq_ || frame.stamp <= last_stamp_)) return std::nullopt;
    if (have_frame_ && (frame.epoch != last_epoch_ || frame.restarted ||
        frame.seq < last_seq_ || frame.stamp - last_stamp_ > cfg_.frame_gap_timeout)) interrupt();
    have_frame_ = true;
    last_seq_ = frame.seq; last_epoch_ = frame.epoch; last_stamp_ = frame.stamp;
    if (!near_entered_ || distance(aircraft, anchor_) >= cfg_.near_distance) return std::nullopt;

    const VisionDetection* selected = nullptr;
    double closest = std::numeric_limits<double>::infinity();
    bool ambiguous = false;
    for (std::size_t i = 0; i < frame.count; ++i) {
        const auto& detection = frame.targets[i];
        const Vec2 point{detection.x, detection.y};
        if (!finite(point) || detection.id < 1 || detection.id > 6) continue;
        const double d = distance(point, anchor_);
        if (d > cfg_.association_radius || distance(aircraft, point) >= cfg_.near_distance) continue;
        // Do not use another queued object's ID just because it is visible in
        // the same image. Category plays no part in spatial correspondence.
        if (std::any_of(others.begin(), others.end(), [&](const Vec2& other) {
            return finite(other) && distance(point, other) <= d + 1e-9;
        })) continue;
        if (d < closest - 1e-9) {
            closest = d; selected = &detection; ambiguous = false;
        } else if (selected && std::abs(d - closest) <= 1e-9 &&
                   (selected->id != detection.id || selected->x != detection.x || selected->y != detection.y)) {
            ambiguous = true;
        }
    }
    if (!selected || ambiguous) { interrupt(); return std::nullopt; }
    const Vec2 point{selected->x, selected->y};
    const double ratio = cfg_.weight_min_distance / std::max(cfg_.weight_min_distance, distance(aircraft, point));
    const double weight = ratio * ratio;
    if (!(weight > 0.0) || !std::isfinite(weight)) { interrupt(); return std::nullopt; }
    if (total_weight_ == 0) weighted_position_ = point;
    else {
        const double fraction = weight / (total_weight_ + weight);
        weighted_position_.x += fraction * (point.x - weighted_position_.x);
        weighted_position_.y += fraction * (point.y - weighted_position_.y);
    }
    total_weight_ += weight;
    ++count_;
    ++votes_[selected->id];
    last_seen_ = now;
    const int id = winner();
    if (count_ >= cfg_.confirm_frames && id != 0) {
        result_ = VisionRefinement{track_id_, weighted_position_, id, count_, frame.stamp};
        return result_;
    }
    return std::nullopt;
}

}  // namespace exploration
