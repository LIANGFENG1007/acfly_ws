#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include "exploration_planner/vision_shm.hpp"
#include "exploration_planner/vision_target_queue.hpp"

namespace exploration {

struct VisualDropConfig {
    double entry_distance = 1.0, sample_seconds = 1.0, drop_height = .5;
    double approach_slow_distance = 1.20, entry_speed = 0.08;
    double xy_tolerance = .10, stable_seconds = 2.0, timeout_seconds = 4.0;
    double offset = .23, descend_speed = .40, climb_speed = .60;
    double association_radius = .60, weight_min_distance = .50, observation_gap = .30;
};

// Pure mission state; control/obstacle checking and serial I/O stay with their
// owners. Every clock argument is local monotonic time, never a ROS wall clock.
class VisualDrop {
public:
    enum class Phase { Idle, Approach, Sample, Descend, Send, Climb, Done };
    explicit VisualDrop(VisualDropConfig cfg = {}) : cfg_(cfg) {
        for (double v : {cfg.entry_distance, cfg.sample_seconds, cfg.drop_height,
                cfg.xy_tolerance, cfg.stable_seconds, cfg.timeout_seconds, cfg.offset,
                cfg.descend_speed, cfg.climb_speed, cfg.association_radius, cfg.weight_min_distance, cfg.observation_gap,
                cfg.approach_slow_distance, cfg.entry_speed})
            if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid visual drop configuration");
        if (cfg.timeout_seconds < cfg.stable_seconds) throw std::invalid_argument("Drop timeout before stable interval");
    }
    void reset() { *this = VisualDrop(cfg_); }
    void begin(const VisionVisitTarget& target) {
        if (completed_ >= 3 || phase_ != Phase::Idle || !target.track_id) return;
        target_id_ = target.track_id; object_ = coarse_ = target.position;
        phase_ = Phase::Approach; samples_ = 0; weight_ = 0; votes_ = {};
        xy_since_ = stable_since_ = -1; have_frame_ = false; refinement_.reset();
        used_coarse_ = true; forced_ = false; last_tick_ = -1;
    }
    void sample(const VisionFrame& frame, Vec2 aircraft, double now, const std::vector<Vec2>& others) {
        if (phase_ != Phase::Sample || !std::isfinite(aircraft.x) || !std::isfinite(aircraft.y) ||
            !std::isfinite(now) || now < sample_start_ ||
            now >= sample_start_ + cfg_.sample_seconds || !std::isfinite(frame.stamp) ||
            frame.stamp <= sample_start_ || frame.stamp > now + .02 || now - frame.stamp > .5 ||
            frame.count > frame.targets.size() || !frame.seq || (frame.seq & 1)) return;
        if (have_frame_ && (frame.stamp <= last_stamp_ ||
            (frame.epoch == last_epoch_ && frame.seq <= last_seq_))) return;
        last_seq_ = frame.seq; last_epoch_ = frame.epoch; last_stamp_ = frame.stamp; have_frame_ = true;
        const VisionDetection* selected = nullptr;
        double nearest = cfg_.association_radius;
        bool ambiguous = false;
        for (size_t i = 0; i < frame.count; ++i) {
            const auto& d = frame.targets[i];
            if (!std::isfinite(d.x) || !std::isfinite(d.y)) continue;
            const Vec2 p{d.x, d.y};
            const double distance = norm(p, coarse_);
            if (distance > cfg_.association_radius || std::any_of(others.begin(), others.end(),
                [&](Vec2 other) { return norm(p, other) <= distance + 1e-9; })) continue;
            if (!selected || distance < nearest - 1e-9) {
                selected = &d; nearest = distance; ambiguous = false;
            } else if (std::abs(distance - nearest) <= 1e-9 &&
                (d.x != selected->x || d.y != selected->y)) ambiguous = true;
        }
        // Missing/ambiguous frames add nothing; they never erase earlier samples.
        if (!selected || ambiguous) return;
        const Vec2 p{selected->x, selected->y};
        const double ratio = cfg_.weight_min_distance / std::max(cfg_.weight_min_distance, norm(p, aircraft));
        const double w = ratio * ratio;
        if (!std::isfinite(w) || w <= 0) return;
        if (!samples_) mean_ = p;
        else { const double f = w / (weight_ + w); mean_.x += f * (p.x - mean_.x); mean_.y += f * (p.y - mean_.y); }
        weight_ += w; ++samples_; accepted_stamp_ = frame.stamp;
        if (selected->id >= 1 && selected->id <= 6) ++votes_[selected->id];
    }
    void tick(Vec2 position, double z, double yaw, double now, double home_z, double z_tolerance) {
        if (!std::isfinite(z) || !std::isfinite(yaw) || !std::isfinite(now) ||
            !std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(home_z) || !std::isfinite(z_tolerance) || z_tolerance <= 0) return;
        // A delayed control loop cannot prove the aircraft stayed within the
        // tolerance while it was not observing. Keep only the four-second deadline.
        if (last_tick_ >= 0 && (now < last_tick_ || now-last_tick_ > cfg_.observation_gap))
            stable_since_ = -1;
        last_tick_ = now;
        if (phase_ == Phase::Approach && norm(position, object_) <= cfg_.entry_distance) {
            anchor_ = position; return_z_ = z; yaw_ = yaw; sample_start_ = now;
            phase_ = Phase::Sample;
        }
        if (phase_ == Phase::Sample && now - sample_start_ >= cfg_.sample_seconds) {
            if (samples_) {
                object_ = mean_; used_coarse_ = false;
                int id = 0, best = 0; bool tie = false;
                for (int i = 1; i <= 6; ++i) {
                    if (votes_[i] > best) { id = i; best = votes_[i]; tie = false; }
                    else if (votes_[i] && votes_[i] == best) tie = true;
                }
                refinement_ = VisionRefinement{target_id_, object_, tie ? 0 : id, samples_, accepted_stamp_};
            }
            phase_ = Phase::Descend;
        }
        if (phase_ == Phase::Descend) {
            const bool xy_ok = norm(position, lidar_target(yaw)) <= cfg_.xy_tolerance;
            const bool z_ok = std::abs(z - (home_z + cfg_.drop_height)) <= z_tolerance;
            // XY alone starts the irreversible four-second release deadline.
            if (xy_ok && xy_since_ < 0) xy_since_ = now;
            if (xy_ok && z_ok) { if (stable_since_ < 0) stable_since_ = now; }
            else stable_since_ = -1;
            const bool stable = stable_since_ >= 0 && now - stable_since_ >= cfg_.stable_seconds;
            forced_ = xy_since_ >= 0 && now - xy_since_ >= cfg_.timeout_seconds;
            if (stable || forced_) phase_ = Phase::Send;
        }
        if (phase_ == Phase::Climb && std::abs(z - return_z_) <= z_tolerance) phase_ = Phase::Done;
    }
    void reject_refinement() {
        object_ = coarse_; used_coarse_ = true; refinement_.reset();
        // tick() may have observed arrival at the just-computed precise point.
        // That point was rejected; it must not start the coarse target's timers.
        xy_since_ = stable_since_ = -1;
    }
    void pose_interrupted() { stable_since_ = -1; }
    bool serial_finished(unsigned index, std::uint64_t target_id, Vec2 position) {
        if (phase_ != Phase::Send || index != completed_ + 1 || target_id != target_id_ ||
            !std::isfinite(position.x) || !std::isfinite(position.y)) return false;
        anchor_ = position; ++completed_; phase_ = Phase::Climb;
        return true;
    }
    void finish_visit() { if (phase_ == Phase::Done) { phase_ = Phase::Idle; target_id_ = 0; } }
    Vec2 lidar_target(double actual_yaw) const {
        if (phase_ == Phase::Sample || phase_ == Phase::Climb || phase_ == Phase::Done) return anchor_;
        if (phase_ == Phase::Approach) return object_;
        const unsigned index = completed_ + 1;
        const double forward = index == 1 ? cfg_.offset : 0;
        const double left = index == 2 ? cfg_.offset : (index == 3 ? -cfg_.offset : 0);
        const double c = std::cos(actual_yaw), s = std::sin(actual_yaw);
        return {object_.x - c * forward + s * left, object_.y - s * forward - c * left};
    }
    double altitude(double home_z) const {
        return phase_ == Phase::Descend || phase_ == Phase::Send ? home_z + cfg_.drop_height : return_z_;
    }
    // Slow down for the sampling circle, which is reached before the object.
    // A positive crossing speed avoids asymptotically stopping outside the
    // distance-only phase transition. This is a command cap, not an arrival gate.
    double approach_speed_limit(Vec2 position, double cruise_speed) const {
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(cruise_speed) || cruise_speed <= 0.0) return 0.0;
        const double entry = std::min(cfg_.entry_speed, cruise_speed);
        const double fraction = std::clamp((norm(position, object_) - cfg_.entry_distance) /
            cfg_.approach_slow_distance, 0.0, 1.0);
        // Ease towards the crossing speed early enough for delayed velocity
        // tracking; a linear ramp leaves too much residual speed at the circle.
        return entry + (cruise_speed - entry) * fraction * fraction;
    }
    double vertical_limit(double z, double home_z, double remaining_xy, double toward_speed) const {
        if (phase_ == Phase::Climb || phase_ == Phase::Done) return cfg_.climb_speed;
        if (phase_ != Phase::Descend && phase_ != Phase::Send) return .10;
        // Estimate the remaining horizontal flight time; speed up descent when
        // XY approaches its goal, but never exceed the old 0.40 m/s drop limit.
        const double seconds = std::max(.5, remaining_xy / std::clamp(toward_speed, .15, .40));
        return std::clamp(std::abs(z - altitude(home_z)) / seconds, std::min(.10, cfg_.descend_speed), cfg_.descend_speed);
    }
    Phase phase() const { return phase_; }
    unsigned completed() const { return completed_; }
    double package_offset() const { return cfg_.offset; }
    unsigned release_index() const { return phase_ == Phase::Send ? completed_ + 1 : 0; }
    std::uint64_t target_id() const { return target_id_; }
    Vec2 object() const { return object_; }
    double yaw() const { return yaw_; }
    double return_z() const { return return_z_; }
    int samples() const { return samples_; }
    bool used_coarse() const { return used_coarse_; }
    bool forced() const { return forced_; }
    double xy_since() const { return xy_since_; }
    const std::optional<VisionRefinement>& refinement() const { return refinement_; }
    static const char* name(Phase p) {
        switch (p) {
            case Phase::Idle: return "idle"; case Phase::Approach: return "approach";
            case Phase::Sample: return "sample_hold"; case Phase::Descend: return "descend_align";
            case Phase::Send: return "serial_wait"; case Phase::Climb: return "climb_hold";
            case Phase::Done: return "done";
        }
        return "invalid";
    }
private:
    static double norm(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }
    VisualDropConfig cfg_;
    Phase phase_ = Phase::Idle;
    unsigned completed_ = 0;
    std::uint64_t target_id_ = 0, last_seq_ = 0, last_epoch_ = 0;
    Vec2 object_{}, coarse_{}, anchor_{}, mean_{};
    double return_z_ = 0, yaw_ = 0, sample_start_ = 0, xy_since_ = -1, stable_since_ = -1;
    double weight_ = 0, last_stamp_ = 0, accepted_stamp_ = 0, last_tick_ = -1;
    bool have_frame_ = false, used_coarse_ = true, forced_ = false;
    int samples_ = 0;
    std::array<int, 7> votes_{};
    std::optional<VisionRefinement> refinement_;
};
}  // namespace exploration
