#pragma once

#include <array>
#include <optional>
#include <vector>

#include "exploration_planner/vision_target_queue.hpp"

namespace exploration {

struct VisionNearConfig {
    int confirm_frames = 10;
    double near_distance = 2.0;
    double speed_cap = 0.4;
    double association_radius = 0.6;
    double weight_min_distance = 0.5;
    double frame_gap_timeout = 0.5;
    double confirmation_timeout = 8.0;
};

// Data-only near-range state machine. It requests a speed cap; the flight
// controller will consume it in the later motion-integration stage.
class VisionNearConfirmation {
public:
    explicit VisionNearConfirmation(const VisionNearConfig& config = {});
    void reset();
    void begin(const VisionVisitTarget& target, double now, double exclude_through_stamp);
    void position(const Vec2& aircraft, double now);
    void interrupt();
    std::optional<VisionRefinement> update(const VisionFrame& frame, const Vec2& aircraft,
        double now, const std::vector<Vec2>& other_target_positions = {});

    std::uint64_t track_id() const { return track_id_; }
    bool near_entered() const { return near_entered_; }
    bool confirmed() const { return result_.has_value(); }
    bool timed_out(double now) const;
    int count() const { return count_; }
    bool tied() const { return count_ >= cfg_.confirm_frames && winner() == 0; }
    const std::array<int, 7>& votes() const { return votes_; }
    double speed_cap(double normal_speed) const;
    const std::optional<VisionRefinement>& result() const { return result_; }

private:
    int winner() const;
    VisionNearConfig cfg_;
    std::uint64_t track_id_ = 0;
    Vec2 anchor_;
    bool near_entered_ = false, have_frame_ = false;
    double entered_at_ = 0, last_seen_ = 0, exclude_through_stamp_ = 0;
    std::uint64_t last_seq_ = 0, last_epoch_ = 0;
    double last_stamp_ = 0;
    int count_ = 0;
    std::array<int, 7> votes_{};
    Vec2 weighted_position_;
    double total_weight_ = 0;
    std::optional<VisionRefinement> result_;
};

}  // namespace exploration
