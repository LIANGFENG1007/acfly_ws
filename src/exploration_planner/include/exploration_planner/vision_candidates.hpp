#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "exploration_planner/types.hpp"
#include "exploration_planner/vision_shm.hpp"

namespace exploration {

struct VisionCandidateConfig {
    int confirm_frames = 10;
    double association_radius = 0.60;
    double weight_min_distance = 0.50;
    double frame_gap_timeout = 0.50;
    double tentative_timeout = 2.0;
    std::size_t max_tracks = 64;
};

// Internal object identity is spatial, never the unreliable far-range class ID.
// A confirmed position is a fixed coarse observation, not an active flight goal.
struct VisionCandidate {
    std::uint64_t track_id = 0;
    Vec2 position;
    int consecutive_frames = 0;
    bool confirmed = false;
    double confirmed_stamp = 0.0;
    double last_seen = 0.0;  // Receiver's monotonic clock.
};

class VisionCandidates {
public:
    struct Statistics {
        std::uint64_t frames = 0;
        std::uint64_t confirmations = 0;
        std::uint64_t suppressed_duplicates = 0;
        std::uint64_t capacity_drops = 0;
        std::uint64_t invalid_detections = 0;
    };

    explicit VisionCandidates(const VisionCandidateConfig& config = {});
    // Only actual new image frames enter here. Repeat seq/time has no effect.
    // Each candidate receives at most one observation per image, including when
    // several boxes overlap. Returns only newly confirmed coarse positions.
    std::vector<VisionCandidate> update(const VisionFrame& frame, const Vec2& aircraft, double now);
    void expire(double now);  // Call without new frames to break long silent gaps.
    void interrupt();         // Invalid/stale aircraft pose or task not collecting.
    void reset();             // New mission: clear observations and confirmations.
    void discard_visited(std::uint64_t track_id, const Vec2& final_position, double radius);
    std::vector<VisionCandidate> snapshot() const;
    const Statistics& statistics() const { return stats_; }
    std::size_t size() const { return tracks_.size(); }

private:
    struct Track {
        VisionCandidate state;
        double total_weight = 0.0;
    };
    void observe(Track& track, const Vec2& point, double weight, double stamp,
                 double now, std::vector<VisionCandidate>& newly_confirmed);
    static void break_streak(Track& track);

    VisionCandidateConfig cfg_;
    std::vector<Track> tracks_;
    std::uint64_t next_track_id_ = 1;
    std::uint64_t last_seq_ = 0, last_epoch_ = 0;
    double last_stamp_ = 0.0;
    bool have_frame_ = false;
    Statistics stats_;
};

}  // namespace exploration
