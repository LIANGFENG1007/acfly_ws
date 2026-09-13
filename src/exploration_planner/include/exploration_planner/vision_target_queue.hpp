#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_set>
#include <vector>

#include "exploration_planner/vision_candidates.hpp"

namespace exploration {

struct VisionQueueConfig {
    double association_radius = 0.60;
    double blacklist_radius = 0.60;
    double retry_delay = 5.0;
    double retry_observation_age = 0.50;
    std::size_t max_pending = 64;
};

struct VisionVisitTarget {
    std::uint64_t track_id = 0; // Spatial identity, independent of class ID.
    Vec2 coarse_position;
    Vec2 position;
    double confirmed_stamp = 0;
    double last_seen = 0;
    double retry_after = 0;
    int class_id = 0;          // Unknown until near-range voting completes.
    bool refined = false;
};

struct VisionRefinement {
    std::uint64_t track_id = 0;
    Vec2 position;
    int class_id = 0;
    int samples = 0;
    double stamp = 0;
};

struct VisionVisited {
    std::uint64_t track_id = 0;
    Vec2 position;
    int class_id = 0;
    double completed_at = 0;
};

class VisionTargetQueue {
public:
    struct Statistics {
        std::uint64_t enqueued = 0, duplicates = 0, capacity_drops = 0;
        std::uint64_t blacklisted_detections = 0, deferred = 0, retried = 0;
    };
    explicit VisionTargetQueue(const VisionQueueConfig& config = {});
    void reset();
    // FIFO across confirmation batches. Same-frame confirmations prefer nearer
    // points, with spatial track ID as the final deterministic tie-breaker.
    void enqueue(std::vector<VisionCandidate> candidates, const Vec2& aircraft);
    bool activate_next(double now);
    bool refine_active(const VisionRefinement& result);
    bool defer_active(std::uint64_t track_id, double now);
    // Called only by the future executor after arrival and its hover timer.
    // Near confirmation alone MUST NOT call this or mark the target visited.
    bool complete_active(std::uint64_t track_id, double now);
    void observe(const VisionFrame& frame, double now);
    VisionFrame filter_blacklisted(const VisionFrame& frame);
    bool blacklisted(const Vec2& point) const;

    const std::optional<VisionVisitTarget>& active() const { return active_; }
    const std::deque<VisionVisitTarget>& pending() const { return pending_; }
    const std::vector<VisionVisitTarget>& deferred() const { return deferred_; }
    const std::vector<VisionVisited>& visited() const { return visited_; }
    const Statistics& statistics() const { return stats_; }
    std::vector<Vec2> other_positions(std::uint64_t except_track) const;
    double blacklist_radius() const { return cfg_.blacklist_radius; }

private:
    bool duplicate(const VisionCandidate& candidate) const;
    VisionQueueConfig cfg_;
    std::optional<VisionVisitTarget> active_;
    std::deque<VisionVisitTarget> pending_;
    std::vector<VisionVisitTarget> deferred_;
    std::vector<VisionVisited> visited_;
    std::unordered_set<std::uint64_t> known_;
    Statistics stats_;
};

}  // namespace exploration
