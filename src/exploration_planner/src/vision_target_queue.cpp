#include "exploration_planner/vision_target_queue.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace exploration {
namespace {
double distance(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
bool finite(const Vec2& p) { return std::isfinite(p.x) && std::isfinite(p.y); }
}

VisionTargetQueue::VisionTargetQueue(const VisionQueueConfig& config) : cfg_(config)
{
    if (!std::isfinite(cfg_.association_radius) || cfg_.association_radius <= 0 ||
        !std::isfinite(cfg_.blacklist_radius) || cfg_.blacklist_radius <= 0 ||
        !std::isfinite(cfg_.retry_delay) || cfg_.retry_delay < 0 ||
        !std::isfinite(cfg_.retry_observation_age) || cfg_.retry_observation_age <= 0 ||
        cfg_.max_pending < 1 || cfg_.max_pending > 1024)
        throw std::invalid_argument("Invalid vision queue limits");
}

void VisionTargetQueue::reset()
{
    active_.reset(); pending_.clear(); deferred_.clear(); visited_.clear(); known_.clear(); stats_ = {};
}

bool VisionTargetQueue::blacklisted(const Vec2& point) const
{
    return std::any_of(visited_.begin(), visited_.end(), [&](const VisionVisited& visit) {
        return distance(visit.position, point) <= cfg_.blacklist_radius;
    });
}

bool VisionTargetQueue::duplicate(const VisionCandidate& candidate) const
{
    const auto same = [&](const VisionVisitTarget& item) {
        return item.track_id == candidate.track_id ||
            distance(item.position, candidate.position) <= cfg_.association_radius ||
            distance(item.coarse_position, candidate.position) <= cfg_.association_radius;
    };
    return (active_ && same(*active_)) || std::any_of(pending_.begin(), pending_.end(), same) ||
        std::any_of(deferred_.begin(), deferred_.end(), same);
}

void VisionTargetQueue::enqueue(std::vector<VisionCandidate> candidates, const Vec2& aircraft)
{
    if (!finite(aircraft)) return;
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const VisionCandidate& c) {
        return !c.confirmed || !c.track_id || !finite(c.position) || !std::isfinite(c.confirmed_stamp);
    }), candidates.end());
    std::sort(candidates.begin(), candidates.end(), [&](const VisionCandidate& a, const VisionCandidate& b) {
        if (a.confirmed_stamp != b.confirmed_stamp) return a.confirmed_stamp < b.confirmed_stamp;
        const double da = distance(aircraft, a.position), db = distance(aircraft, b.position);
        return da != db ? da < db : a.track_id < b.track_id;
    });
    for (const auto& candidate : candidates) {
        if (known_.count(candidate.track_id)) continue;
        if (blacklisted(candidate.position) || duplicate(candidate)) {
            known_.insert(candidate.track_id);
            ++stats_.duplicates;
            continue;
        }
        if (pending_.size() + deferred_.size() + (active_ ? 1 : 0) >= cfg_.max_pending) {
            ++stats_.capacity_drops;
            continue; // Not marked known: a later snapshot may retry after capacity frees.
        }
        known_.insert(candidate.track_id);
        pending_.push_back({candidate.track_id, candidate.position, candidate.position,
                            candidate.confirmed_stamp, candidate.last_seen, 0, 0, false});
        ++stats_.enqueued;
    }
}

bool VisionTargetQueue::activate_next(double now)
{
    if (!std::isfinite(now) || active_) return false;
    for (auto it = deferred_.begin(); it != deferred_.end();) {
        if (now >= it->retry_after && it->last_seen >= it->retry_after &&
            now - it->last_seen >= 0 && now - it->last_seen <= cfg_.retry_observation_age) {
            pending_.push_back(*it); // Existing pending targets go first.
            it = deferred_.erase(it);
            ++stats_.retried;
        } else { ++it; }
    }
    while (!pending_.empty()) {
        auto target = pending_.front(); pending_.pop_front();
        if (blacklisted(target.position)) continue;
        active_ = target;
        return true;
    }
    return false;
}

bool VisionTargetQueue::refine_active(const VisionRefinement& result)
{
    if (!active_ || active_->refined || active_->track_id != result.track_id ||
        result.class_id < 1 || result.class_id > 6 || !finite(result.position) ||
        result.samples < 1 || !std::isfinite(result.stamp) ||
        result.stamp <= active_->confirmed_stamp ||
        distance(result.position, active_->coarse_position) > cfg_.association_radius ||
        blacklisted(result.position)) return false;
    active_->position = result.position;
    active_->class_id = result.class_id;
    active_->refined = true;
    return true;
}

bool VisionTargetQueue::defer_active(std::uint64_t track_id, double now)
{
    if (!active_ || active_->track_id != track_id || !std::isfinite(now)) return false;
    auto target = *active_;
    target.coarse_position = target.position;
    target.class_id = 0;
    target.refined = false;
    target.retry_after = now + cfg_.retry_delay;
    target.last_seen = now;
    deferred_.push_back(target);
    active_.reset();
    ++stats_.deferred;
    return true;
}

bool VisionTargetQueue::complete_active(std::uint64_t track_id, double now)
{
    if (!active_ || active_->track_id != track_id || !active_->refined || !std::isfinite(now)) return false;
    visited_.push_back({track_id, active_->position, active_->class_id, now});
    active_.reset();
    const auto suppressed = [&](const VisionVisitTarget& target) { return blacklisted(target.position); };
    pending_.erase(std::remove_if(pending_.begin(), pending_.end(), suppressed), pending_.end());
    deferred_.erase(std::remove_if(deferred_.begin(), deferred_.end(), suppressed), deferred_.end());
    return true;
}

void VisionTargetQueue::observe(const VisionFrame& frame, double now)
{
    if (!std::isfinite(now) || frame.count > frame.targets.size()) return;
    const auto refresh = [&](VisionVisitTarget& item) {
        for (std::size_t i = 0; i < frame.count; ++i) {
            const Vec2 point{frame.targets[i].x, frame.targets[i].y};
            if (finite(point) && !blacklisted(point) && distance(point, item.position) <= cfg_.association_radius) {
                item.last_seen = now;
                return;
            }
        }
    };
    if (active_) refresh(*active_);
    for (auto& item : pending_) refresh(item);
    for (auto& item : deferred_) refresh(item);
}

VisionFrame VisionTargetQueue::filter_blacklisted(const VisionFrame& frame)
{
    VisionFrame filtered = frame;
    filtered.targets = {};
    filtered.count = 0;
    if (frame.count > frame.targets.size()) return filtered;
    for (std::size_t i = 0; i < frame.count; ++i) {
        if (blacklisted({frame.targets[i].x, frame.targets[i].y})) {
            ++stats_.blacklisted_detections;
        } else {
            filtered.targets[filtered.count++] = frame.targets[i];
        }
    }
    return filtered;
}

std::vector<Vec2> VisionTargetQueue::other_positions(std::uint64_t except_track) const
{
    std::vector<Vec2> positions;
    if (active_ && active_->track_id != except_track) positions.push_back(active_->position);
    for (const auto& item : pending_) if (item.track_id != except_track) positions.push_back(item.position);
    for (const auto& item : deferred_) if (item.track_id != except_track) positions.push_back(item.position);
    return positions;
}

}  // namespace exploration
