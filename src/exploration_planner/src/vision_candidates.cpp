#include "exploration_planner/vision_candidates.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace exploration {
namespace {
double distance(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
bool finite(const Vec2& point) { return std::isfinite(point.x) && std::isfinite(point.y); }
int bits(unsigned mask) { return __builtin_popcount(mask); }
}  // namespace

VisionCandidates::VisionCandidates(const VisionCandidateConfig& config) : cfg_(config)
{
    if (cfg_.confirm_frames < 1 || !std::isfinite(cfg_.association_radius) ||
        cfg_.association_radius <= 0.0 || !std::isfinite(cfg_.weight_min_distance) ||
        cfg_.weight_min_distance <= 0.0 || !std::isfinite(cfg_.frame_gap_timeout) ||
        cfg_.frame_gap_timeout <= 0.0 || !std::isfinite(cfg_.tentative_timeout) ||
        cfg_.tentative_timeout < cfg_.frame_gap_timeout || cfg_.max_tracks < 1 || cfg_.max_tracks > 1024)
        throw std::invalid_argument("Invalid vision candidate confirmation/association limits");
}

void VisionCandidates::break_streak(Track& track)
{
    if (track.state.confirmed) return;
    track.state.consecutive_frames = 0;
    track.total_weight = 0.0;
    // Keep the last spatial anchor until tentative expiry, but do not reuse any
    // of its old samples in the next consecutive confirmation window.
}

void VisionCandidates::interrupt()
{
    for (auto& track : tracks_) break_streak(track);
}

void VisionCandidates::reset()
{
    tracks_.clear();
    next_track_id_ = 1;
    last_seq_ = last_epoch_ = 0;
    last_stamp_ = 0;
    have_frame_ = false;
    stats_ = {};
}

void VisionCandidates::discard_visited(std::uint64_t track_id, const Vec2& final_position, double radius)
{
    if (!finite(final_position) || !std::isfinite(radius) || radius < 0) return;
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& track) {
        return track.state.track_id == track_id || distance(track.state.position, final_position) <= radius;
    }), tracks_.end());
}

void VisionCandidates::expire(double now)
{
    if (!std::isfinite(now)) { interrupt(); return; }
    for (auto& track : tracks_)
        if (now < track.state.last_seen || now - track.state.last_seen > cfg_.frame_gap_timeout)
            break_streak(track);
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& track) {
        return !track.state.confirmed && now - track.state.last_seen > cfg_.tentative_timeout;
    }), tracks_.end());
}

void VisionCandidates::observe(Track& track, const Vec2& point, double weight,
                               double stamp, double now, std::vector<VisionCandidate>& newly_confirmed)
{
    track.state.last_seen = now;
    if (track.state.confirmed) return;  // Coarse execution target must not chase every new pixel.
    if (track.total_weight == 0.0) {
        track.state.position = point;
        track.total_weight = weight;
    } else {
        track.total_weight += weight;
        const double ratio = weight / track.total_weight;
        track.state.position.x += ratio * (point.x - track.state.position.x);
        track.state.position.y += ratio * (point.y - track.state.position.y);
    }
    ++track.state.consecutive_frames;
    if (track.state.consecutive_frames == cfg_.confirm_frames) {
        track.state.confirmed = true;
        track.state.confirmed_stamp = stamp;
        ++stats_.confirmations;
        newly_confirmed.push_back(track.state);
    }
}

std::vector<VisionCandidate> VisionCandidates::update(
    const VisionFrame& frame, const Vec2& aircraft, double now)
{
    std::vector<VisionCandidate> newly_confirmed;
    if (!finite(aircraft) || !std::isfinite(now) || !std::isfinite(frame.stamp) ||
        frame.stamp <= 0 || frame.seq == 0 || (frame.seq & 1) || frame.count > frame.targets.size()) {
        interrupt();
        return newly_confirmed;
    }
    if (have_frame_ && frame.epoch == last_epoch_ &&
        (frame.seq == last_seq_ || frame.stamp <= last_stamp_)) return newly_confirmed;
    if (have_frame_ && (frame.epoch != last_epoch_ || frame.restarted ||
        frame.seq < last_seq_ || frame.stamp - last_stamp_ > cfg_.frame_gap_timeout)) interrupt();
    expire(now);
    have_frame_ = true;
    last_epoch_ = frame.epoch;
    last_seq_ = frame.seq;
    last_stamp_ = frame.stamp;
    ++stats_.frames;

    struct Observation { Vec2 point; double weight; };
    std::vector<Observation> observations;
    for (std::size_t i = 0; i < frame.count; ++i) {
        const Vec2 point{frame.targets[i].x, frame.targets[i].y};
        const double d = distance(point, aircraft);
        // Proportional to 1/max(d,d_min)^2, normalized so max weight is 1.
        // ID is deliberately never examined during far-range accumulation.
        const double ratio = cfg_.weight_min_distance / std::max(d, cfg_.weight_min_distance);
        const double weight = ratio * ratio;
        if (!finite(point) || !std::isfinite(d) || !(weight > 0.0) || !std::isfinite(weight)) {
            ++stats_.invalid_detections;
            continue;
        }
        observations.push_back({point, weight});
    }
    // Input slot ordering and category flicker must not decide association ties.
    std::sort(observations.begin(), observations.end(), [](const Observation& a, const Observation& b) {
        return a.point.x != b.point.x ? a.point.x < b.point.x : a.point.y < b.point.y;
    });

    // At most six observations: DP over their bit mask gives a maximum-cardinality,
    // minimum-distance one-to-one matching in O(tracks * 64 * 6). Greedy matching
    // can steal the only observation of a neighbouring track and break its streak.
    const std::size_t n = tracks_.size(), states = std::size_t{1} << observations.size();
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> cost(states, infinity);
    struct Parent { unsigned mask = 0; int observation = -1; };
    std::vector<std::vector<Parent>> parents(n + 1, std::vector<Parent>(states));
    cost[0] = 0;
    for (std::size_t t = 0; t < n; ++t) {
        std::vector<double> next = cost;
        for (unsigned mask = 0; mask < states; ++mask) parents[t + 1][mask] = {mask, -1};
        for (unsigned mask = 0; mask < states; ++mask) {
            if (!std::isfinite(cost[mask])) continue;
            for (std::size_t j = 0; j < observations.size(); ++j) {
                if (mask & (1u << j)) continue;
                const double d = distance(tracks_[t].state.position, observations[j].point);
                if (d > cfg_.association_radius) continue;
                const unsigned assigned = mask | (1u << j);
                const double candidate = cost[mask] + d * d;
                if (candidate < next[assigned]) {
                    next[assigned] = candidate;
                    parents[t + 1][assigned] = {mask, static_cast<int>(j)};
                }
            }
        }
        cost.swap(next);
    }
    unsigned best = 0;
    for (unsigned mask = 0; mask < states; ++mask)
        if (std::isfinite(cost[mask]) && (bits(mask) > bits(best) ||
            (bits(mask) == bits(best) && cost[mask] < cost[best]))) best = mask;
    std::vector<int> matched(n, -1);
    unsigned mask = best;
    for (std::size_t t = n; t > 0; --t) {
        matched[t - 1] = parents[t][mask].observation;
        mask = parents[t][mask].mask;
    }

    // Suppress unmatched duplicate boxes against the *pre-update* anchors.
    // A matched track moving slightly this frame must not expose a duplicate.
    std::vector<Vec2> occupied;
    for (const auto& track : tracks_) occupied.push_back(track.state.position);
    for (std::size_t t = 0; t < n; ++t) {
        if (matched[t] < 0) { break_streak(tracks_[t]); continue; }
        const auto& obs = observations[static_cast<std::size_t>(matched[t])];
        observe(tracks_[t], obs.point, obs.weight, frame.stamp, now, newly_confirmed);
    }
    for (std::size_t j = 0; j < observations.size(); ++j) {
        if (best & (1u << j)) continue;
        const auto& obs = observations[j];
        const bool duplicate = std::any_of(occupied.begin(), occupied.end(), [&](const Vec2& p) {
            return distance(p, obs.point) <= cfg_.association_radius;
        });
        if (duplicate) { ++stats_.suppressed_duplicates; continue; }
        if (tracks_.size() >= cfg_.max_tracks) { ++stats_.capacity_drops; continue; }
        Track track;
        track.state.track_id = next_track_id_++;
        observe(track, obs.point, obs.weight, frame.stamp, now, newly_confirmed);
        occupied.push_back(obs.point);
        tracks_.push_back(track);
    }
    return newly_confirmed;
}

std::vector<VisionCandidate> VisionCandidates::snapshot() const
{
    std::vector<VisionCandidate> result;
    result.reserve(tracks_.size());
    for (const auto& track : tracks_) result.push_back(track.state);
    return result;
}

}  // namespace exploration
