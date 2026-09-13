#include "exploration_planner/vision_candidates.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace exploration;
namespace {
void require(bool ok, const std::string& message)
{
    if (!ok) throw std::runtime_error(message);
}
void near(double a, double b, const std::string& message)
{
    require(std::abs(a - b) < 1e-9, message + " actual=" + std::to_string(a));
}
VisionFrame frame(int index, std::initializer_list<VisionDetection> targets)
{
    VisionFrame result;
    result.seq = index * 2;
    result.stamp = 100 + index * .05;
    result.epoch = 1;
    result.count = targets.size();
    std::copy(targets.begin(), targets.end(), result.targets.begin());
    return result;
}
std::vector<VisionCandidate> send(VisionCandidates& candidates, VisionFrame image, Vec2 aircraft = {})
{
    return candidates.update(image, aircraft, image.stamp + .01);
}

void ten_frames_ignore_category_and_freeze_confirmed_position()
{
    VisionCandidates candidates;
    for (int i = 1; i <= 10; ++i) {
        const auto image = frame(i, {{1 + i % 6, 2, -1}});
        const auto confirmed = send(candidates, image);
        require(confirmed.size() == static_cast<size_t>(i == 10), "confirmation happened before/after frame ten");
        require(candidates.size() == 1, "unreliable class ID split one spatial object");
        require(candidates.snapshot()[0].consecutive_frames == i, "new image count incorrect");
        require(send(candidates, image).empty() && candidates.snapshot()[0].consecutive_frames == i,
                "rereading the same image incremented confirmation");
    }
    auto after = send(candidates, frame(11, {{6, 2.4, -1}}));
    require(after.empty() && candidates.statistics().confirmations == 1, "confirmed point emitted repeatedly");
    near(candidates.snapshot()[0].position.x, 2, "coarse point moved after confirmation");
    send(candidates, frame(12, {}));
    candidates.expire(104);
    require(candidates.size() == 1 && candidates.snapshot()[0].confirmed,
            "temporarily lost confirmed coarse point was erased");
    candidates.reset();
    require(candidates.size() == 0 && candidates.statistics().confirmations == 0, "new mission retained old points");
}

void weighted_coordinates_use_observation_distance()
{
    VisionCandidates candidates;
    double sum_x = 0, sum_y = 0, sum_w = 0;
    for (int i = 1; i <= 10; ++i) {
        const Vec2 point{i <= 5 ? 3.0 : 3.2, i <= 5 ? -.1 : .1};
        const Vec2 aircraft{i <= 5 ? 0.0 : 2.0, 0};
        const double d = std::hypot(point.x - aircraft.x, point.y - aircraft.y);
        const double w = 1.0 / std::pow(std::max(.5, d), 2);
        sum_x += w * point.x; sum_y += w * point.y; sum_w += w;
        send(candidates, frame(i, {{1, point.x, point.y}}), aircraft);
    }
    const auto candidate = candidates.snapshot().at(0);
    require(candidate.confirmed, "weighted fixture did not confirm");
    near(candidate.position.x, sum_x / sum_w, "weighted X differs from direct weighted mean");
    near(candidate.position.y, sum_y / sum_w, "weighted Y differs from direct weighted mean");
    require(candidate.position.x > 3.1, "closer reliable coordinates did not receive more weight");

    VisionCandidateConfig cfg; cfg.confirm_frames = 2;
    VisionCandidates close(cfg);
    send(close, frame(1, {{1, 0, 0}}));
    send(close, frame(2, {{2, .2, 0}}));
    near(close.snapshot()[0].position.x, .1, "weight cap failed near zero distance");
}

void six_spatial_targets_survive_slot_changes()
{
    VisionCandidates candidates;
    for (int i = 1; i <= 10; ++i) {
        auto image = frame(i, {}); image.count = 6;
        for (int j = 0; j < 6; ++j) image.targets[(j + i) % 6] = {i % 6 + 1, 2.0 + j, .2};
        const auto result = send(candidates, image);
        require(candidates.size() == 6, "same-class objects at different positions merged");
        require(result.size() == static_cast<size_t>(i == 10 ? 6 : 0), "six objects did not confirm together");
    }
    for (const auto& candidate : candidates.snapshot())
        require(candidate.confirmed && candidate.consecutive_frames == 10, "slot permutation broke a track");
}

void misses_empty_frames_gaps_and_restarts_reset_only_pending_windows()
{
    VisionCandidateConfig cfg; cfg.confirm_frames = 3;
    VisionCandidates candidates(cfg);
    send(candidates, frame(1, {{1, 2, 0}, {1, 4, 0}}));
    send(candidates, frame(2, {{6, 4, 0}}));
    auto state = candidates.snapshot();
    require(state[0].consecutive_frames == 0 && state[1].consecutive_frames == 2,
            "a missing target reset all targets or kept its own streak");
    auto confirmed = send(candidates, frame(3, {{2, 2.2, 0}, {3, 4, 0}}));
    require(confirmed.size() == 1 && confirmed[0].position.x == 4, "uninterrupted neighbour did not confirm");
    send(candidates, frame(4, {{2, 2.2, 0}}));
    confirmed = send(candidates, frame(5, {{5, 2.2, 0}}));
    require(confirmed.size() == 1, "missed target failed to restart its three-frame window");
    near(confirmed[0].position.x, 2.2, "old pre-miss coordinates leaked into new mean");

    candidates.reset();
    send(candidates, frame(1, {{1, 2, 0}}));
    send(candidates, frame(2, {}));
    send(candidates, frame(3, {{2, 2.1, 0}}));
    require(candidates.snapshot()[0].consecutive_frames == 1, "empty frame failed to break continuity");
    auto restarted = frame(4, {{3, 2.2, 0}}); restarted.epoch = 2; restarted.seq = 2; restarted.restarted = true;
    send(candidates, restarted);
    require(candidates.snapshot()[0].consecutive_frames == 1, "writer restart joined old confirmation samples");
    near(candidates.snapshot()[0].position.x, 2.2, "writer restart retained old weights");

    candidates.reset();
    send(candidates, frame(1, {{1, 2, 0}}));
    auto skipped = frame(2, {{2, 2, 0}}); skipped.seq = 100;
    send(candidates, skipped);
    require(candidates.snapshot()[0].consecutive_frames == 2, "skipped unreceived frames broke continuity");
    candidates.expire(101);
    require(candidates.snapshot()[0].consecutive_frames == 0, "silent stream did not break pending streak");
    auto later = frame(30, {{2, 2.1, 0}});
    send(candidates, later);
    require(candidates.snapshot()[0].consecutive_frames == 1, "long source-time gap reused old samples");
    candidates.expire(104);
    require(candidates.size() == 0, "unconfirmed lost candidates never expired");
}

void duplicate_boxes_and_global_assignment()
{
    VisionCandidates candidates;
    for (int i = 1; i <= 10; ++i) {
        send(candidates, frame(i, {{1, 2, 0}, {2, 2, 0}, {3, 2.01, 0}}));
        require(candidates.size() == 1 && candidates.snapshot()[0].consecutive_frames == i,
                "multiple boxes from one image confirmed too quickly or created duplicate objects");
    }
    require(candidates.statistics().confirmations == 1 && candidates.statistics().suppressed_duplicates == 20,
            "duplicate suppression statistics incorrect");

    candidates.reset();
    send(candidates, frame(1, {{1, 2, 0}, {1, 2.8, 0}}));
    send(candidates, frame(2, {{6, 2.35, 0}, {5, 1.5, 0}}));
    const auto state = candidates.snapshot();
    require(state.size() == 2 && state[0].consecutive_frames == 2 && state[1].consecutive_frames == 2,
            "greedy association stole the sole observation of a neighbouring track");
}

void bounds_invalid_pose_and_interrupt()
{
    VisionCandidateConfig cfg; cfg.max_tracks = 2;
    VisionCandidates candidates(cfg);
    send(candidates, frame(1, {{1, 2, 0}, {2, 4, 0}, {3, 6, 0}}));
    require(candidates.size() == 2 && candidates.statistics().capacity_drops == 1, "candidate capacity not bounded");
    candidates.interrupt();
    for (auto& track : candidates.snapshot()) require(track.consecutive_frames == 0, "pause retained a pending streak");
    send(candidates, frame(2, {{1, 2, 0}}), {std::numeric_limits<double>::quiet_NaN(), 0});
    require(candidates.snapshot()[0].consecutive_frames == 0, "invalid aircraft position produced a weight");
    candidates.expire(104);
    send(candidates, frame(81, {{1, 2, 0}}));
    require(candidates.size() == 1 && candidates.snapshot()[0].consecutive_frames == 1,
            "expired tentative tracks did not release capacity");
}
}  // namespace

int main()
{
    try {
        ten_frames_ignore_category_and_freeze_confirmed_position();
        weighted_coordinates_use_observation_distance();
        six_spatial_targets_survive_slot_changes();
        misses_empty_frames_gaps_and_restarts_reset_only_pending_windows();
        duplicate_boxes_and_global_assignment();
        bounds_invalid_pose_and_interrupt();
        std::cout << "vision_candidates_test: spatial identity, 10-frame confirmation and weighted coordinates passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
