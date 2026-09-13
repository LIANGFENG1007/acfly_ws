#include "exploration_planner/vision_near_confirmation.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace exploration;
namespace {
void require(bool ok, const char* text) { if (!ok) throw std::runtime_error(text); }
VisionVisitTarget target(std::uint64_t id = 1, double x = 2, double y = 0)
{
    return {id, {x, y}, {x, y}, 100, 100, 0, 0, false};
}
VisionFrame frame(int i, int id, double x = 2.1, double y = 0)
{
    VisionFrame f;
    f.seq = 2 * i; f.stamp = 100 + i * .05; f.epoch = 1;
    f.count = 1; f.targets[0] = {id, x, y};
    return f;
}
std::optional<VisionRefinement> send(VisionNearConfirmation& near, VisionFrame f, Vec2 aircraft = {1, 0},
                                     const std::vector<Vec2>& other = {})
{
    return near.update(f, aircraft, f.stamp + .01, other);
}

void distant_ids_are_excluded_and_near_cap_is_latched()
{
    VisionNearConfirmation near;
    near.begin(target(), 100, 100);
    for (int i = 1; i <= 10; ++i) send(near, frame(i, 6), {-1, 0});
    require(!near.near_entered() && near.count() == 0 && near.speed_cap(.6) == .6,
            "far IDs voted or triggered the close speed cap");
    near.position({0, 0}, 100.6);
    require(!near.near_entered(), "near threshold must be less than 2m");
    std::optional<VisionRefinement> result;
    for (int i = 11; i <= 20; ++i) {
        result = send(near, frame(i, i <= 16 ? 3 : 2));
        require(near.count() == i - 10 && result.has_value() == (i == 20),
                "near confirmation reused distant frames or confirmed at wrong count");
        require(near.speed_cap(.6) == .4 && near.speed_cap(.2) == .2,
                "close cap raised a lower existing limit");
    }
    require(result && result->class_id == 3 && result->samples == 10 && result->position.x == 2.1,
            "near vote was contaminated by far IDs");
    const auto locked = *near.result();
    send(near, frame(21, 6, 2.4));
    require(near.result()->class_id == locked.class_id && near.result()->position.x == locked.position.x,
            "confirmed near coordinate/ID changed after locking");
    near.position({-1, 0}, 102);
    require(near.speed_cap(.6) == .4, "crossing 2m again restored the high speed cap");
}

void tie_requires_more_images_and_coordinates_are_weighted()
{
    VisionNearConfirmation near; near.begin(target(), 100, 100);
    double sum = 0, weights = 0;
    for (int i = 1; i <= 11; ++i) {
        const double x = i <= 5 ? 2.0 : 2.2;
        const Vec2 aircraft{i <= 5 ? .5 : 1.5, 0};
        const double w = 1 / std::pow(std::max(.5, x - aircraft.x), 2);
        sum += w * x; weights += w;
        const auto result = send(near, frame(i, i <= 5 ? 1 : 2, x), aircraft);
        require(result.has_value() == (i == 11), "tie was resolved arbitrarily before an extra frame");
        if (i == 10) require(near.tied() && near.count() == 10, "5/5 votes were not reported as tied");
    }
    require(near.result()->class_id == 2 && near.result()->samples == 11 &&
            std::abs(near.result()->position.x - sum / weights) < 1e-9,
            "near refinement differs from weighted mean of its own near samples");
}

void duplicates_misses_and_restarts()
{
    VisionNearConfirmation near; near.begin(target(), 100, 100);
    auto f = frame(1, 2); f.count = 2; f.targets[1] = f.targets[0];
    send(near, f);
    send(near, f);
    require(near.count() == 1, "duplicate image/box added multiple votes");
    f = frame(2, 2); f.count = 2; f.targets[1] = {5, 2.1, 0};
    send(near, f);
    require(near.count() == 0, "ambiguous duplicate boxes silently chose a category");
    send(near, frame(3, 2));
    f = frame(4, 2); f.count = 0; send(near, f);
    require(near.count() == 0 && near.votes()[2] == 0, "empty frame retained old near votes");
    for (int i = 5; i <= 8; ++i) send(near, frame(i, 2));
    f = frame(9, 3); f.seq = 2; f.epoch = 2; f.restarted = true;
    send(near, f);
    require(near.count() == 1 && near.votes()[2] == 0 && near.votes()[3] == 1,
            "writer restart combined two different confirmation windows");
    near.position({1, 0}, 102);
    require(near.count() == 0, "silent near stream did not clear partial votes");
}

void current_object_only_and_fresh_activation()
{
    VisionNearConfirmation near; near.begin(target(), 100.5, frame(10, 1).stamp);
    send(near, frame(10, 1));
    require(near.count() == 0, "far-confirmation frame counted again as a near frame");
    auto f = frame(11, 4, 2.5); // Equally close to current target and the neighbour.
    send(near, f, {1, 0}, {{3, 0}});
    require(near.count() == 0, "neighbouring object's ID leaked into current vote");
    f = frame(12, 4, 3); f.count = 2; f.targets[1] = {2, 2.1, 0};
    send(near, f, {1, 0}, {{3, 0}});
    require(near.count() == 1 && near.votes()[2] == 1 && near.votes()[4] == 0,
            "visible queued target voted for current object");
    send(near, frame(13, 2, 2.3), {.1, 0});
    require(near.count() == 0, "an observed point farther than 2m contributed a near measurement");
    near.begin(target(2, 4, 0), 102, 101);
    require(near.track_id() == 2 && near.count() == 0 && !near.near_entered(),
            "next target inherited previous votes or close-range state");
}

void missing_near_data_times_out_without_confirmation()
{
    VisionNearConfirmation near;
    near.begin(target(), 100, 100);
    near.position({1, 0}, 100);
    require(!near.timed_out(107.9) && near.timed_out(108), "bounded near confirmation timeout incorrect");
    auto f = frame(161, 3);
    require(!send(near, f) && !near.confirmed(), "late sample bypassed expired confirmation deadline");
}
}

int main()
{
    try {
        distant_ids_are_excluded_and_near_cap_is_latched();
        tie_requires_more_images_and_coordinates_are_weighted();
        duplicates_misses_and_restarts();
        current_object_only_and_fresh_activation();
        missing_near_data_times_out_without_confirmation();
        std::cout << "vision_near_confirmation_test: 2m gate, 10-frame vote, ties and refined coordinates passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
