#include "exploration_planner/vision_target_queue.hpp"

#include <iostream>
#include <stdexcept>

using namespace exploration;
namespace {
void require(bool ok, const char* text) { if (!ok) throw std::runtime_error(text); }
VisionCandidate candidate(std::uint64_t id, double x, double y, double stamp = 10)
{
    return {id, {x, y}, 10, true, stamp, stamp};
}
VisionFrame observation(double x, double y, double stamp)
{
    VisionFrame frame;
    frame.seq = static_cast<std::uint64_t>(stamp * 100) * 2;
    frame.stamp = stamp; frame.epoch = 1; frame.count = 1;
    frame.targets[0] = {6, x, y};
    return frame;
}

void fifo_nonpreemption_and_same_frame_order()
{
    VisionTargetQueue queue;
    queue.enqueue({candidate(3, 8, 0), candidate(2, 5, 0), candidate(1, 2, 0)}, {0, 0});
    require(queue.pending().size() == 3 && queue.pending()[0].track_id == 1 &&
            queue.pending()[1].track_id == 2, "same-frame targets must start nearest first");
    require(queue.activate_next(11) && queue.active()->track_id == 1, "first activation");
    queue.enqueue({candidate(4, 0, 0, 12)}, {0, 0});
    require(!queue.activate_next(12) && queue.active()->track_id == 1, "new closer target preempted active target");
    require(queue.pending().back().track_id == 4, "later confirmation cut into the FIFO queue");
    queue.enqueue({candidate(1, 2, 0), candidate(9, 2.1, 0)}, {0, 0});
    require(queue.pending().size() == 3 && queue.statistics().enqueued == 4,
            "repeated spatial object produced duplicate visits");
    require(!queue.complete_active(1, 13), "unrefined target was marked completed");
    require(queue.refine_active({1, {2.1, 0}, 4, 10, 13}), "valid near confirmation rejected");
    require(!queue.refine_active({1, {2.2, 0}, 5, 11, 14}), "locked class or position changed later");
    require(queue.complete_active(1, 15) && queue.activate_next(15) && queue.active()->track_id == 2,
            "queue did not directly hand off to its next item");
    require(queue.refine_active({2, {5, 0}, 4, 10, 16}) && queue.complete_active(2, 17),
            "same class at another position was globally suppressed");
}

void final_position_blacklist_and_input_filter()
{
    VisionTargetQueue queue;
    queue.enqueue({candidate(1, 2, 0), candidate(2, 3.05, 0), candidate(3, 5, 0)}, {});
    queue.activate_next(11);
    require(queue.refine_active({1, {2.5, 0}, 3, 10, 12}), "refined point rejected");
    require(!queue.blacklisted({2.5, 0}), "ID confirmation blacklisted target before visit completion");
    require(!queue.complete_active(99, 13), "another track completed active target");
    require(queue.complete_active(1, 13), "completion failed");
    require(queue.visited().size() == 1 && queue.visited()[0].position.x == 2.5,
            "blacklist used old coarse position");
    require(queue.blacklisted({3.09, 0}) && !queue.blacklisted({1.8, 0}),
            "0.6m blacklist not centred on refined target");
    require(queue.pending().size() == 1 && queue.pending()[0].track_id == 3,
            "queued duplicate inside final blacklist was retained");
    auto raw = observation(2.5, 0, 14);
    raw.count = 2; raw.targets[1] = {3, 5, 0};
    const auto filtered = queue.filter_blacklisted(raw);
    require(filtered.count == 1 && filtered.targets[0].x == 5 && filtered.seq == raw.seq,
            "input filtering discarded independent targets or changed image identity");
    queue.enqueue({candidate(20, 2.6, 0, 14)}, {});
    require(queue.pending().size() == 1, "blacklisted point re-enqueued with a new track/class");
    queue.reset();
    require(queue.pending().empty() && !queue.active() && queue.visited().empty() &&
            !queue.blacklisted({2.5, 0}), "new mission failed to reset blacklist");
}

void failures_wait_for_cooldown_and_fresh_observation()
{
    VisionTargetQueue queue;
    queue.enqueue({candidate(1, 2, 0), candidate(2, 4, 0)}, {});
    queue.activate_next(11);
    require(queue.defer_active(1, 12) && queue.visited().empty(), "deferred failure marked visited");
    queue.observe(observation(2, 0, 16), 16);
    require(queue.activate_next(17) && queue.active()->track_id == 2 && queue.deferred().size() == 1,
            "cooldown reused an observation from before the retry window");
    queue.refine_active({2, {4, 0}, 2, 10, 18}); queue.complete_active(2, 19);
    require(!queue.activate_next(19), "unobserved deferred target prevented exploration resuming");
    queue.observe(observation(2, 0, 19.1), 19.1);
    require(queue.activate_next(19.2) && queue.active()->track_id == 1 &&
            queue.active()->class_id == 0 && !queue.active()->refined,
            "fresh deferred target failed to rejoin normal near confirmation");
}

void capacity_and_unconfirmed_input()
{
    VisionQueueConfig cfg; cfg.max_pending = 1;
    VisionTargetQueue queue(cfg);
    auto unconfirmed = candidate(10, 1, 0); unconfirmed.confirmed = false;
    queue.enqueue({unconfirmed, candidate(1, 2, 0), candidate(2, 4, 0)}, {});
    require(queue.pending().size() == 1 && queue.statistics().capacity_drops == 1,
            "capacity ignored or tentative point enqueued");
    queue.activate_next(11);
    queue.refine_active({1, {2, 0}, 1, 10, 12}); queue.complete_active(1, 13);
    queue.enqueue({candidate(2, 4, 0)}, {});
    require(queue.pending().size() == 1 && queue.pending()[0].track_id == 2,
            "capacity-dropped confirmed point could never retry admission");
}
}

int main()
{
    try {
        fifo_nonpreemption_and_same_frame_order();
        final_position_blacklist_and_input_filter();
        failures_wait_for_cooldown_and_fresh_observation();
        capacity_and_unconfirmed_input();
        std::cout << "vision_target_queue_test: FIFO, nonpreemption, spatial blacklist and deferred retry passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
