#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace exploration {

// The existing uav_cv_out wire contract: 512 bytes, 8 physical slots of 40
// bytes. This exploration application consumes at most 6 targets per frame.
// All offsets are explicit; this decoded struct is never copied onto the wire.
struct VisionDetection {
    std::int32_t id = 0;
    double x = 0.0;
    double y = 0.0;
};

struct VisionFrame {
    std::uint64_t seq = 0;  // Whole-image sequence, independent of target IDs.
    double stamp = 0.0;     // Image capture time, CLOCK_MONOTONIC seconds.
    std::size_t count = 0;
    std::array<VisionDetection, 6> targets{};
    std::uint64_t epoch = 0;
    bool restarted = false;
};

class VisionShmReader {
public:
    static constexpr std::size_t kBytes = 512;
    static constexpr std::size_t kTargetOffset = 40;
    static constexpr std::size_t kTargetStride = 40;
    static constexpr std::size_t kMaxTargets = 6;

    enum class Status {
        Missing, Unavailable, InvalidSize, Writing, Uninitialized,
        InvalidData, Stale, Future, Duplicate, OutOfOrder, NewFrame
    };

    struct Statistics {
        std::uint64_t frames = 0;
        std::uint64_t empty_frames = 0;
        std::uint64_t restarts = 0;
        std::uint64_t invalid_polls = 0;
    };

    VisionShmReader(std::string path, double max_age_s, double future_tolerance_s);
    ~VisionShmReader();
    VisionShmReader(const VisionShmReader&) = delete;
    VisionShmReader& operator=(const VisionShmReader&) = delete;

    // Called from one thread. Never creates, resizes, clears or writes SHM.
    // nullopt means no accepted new frame; an accepted empty frame has count=0.
    std::optional<VisionFrame> poll(double monotonic_now);
    Status status() const { return status_; }
    const Statistics& statistics() const { return stats_; }
    double last_stamp() const { return last_stamp_; }
    const std::string& path() const { return path_; }
    static const char* status_name(Status status);

private:
    bool map_current_file();
    void close_mapping();
    std::optional<VisionFrame> reject(Status status);

    std::string path_;
    double max_age_s_;
    double future_tolerance_s_;
    int fd_ = -1;
    const std::uint64_t* words_ = nullptr;
    std::uint64_t device_ = 0, inode_ = 0;
    std::uint64_t last_seq_ = 0, epoch_ = 0;
    double last_stamp_ = 0.0;
    bool have_frame_ = false;
    bool remapped_ = false;
    bool saw_uninitialized_ = false;
    Status status_ = Status::Missing;
    Statistics stats_;
};

}  // namespace exploration
