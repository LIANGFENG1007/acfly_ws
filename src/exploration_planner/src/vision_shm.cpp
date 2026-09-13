#include "exploration_planner/vision_shm.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "The uav_cv_out wire format requires little-endian decoding."
#endif

namespace exploration {
namespace {
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
static_assert(sizeof(std::int32_t) == 4 && sizeof(std::uint64_t) == 8);
static_assert(__atomic_always_lock_free(8, nullptr), "Native 64-bit atomics required");

template<typename T>
T read_at(const std::array<std::uint64_t, 64>& data, std::size_t offset)
{
    T value{};
    std::memcpy(&value, reinterpret_cast<const unsigned char*>(data.data()) + offset, sizeof(T));
    return value;
}
}  // namespace

VisionShmReader::VisionShmReader(std::string path, double max_age_s, double future_tolerance_s)
    : path_(std::move(path)), max_age_s_(max_age_s), future_tolerance_s_(future_tolerance_s)
{
    if (path_.empty() || !std::isfinite(max_age_s_) || max_age_s_ <= 0.0 ||
        !std::isfinite(future_tolerance_s_) || future_tolerance_s_ < 0.0)
        throw std::invalid_argument("Vision SHM requires a path, positive max age and nonnegative future tolerance");
}

VisionShmReader::~VisionShmReader() { close_mapping(); }

void VisionShmReader::close_mapping()
{
    if (words_) { ::munmap(const_cast<std::uint64_t*>(words_), kBytes); words_ = nullptr; }
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool VisionShmReader::map_current_file()
{
    struct stat named{};
    if (::stat(path_.c_str(), &named) != 0) {
        status_ = errno == ENOENT ? Status::Missing : Status::Unavailable;
        close_mapping();
        return false;
    }
    if (!S_ISREG(named.st_mode) || named.st_size != static_cast<off_t>(kBytes)) {
        status_ = Status::InvalidSize;
        ++stats_.invalid_polls;
        close_mapping();
        return false;
    }
    if (words_ && device_ == static_cast<std::uint64_t>(named.st_dev) &&
        inode_ == static_cast<std::uint64_t>(named.st_ino)) return true;

    close_mapping();
    fd_ = ::open(path_.c_str(), O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    if (fd_ < 0) { status_ = Status::Unavailable; return false; }
    struct stat opened{};
    if (::fstat(fd_, &opened) != 0 || !S_ISREG(opened.st_mode) ||
        opened.st_size != static_cast<off_t>(kBytes)) {
        status_ = Status::InvalidSize;
        ++stats_.invalid_polls;
        close_mapping();
        return false;
    }
    void* memory = ::mmap(nullptr, kBytes, PROT_READ, MAP_SHARED, fd_, 0);
    if (memory == MAP_FAILED) {
        status_ = Status::Unavailable;
        close_mapping();
        return false;
    }
    words_ = static_cast<const std::uint64_t*>(memory);
    device_ = static_cast<std::uint64_t>(opened.st_dev);
    inode_ = static_cast<std::uint64_t>(opened.st_ino);
    remapped_ = have_frame_;
    return true;
}

std::optional<VisionFrame> VisionShmReader::reject(Status status)
{
    status_ = status;
    ++stats_.invalid_polls;
    return std::nullopt;
}

std::optional<VisionFrame> VisionShmReader::poll(double now)
{
    if (!std::isfinite(now)) return reject(Status::InvalidData);
    if (!map_current_file()) return std::nullopt;
    // The producer must not truncate a live mapping. Size is checked on every
    // poll; replacement by rename/recreation is detected by device/inode.
    std::array<std::uint64_t, 64> snapshot{};
    bool coherent = false;
    for (int retry = 0; retry < 4; ++retry) {
        const auto first = __atomic_load_n(words_, __ATOMIC_ACQUIRE);
        if (first == 0) {
            saw_uninitialized_ = have_frame_;
            status_ = Status::Uninitialized;
            return std::nullopt;
        }
        if (first & 1) continue;
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        snapshot[0] = first;
        for (std::size_t i = 1; i < snapshot.size(); ++i)
            snapshot[i] = __atomic_load_n(words_ + i, __ATOMIC_RELAXED);
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        if (__atomic_load_n(words_, __ATOMIC_ACQUIRE) == first) { coherent = true; break; }
    }
    if (!coherent) { status_ = Status::Writing; return std::nullopt; }

    VisionFrame frame;
    frame.seq = snapshot[0];
    frame.stamp = read_at<double>(snapshot, 8);
    const auto count = read_at<std::int32_t>(snapshot, 32);
    if (!std::isfinite(frame.stamp) || frame.stamp <= 0.0 || count < 0 || count > 6)
        return reject(Status::InvalidData);
    if (now - frame.stamp > max_age_s_) return reject(Status::Stale);
    if (frame.stamp - now > future_tolerance_s_) return reject(Status::Future);
    frame.count = static_cast<std::size_t>(count);
    for (std::size_t i = 0; i < frame.count; ++i) {
        const auto offset = kTargetOffset + kTargetStride * i;
        auto& target = frame.targets[i];
        target.id = read_at<std::int32_t>(snapshot, offset);
        target.x = read_at<double>(snapshot, offset + 16);
        target.y = read_at<double>(snapshot, offset + 24);
        if (target.id < 1 || target.id > 6 || !std::isfinite(target.x) || !std::isfinite(target.y))
            return reject(Status::InvalidData);
    }

    // Frame identity is (writer epoch, seq, source time), never target ID.
    // Source time must advance even if a timer republishes an old payload with
    // a new seq. A reset/recreated writer is explicitly reported downstream.
    if (have_frame_) {
        if (frame.stamp <= last_stamp_) {
            if (frame.seq == last_seq_ && frame.stamp == last_stamp_) {
                status_ = Status::Duplicate;
                return std::nullopt;
            }
            return reject(Status::OutOfOrder);
        }
        frame.restarted = remapped_ || saw_uninitialized_ || frame.seq <= last_seq_;
    }
    if (!have_frame_ || frame.restarted) ++epoch_;
    if (frame.restarted) ++stats_.restarts;
    frame.epoch = epoch_;
    have_frame_ = true;
    remapped_ = saw_uninitialized_ = false;
    last_seq_ = frame.seq;
    last_stamp_ = frame.stamp;
    ++stats_.frames;
    if (frame.count == 0) ++stats_.empty_frames;
    status_ = Status::NewFrame;
    return frame;
}

const char* VisionShmReader::status_name(Status status)
{
    switch (status) {
    case Status::Missing: return "missing";
    case Status::Unavailable: return "unavailable";
    case Status::InvalidSize: return "invalid_size";
    case Status::Writing: return "writing";
    case Status::Uninitialized: return "uninitialized";
    case Status::InvalidData: return "invalid_data";
    case Status::Stale: return "stale";
    case Status::Future: return "future_stamp";
    case Status::Duplicate: return "no_new_frame";
    case Status::OutOfOrder: return "out_of_order";
    case Status::NewFrame: return "new_frame";
    }
    return "unknown";
}

}  // namespace exploration
