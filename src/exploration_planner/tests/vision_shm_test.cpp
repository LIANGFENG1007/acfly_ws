#include "exploration_planner/vision_shm.hpp"
#include "vision_shm_test_writer.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>

using exploration::VisionShmReader;
namespace {
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

struct Fixture {
    std::string directory;
    std::string path;
    Fixture()
    {
        char pattern[] = "/tmp/acfly-vision-shm-XXXXXX";
        const char* dir = ::mkdtemp(pattern);
        if (!dir) throw std::runtime_error("mkdtemp");
        directory = dir;
        path = directory + "/target.bin";
    }
    ~Fixture() { std::filesystem::remove_all(directory); }
};

// Independent byte packer for malformed/legacy input and explicit frame times.
void raw_frame(const std::string& path, std::uint64_t seq, double stamp,
               std::int32_t count, double x = 2.0, std::int32_t id = 2)
{
    std::array<unsigned char, 512> bytes{};
    std::memcpy(bytes.data(), &seq, 8);
    std::memcpy(bytes.data() + 8, &stamp, 8);
    std::memcpy(bytes.data() + 32, &count, 4);
    for (int i = 0; i < 6; ++i) {
        const double y = -i - .25;
        std::memcpy(bytes.data() + 40 + 40 * i, &id, 4);
        std::memcpy(bytes.data() + 56 + 40 * i, &x, 8);
        std::memcpy(bytes.data() + 64 + 40 * i, &y, 8);
    }
    const int fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    require(fd >= 0, "raw input open");
    const auto written = ::pwrite(fd, bytes.data(), bytes.size(), 0);
    ::close(fd);
    require(written == 512, "raw input write");
}

void protocol_and_frame_identity()
{
    Fixture f;
    VisionShmReader reader(f.path, .5, .02);
    require(!reader.poll(100) && reader.status() == VisionShmReader::Status::Missing,
            "absent writer must not fabricate a frame");
    require(!std::filesystem::exists(f.path), "reader created SHM");
    raw_frame(f.path, 2, 99.9, 6);
    const auto first = reader.poll(100);
    require(first && first->count == 6 && first->seq == 2 && first->epoch == 1 && !first->restarted,
            "six-target legacy byte layout not decoded");
    for (int i = 0; i < 6; ++i)
        require(first->targets[i].id == 2 && first->targets[i].x == 2 && first->targets[i].y == -i - .25,
                "slot ordering, coordinates, or repeated category IDs were changed");
    require(!reader.poll(100) && reader.statistics().frames == 1, "same seq counted twice");
    raw_frame(f.path, 8, 99.95, 1, 2.01, 5);
    const auto changed_id = reader.poll(100);
    require(changed_id && changed_id->targets[0].id == 5 && !changed_id->restarted,
            "category change or skipped seq prevented receiving a new image");
    raw_frame(f.path, 10, 100, 0);
    const auto empty = reader.poll(100);
    require(empty && empty->count == 0 && reader.statistics().empty_frames == 1,
            "a real no-target image must be delivered as a new empty frame");
    require(!reader.poll(101) && reader.status() == VisionShmReader::Status::Stale,
            "stopped writer must become stale");
}

void malformed_input_and_restarts()
{
    Fixture f;
    VisionShmReader reader(f.path, .5, .02);
    const int fd = ::open(f.path.c_str(), O_CREAT | O_RDWR, 0600);
    require(fd >= 0, "create short file");
    require(!reader.poll(100) && reader.status() == VisionShmReader::Status::InvalidSize,
            "zero-byte SHM was mapped");
    require(::ftruncate(fd, 511) == 0, "resize short fixture");
    require(!reader.poll(100), "short SHM was accepted");
    ::close(fd);
    raw_frame(f.path, 1, 99.9, 1);
    require(!reader.poll(100) && reader.status() == VisionShmReader::Status::Writing,
            "odd/in-progress seq was consumed");
    raw_frame(f.path, 0, 99.9, 1);
    require(!reader.poll(100) && reader.status() == VisionShmReader::Status::Uninitialized,
            "uninitialized frame was consumed");
    for (int count : {-1, 7, 8, 100}) {
        raw_frame(f.path, 2, 99.9, count);
        require(!reader.poll(100), "invalid target count was silently clamped");
    }
    for (double x : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        raw_frame(f.path, 2, 99.9, 1, x);
        require(!reader.poll(100), "nonfinite coordinates accepted");
    }
    raw_frame(f.path, 2, 99.9, 1, 2, 9);
    require(!reader.poll(100), "ID outside the wire contract accepted");
    raw_frame(f.path, 2, 100.2, 1);
    require(!reader.poll(100) && reader.status() == VisionShmReader::Status::Future,
            "future/system-clock stamp accepted");
    raw_frame(f.path, 2, std::numeric_limits<double>::quiet_NaN(), 1);
    require(!reader.poll(100), "nonfinite stamp accepted");

    raw_frame(f.path, 20, 99.9, 1);
    require(reader.poll(100).has_value(), "valid data after errors did not recover");
    raw_frame(f.path, 22, 99.9, 1);
    require(!reader.poll(100), "republishing one source image with a new seq counted as a new frame");
    raw_frame(f.path, 24, 99.8, 1);
    require(!reader.poll(100) && reader.status() == VisionShmReader::Status::OutOfOrder,
            "old source image overwrote the accepted stream");
    raw_frame(f.path, 2, 99.95, 1);
    auto frame = reader.poll(100);
    require(frame && frame->restarted && frame->epoch == 2, "writer seq reset was not signalled");
    // A very fast restart can reuse exactly the last seq without an observed zero.
    raw_frame(f.path, 2, 99.96, 1);
    frame = reader.poll(100);
    require(frame && frame->restarted && frame->epoch == 3, "same-seq restart was lost");
    raw_frame(f.path + ".new", 2, 99.98, 1);
    std::filesystem::rename(f.path + ".new", f.path);
    frame = reader.poll(100);
    require(frame && frame->restarted && frame->epoch == 4, "reader remained mapped to an obsolete inode");
    const auto hash_before = std::filesystem::file_size(f.path);
    require(!reader.poll(100), "remapped frame consumed twice");
    require(hash_before == 512, "reader resized SHM");
}

void supplied_writer_interoperability()
{
    Fixture f;
    VisionShmReader reader(f.path, .5, .02);
    {
        vision_shm::Writer writer(f.path);
        const auto stamp = vision_shm::monotonic_now();
        require(writer.publish(stamp, {{1, 2.5, -3.25}, {1, 6.5, 7.25}}), "writer publish");
        const auto frame = reader.poll(vision_shm::monotonic_now());
        require(frame && frame->count == 2 && frame->targets[1].x == 6.5 &&
                frame->targets[0].y == -3.25, "developer's reference writer did not interoperate");
        require(!writer.publish(stamp, {}) && !reader.poll(vision_shm::monotonic_now()),
                "duplicate source image became an empty new observation");
    }
    {
        vision_shm::Writer restarted(f.path);
        require(!reader.poll(vision_shm::monotonic_now()), "writer startup created a fake observation");
        restarted.publish(vision_shm::monotonic_now(), {});
        const auto frame = reader.poll(vision_shm::monotonic_now());
        require(frame && frame->count == 0, "writer restart did not resume empty frames");
    }
}

void concurrent_writer_snapshots()
{
    Fixture f;
    raw_frame(f.path, 0, vision_shm::monotonic_now(), 0);
    const pid_t child = ::fork();
    require(child >= 0, "fork failed");
    if (child == 0) {
        try {
            vision_shm::Writer writer(f.path);
            for (int k = 1; k <= 600; ++k) {
                std::vector<vision_shm::Target> targets;
                for (int i = 0; i < 6; ++i) targets.push_back({1 + i, k * 10.0 + i, -(k * 10.0 + i)});
                writer.publish(vision_shm::monotonic_now(), targets);
                ::usleep(100);
            }
            ::_exit(0);
        } catch (...) { ::_exit(1); }
    }
    VisionShmReader reader(f.path, .5, .02);
    bool valid = true;
    int received = 0, status = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool finished = false;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto frame = reader.poll(vision_shm::monotonic_now());
        if (frame && frame->count == 6) {
            ++received;
            for (int i = 0; i < 6; ++i)
                valid = valid && frame->targets[i].id == 1 + i &&
                    frame->targets[i].x == frame->targets[0].x + i &&
                    frame->targets[i].y == -frame->targets[i].x;
        }
        if (::waitpid(child, &status, WNOHANG) == child) { finished = true; break; }
    }
    if (!finished) { ::kill(child, SIGKILL); ::waitpid(child, &status, 0); }
    require(finished && WIFEXITED(status) && WEXITSTATUS(status) == 0, "concurrent writer failed");
    require(valid && received > 0, "accepted mixed payload from concurrent image writes");
    std::cout << "concurrent coherent snapshots=" << received << '\n';
}
}  // namespace

int main()
{
    try {
        protocol_and_frame_identity();
        malformed_input_and_restarts();
        supplied_writer_interoperability();
        concurrent_writer_snapshots();
        std::cout << "vision_shm_test: all checks passed (private /tmp mailboxes only)\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
