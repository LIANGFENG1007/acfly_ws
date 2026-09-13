#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <ctime>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace vision_shm {
struct Target { std::int32_t id; double x; double y; };
inline double monotonic_now()
{
    timespec ts{}; if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) throw std::runtime_error("clock_gettime");
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}
class Writer {
public:
    explicit Writer(const std::string& path) : path_(path)
    {
        fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
        if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0) throw std::runtime_error("writer open/lock");
        if (::ftruncate(fd_, 512) != 0) throw std::runtime_error("writer resize");
        void* p = ::mmap(nullptr, 512, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (p == MAP_FAILED) throw std::runtime_error("writer mmap");
        bytes_ = static_cast<unsigned char*>(p);
        const auto old = load_seq(); seq_ = old & ~std::uint64_t{1};
        store_seq(seq_ + 1); __sync_synchronize();
    }
    ~Writer() { if (bytes_) ::munmap(bytes_, 512); if (fd_ >= 0) ::close(fd_); }
    Writer(const Writer&) = delete;
    bool publish(double stamp, const std::vector<Target>& targets)
    {
        if (targets.size() > 6 || !std::isfinite(stamp) || stamp <= last_stamp_) return false;
        std::array<unsigned char, 512> frame{}; std::int32_t count=static_cast<std::int32_t>(targets.size());
        std::memcpy(frame.data()+8,&stamp,8);std::memcpy(frame.data()+32,&count,4);
        for (std::size_t i=0;i<targets.size();++i) { auto o=40+40*i;std::memcpy(frame.data()+o,&targets[i].id,4);std::memcpy(frame.data()+o+16,&targets[i].x,8);std::memcpy(frame.data()+o+24,&targets[i].y,8); }
        seq_ += 2; store_seq(seq_-1); __sync_synchronize(); std::memcpy(bytes_+8,frame.data()+8,504); __sync_synchronize(); store_seq(seq_); last_stamp_=stamp; return true;
    }
private:
    std::uint64_t load_seq() const { std::uint64_t s{};std::memcpy(&s,bytes_,8);return s; }
    void store_seq(std::uint64_t s) { std::memcpy(bytes_,&s,8); }
    std::string path_; int fd_=-1; unsigned char* bytes_=nullptr; std::uint64_t seq_=0; double last_stamp_=0;
};
}
