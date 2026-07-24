#pragma once

// ============================================================================
//  shm_mailbox.hpp  ── 共享内存"最新值信箱"(seqlock 无锁)——视觉↔主控延迟优化
//
//  两个信箱(都是几百字节，读/写一次=一次小 memcpy，微秒级以下)：
//    信箱A /dev/shm/uav_cv_out   视觉→主控：检测结果。本文件是【读端】，写端=
//          opencv/finding_new1.2.1.py ShmMailboxWriter。★已是正式数据源★
//          (params::USE_SHM_CV=true，on_timer 每拍读、新帧喂 find_/lf_)。
//    信箱B /dev/shm/uav_pose_out 主控→视觉：飞机位姿。本文件是【写端】，
//          drone_controller 的 odom 回调雷达来一条写一条(20Hz)；视觉进程直读它
//          代替自己订阅 odom → rclpy 可整个删掉(第4步)。读端=finding_new1.2.1.py
//          ShmPoseReader。
//
//  seqlock 协议(每个信箱写端唯一，读端任意)：
//    写: seq++(奇=写入中) → 写 payload → seq++(偶=写完有效)。
//    读: 读 seq(奇则重试) → 拷 payload → 再读 seq，两次一致才采纳。
//    ★屏障★：C++ 写端(信箱B)在两次 seq 更新旁放全屏障(__sync_synchronize)——NX 是
//      ARM，store 可乱序，没屏障读端可能拿到"seq 已偶但 payload 还旧"的撕裂数据。
//      Python 写端(信箱A)没有可移植屏障，靠 seqlock 双检 + 下游多帧确认(FF_CONFIRM
//      _FRAMES/关联距离)兜底——撕裂窗口 ns 级、每帧 8ms，概率可忽略。
//
//  ★★信箱A 二进制布局(小端)——必须与 Python 写端完全一致，两边一起改★★
//    偏移   类型      含义
//    0      uint64    seq       (seqlock 序号，偶=有效)
//    8      double    stamp     (抓帧时刻, 视觉进程 time.monotonic() 秒 ← ★与本进程
//                                CLOCK_MONOTONIC 同源同机可直接相减算延迟★)
//    16     double    line_x    (寻线前视点 前后m; finding 程序恒0)
//    24     double    line_y    (寻线前视点 左右m; finding 程序恒0)
//    32     int32     n_targets (找图目标数 0..8)
//    36     int32     _pad
//    40     Target×8, 每个40字节: int32 id,color,shape,_pad; double x,y,保留
//    color: 0=red 1=green 2=yellow 3=blue   shape: 0=triangle 1=square 2=round 3=unknow
//
//  ★★信箱B 二进制布局(小端, 64B)——与 Python 读端 ShmPoseReader 一致，两边一起改★★
//    0      uint64    seq       (seqlock 序号，偶=有效；0=主控还没写过)
//    8      double    stamp     (写入时刻 CLOCK_MONOTONIC 秒 ≈ 雷达 odom 到达时刻)
//    16     double    x         (SLAM/camera_init 系, m)
//    24     double    y
//    32     double    z
//    40     double    yaw       (rad, SLAM 系)
//    48     16B       保留
// ============================================================================

#include <cstdint>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace fly_mission {
namespace shm {

constexpr const char* CV_SHM_PATH = "/dev/shm/uav_cv_out";
constexpr size_t      CV_SHM_SIZE = 512;
constexpr int         CV_MAX_TGT  = 8;

constexpr const char* POSE_SHM_PATH = "/dev/shm/uav_pose_out";
constexpr size_t      POSE_SHM_SIZE = 64;

// 读出的一帧视觉结果(已按布局解好)
struct CvTarget { int32_t id; int32_t color; int32_t shape; double x; double y; };
struct CvFrame {
    uint64_t seq = 0;          // 该帧的 seqlock 序号(判"有没有新帧"用：比上次大=新)
    double   stamp = 0.0;      // 抓帧时刻(视觉进程 monotonic 秒)
    double   line_x = 0.0, line_y = 0.0;
    int32_t  n_targets = 0;
    CvTarget targets[CV_MAX_TGT] = {};
};

class ShmMailboxReader
{
public:
    // 打开信箱(视觉进程负责创建)。open() 失败(视觉还没跑过)不是错误——ok() 为 false，
    //   read() 恒 false，调用方自然回退；之后每次 read() 会自动重试打开(视觉后启动也能接上)。
    ShmMailboxReader() { try_open(); }

    ~ShmMailboxReader()
    {
        if (mem_ != nullptr && mem_ != MAP_FAILED) munmap(mem_, CV_SHM_SIZE);
    }

    bool ok() const { return mem_ != nullptr && mem_ != MAP_FAILED; }

    // 读最新一帧到 out。返回 false = 信箱不存在/写入撞车重试超限/从未写入(seq=0)。
    //   ★不判新旧★：调用方拿 out.seq 与自己上次的比(变大=新帧)，拿 stamp 算数据年龄。
    bool read(CvFrame& out)
    {
        if (!ok() && !try_open()) return false;
        const volatile uint8_t* p = static_cast<const volatile uint8_t*>(mem_);
        for (int attempt = 0; attempt < 4; ++attempt) {   // 写端一次写仅~µs，4次必中
            uint64_t s1;
            std::memcpy(&s1, const_cast<const uint8_t*>(p), 8);
            if (s1 == 0) return false;                    // 还没写过
            if (s1 & 1) continue;                         // 奇=正在写，重试
            uint8_t buf[CV_SHM_SIZE];
            std::memcpy(buf, const_cast<const uint8_t*>(p), CV_SHM_SIZE);
            uint64_t s2;
            std::memcpy(&s2, buf, 8);
            // 两次 seq 一致(且开头快照也一致)才采纳——期间没被写过
            uint64_t s3;
            std::memcpy(&s3, const_cast<const uint8_t*>(p), 8);
            if (s2 != s1 || s3 != s1) continue;
            // 按布局解包
            out.seq = s1;
            std::memcpy(&out.stamp,  buf + 8,  8);
            std::memcpy(&out.line_x, buf + 16, 8);
            std::memcpy(&out.line_y, buf + 24, 8);
            std::memcpy(&out.n_targets, buf + 32, 4);
            if (out.n_targets < 0) out.n_targets = 0;
            if (out.n_targets > CV_MAX_TGT) out.n_targets = CV_MAX_TGT;
            for (int i = 0; i < out.n_targets; ++i) {
                const uint8_t* t = buf + 40 + 40 * i;
                std::memcpy(&out.targets[i].id,    t + 0,  4);
                std::memcpy(&out.targets[i].color, t + 4,  4);
                std::memcpy(&out.targets[i].shape, t + 8,  4);
                std::memcpy(&out.targets[i].x,     t + 16, 8);
                std::memcpy(&out.targets[i].y,     t + 24, 8);
            }
            return true;
        }
        return false;
    }

    // 本进程的 CLOCK_MONOTONIC 秒——与 Python time.monotonic() 同源(同一台机)，
    //   now_mono() - frame.stamp = 这帧结果的"年龄"(端到端延迟观测用)。
    static double now_mono()
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
    }

private:
    bool try_open()
    {
        const int fd = ::open(CV_SHM_PATH, O_RDONLY);
        if (fd < 0) return false;                         // 视觉还没创建信箱
        void* m = mmap(nullptr, CV_SHM_SIZE, PROT_READ, MAP_SHARED, fd, 0);
        ::close(fd);
        if (m == MAP_FAILED) return false;
        mem_ = m;
        return true;
    }

    void* mem_ = nullptr;
};

// ============================================================================
//  信箱B 写端：主控→视觉 飞机位姿(x/y/z/yaw)。
//  用法：DroneController 构造一个成员，odom 回调里每条 write() 一次(20Hz)。
//  写一次 = 64B memcpy + 两次屏障，亚微秒级，回调里顺手写零负担。
//  ★主控负责创建文件(O_CREAT)★——视觉读端只 open 不建，等主控先写(自动重试)。
// ============================================================================
class PoseMailboxWriter
{
public:
    PoseMailboxWriter()
    {
        const int fd = ::open(POSE_SHM_PATH, O_CREAT | O_RDWR, 0666);
        if (fd < 0) return;                               // /dev/shm 打不开(罕见)→ok()=false,write()静默跳过
        if (ftruncate(fd, static_cast<off_t>(POSE_SHM_SIZE)) != 0) { ::close(fd); return; }
        void* m = mmap(nullptr, POSE_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (m == MAP_FAILED) return;
        mem_ = m;
        std::memset(mem_, 0, POSE_SHM_SIZE);              // seq=0：读端知道"还没写过"
    }

    ~PoseMailboxWriter()
    {
        if (ok()) munmap(mem_, POSE_SHM_SIZE);
    }

    bool ok() const { return mem_ != nullptr && mem_ != MAP_FAILED; }

    // 写最新位姿(SLAM 系, yaw 弧度)。stamp 自动取 CLOCK_MONOTONIC(≈odom 到达时刻)。
    void write(double x, double y, double z, double yaw)
    {
        if (!ok()) return;
        uint8_t* p = static_cast<uint8_t*>(mem_);
        ++seq_;                                           // 奇：写入中
        std::memcpy(p, &seq_, 8);
        __sync_synchronize();                             // seq(奇) 先于 payload 落地
        const double vals[5] = { now_mono(), x, y, z, yaw };
        std::memcpy(p + 8, vals, sizeof(vals));
        __sync_synchronize();                             // payload 先于 seq(偶) 落地
        ++seq_;                                           // 偶：写完有效
        std::memcpy(p, &seq_, 8);
    }

    static double now_mono()
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
    }

private:
    void*    mem_ = nullptr;
    uint64_t seq_ = 0;
};

}  // namespace shm
}  // namespace fly_mission
