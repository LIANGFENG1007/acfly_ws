#pragma once

// ============================================================================
//  udp_pose_link.hpp  ── 跨机位姿传输(UDP 单播)：绕开 DDS，只传 x/y/z/yaw
//
//  ★为什么不用 ROS 话题跨机★(2026-07-29)：
//    Fast DDS 默认靠【组播】做发现，且是"全连接"——两台机器一旦互相看见，每个节点
//    都要和对面每个节点建连并周期心跳(PDP/EDP 全量交换)，连接数是【乘积关系】。
//    本机 mavros 有 50+ 插件节点，雷达机一开，发现流量在 WiFi 上炸开；而 WiFi 的
//    组播是按【最低速率】发且不重传，于是挤爆链路 + 抢 CPU → mavros 串口收发失去
//    实时性 → setpoint 发不出去。实测现象就是"雷达机一启动 mavros 就不传数据了"。
//    UDP 单播没有发现机制、没有心跳、点对点直达，彻底没有这个问题。
//
//  ★角色★：本文件是【读端(飞机)】。发送端在雷达机上跑一个小节点(订 ROS 话题 →
//    发 UDP)，见本文件末尾注释里的发送端代码。
//
//  ★为什么不复用 shm_mailbox★：shm 只能同机进程间用(共享内存)；跨机必须走网络。
//    但设计风格刻意保持一致(定长小结构 + memcpy + 无锁 + 读端容错)，便于对照。
//
//  ★★★ 二进制布局(小端, 48B)——两端必须完全一致，改一边就要改另一边 ★★★
//    偏移  类型      含义
//    0     uint32    magic  = 0x50534531 ('PSE1')，防误收其它 UDP 包
//    4     uint32    seq    序号(发送端每包 +1)：判丢包/判新旧
//    8     double    stamp  发送时刻(发送端 CLOCK_MONOTONIC 秒)
//    16    double    x      (SLAM 系, m)
//    24    double    y
//    32    double    z
//    40    double    yaw    (rad)
//
//  ★时钟注意★：stamp 是【发送端】的 monotonic，与本机 monotonic【不同源】(两台
//    机器各自从开机计时)，所以 now_mono()-stamp 【不能】当延迟用，只能用来判
//    "发送端自己有没有卡住"(相邻包的 stamp 差)。判数据新鲜度请用本机收包时刻，
//    即 age_sec()——它记录的是本机最后一次成功收包到现在多久。
// ============================================================================

#include <cstdint>
#include <cstring>
#include <ctime>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace fly_mission {
namespace udp_pose {

constexpr uint32_t MAGIC        = 0x50534531u;  // 'PSE1'
constexpr int      DEFAULT_PORT = 9870;         // 两端一致即可(避开常用端口)
constexpr size_t   PACKET_SIZE  = 48;

// 解好的一帧位姿
struct Pose {
    uint32_t seq   = 0;
    double   stamp = 0.0;   // 发送端 monotonic 秒(★与本机不同源，别用来算延迟★)
    double   x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
};

// ============================================================================
//  读端(飞机)：非阻塞 UDP 接收。用法——
//      UdpPoseReceiver rx;                    // 构造即 bind，失败也不抛
//      ...每拍...
//      if (rx.poll()) {                       // 收干本拍所有包，只留最新一帧
//          const Pose& p = rx.latest();
//          ...用 p.x / p.y / p.z / p.yaw...
//      }
//      if (rx.age_sec() > 0.5) { /* 数据太旧：发送端挂了/网断了 → 自己兜底 */ }
//
//  线程：非线程安全。在单一线程(如主控 20/50Hz 定时器)里用即可。
// ============================================================================
class UdpPoseReceiver
{
public:
    explicit UdpPoseReceiver(int port = DEFAULT_PORT) { open(port); }

    ~UdpPoseReceiver() { if (fd_ >= 0) ::close(fd_); }

    bool ok() const { return fd_ >= 0; }

    // 收本拍所有排队的包，只保留【最新】一帧(UDP 可能乱序/堆积，旧的没用)。
    //   返回 true = 本拍收到过至少一个有效包。不阻塞。
    bool poll()
    {
        if (fd_ < 0) return false;
        bool got = false;
        uint8_t buf[PACKET_SIZE];
        // 循环读干缓冲：网络突发时一拍可能积了好几包，只认最后一个
        for (int guard = 0; guard < 64; ++guard) {
            const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
            if (n < 0) break;                       // EAGAIN：没有更多包了
            if (n != static_cast<ssize_t>(PACKET_SIZE)) continue;   // 长度不符 → 丢弃

            uint32_t magic;
            std::memcpy(&magic, buf + 0, 4);
            if (magic != MAGIC) continue;            // 不是我们的包(误收别的 UDP)

            Pose p;
            std::memcpy(&p.seq,   buf + 4,  4);
            std::memcpy(&p.stamp, buf + 8,  8);
            std::memcpy(&p.x,     buf + 16, 8);
            std::memcpy(&p.y,     buf + 24, 8);
            std::memcpy(&p.z,     buf + 32, 8);
            std::memcpy(&p.yaw,   buf + 40, 8);

            // 丢包统计(仅诊断)：seq 应该连续 +1
            if (have_ && p.seq > latest_.seq + 1) lost_ += (p.seq - latest_.seq - 1);
            latest_ = p;
            have_   = true;
            got     = true;
            last_recv_ = now_mono();                 // ★本机时钟★，用于 age_sec()
        }
        return got;
    }

    bool         have() const { return have_; }
    const Pose&  latest() const { return latest_; }
    uint32_t     lost_count() const { return lost_; }

    // 距本机最后一次成功收包多久(秒)。从未收到过 → 返回很大值。
    //   ★判数据新鲜度用这个★(不要用 latest().stamp，那是对端时钟)。
    double age_sec() const
    {
        if (!have_) return 1e9;
        return now_mono() - last_recv_;
    }

    static double now_mono()
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
    }

private:
    bool open(int port)
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return false;

        // 非阻塞：poll() 里靠 EAGAIN 判"读干了"，绝不卡住控制循环
        const int fl = ::fcntl(fd_, F_GETFL, 0);
        if (fl >= 0) ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);

        // 允许重启后立刻重新 bind(避免 TIME_WAIT 占着端口)
        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);    // 收任意网卡进来的包
        addr.sin_port        = htons(static_cast<uint16_t>(port));
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        return true;
    }

    int      fd_ = -1;
    bool     have_ = false;
    Pose     latest_;
    double   last_recv_ = 0.0;
    uint32_t lost_ = 0;
};

// ============================================================================
//  写端(雷达机)：把一帧位姿打包发出去。见 udp_pose_sender.cpp 的用法。
// ============================================================================
class UdpPoseSender
{
public:
    // dest_ip：接收方(飞机)IP。port 两端一致。
    UdpPoseSender(const char* dest_ip, int port = DEFAULT_PORT) { open(dest_ip, port); }

    ~UdpPoseSender() { if (fd_ >= 0) ::close(fd_); }

    bool ok() const { return fd_ >= 0; }

    // 发一帧。返回 false = socket 没开好/发送失败(不重试，下一帧照发即可)。
    bool send(double x, double y, double z, double yaw)
    {
        if (fd_ < 0) return false;
        uint8_t buf[PACKET_SIZE];
        const uint32_t magic = MAGIC;
        const double   stamp = UdpPoseReceiver::now_mono();
        ++seq_;
        std::memcpy(buf + 0,  &magic, 4);
        std::memcpy(buf + 4,  &seq_,  4);
        std::memcpy(buf + 8,  &stamp, 8);
        std::memcpy(buf + 16, &x,     8);
        std::memcpy(buf + 24, &y,     8);
        std::memcpy(buf + 32, &z,     8);
        std::memcpy(buf + 40, &yaw,   8);
        const ssize_t n = ::sendto(fd_, buf, PACKET_SIZE, 0,
                                   reinterpret_cast<sockaddr*>(&dest_), sizeof(dest_));
        return n == static_cast<ssize_t>(PACKET_SIZE);
    }

    uint32_t seq() const { return seq_; }

private:
    bool open(const char* dest_ip, int port)
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return false;
        dest_ = sockaddr_in{};
        dest_.sin_family = AF_INET;
        dest_.sin_port   = htons(static_cast<uint16_t>(port));
        if (::inet_pton(AF_INET, dest_ip, &dest_.sin_addr) != 1) {
            ::close(fd_);
            fd_ = -1;
            return false;                            // IP 字符串非法
        }
        return true;
    }

    int         fd_ = -1;
    sockaddr_in dest_{};
    uint32_t    seq_ = 0;
};

}  // namespace udp_pose
}  // namespace fly_mission
