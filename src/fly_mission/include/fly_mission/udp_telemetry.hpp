#pragma once

// ============================================================================
//  udp_telemetry.hpp  ── 遥测上报(UDP 单播)：飞机坐标 + 小车原始坐标 + 飞行状态
//
//  ★方向★：本机(飞机) → 地面/监控端。★只发不收★，发送端完全不关心对方在不在，
//    UDP 无连接：对方没开、掉线、重启，本机都照发，不阻塞、不报错、不影响飞行。
//
//  ★与另两条 UDP 通道的分工(端口都不同，互不干扰)★：
//      9870  udp_pose_link.hpp  小车位姿   小车机 → 飞机  (收)
//      9871  udp_cmd_link.hpp   启动指令   任意机 → 飞机  (收)
//    ★9872  本文件              遥测上报   飞机 → 监控端  (发)★
//
//  ★为什么不用 ROS 话题★：跨机 ROS 话题会让 Fast DDS 组播发现打爆 WiFi、拖垮
//    mavros 串口实时性(详见 params::CAR_USE_UDP 的注释)。遥测虽然是"出方向"，
//    但只要建了跨机 ROS 连接，DDS 发现就会双向进行，问题照旧。所以走 UDP。
//
//  ★★★ 二进制布局(小端, 56B)——两端必须完全一致，改一边就要改另一边 ★★★
//    偏移  类型     字段        含义
//    0     uint32   magic     = 0x544C4D31 ('TLM1')，防误收其它 UDP 包
//    4     uint32   seq       序号，每包 +1(判丢包/判新旧)
//    8     double   stamp     发送时刻(本机 CLOCK_MONOTONIC 秒)
//    16    double   drone_x   ★飞机★坐标(飞机 SLAM/camera_init 系, m)
//    24    double   drone_y
//    32    double   drone_z
//    40    double   car_x     ★小车原始坐标★(小车自己那套 SLAM 的 B 系, m)
//    48    double   car_y     ——★原始值，未加 CAR_ORIGIN 平移标定★
//    56    int32    status    飞行状态：1=起飞 2=追踪 3=投掷 4=降落 0=其它/未开始
//    60    int32    flags     位标志：bit0 小车数据有效 bit1 视觉数据有效
//    ★总长 64 字节★(已用 #pragma pack(1) + static_assert 锁死，无填充)
//
//  ★car_x/car_y 是"原始"的含义★：直接是小车雷达报的 B 系坐标，没有经过主控的
//    CAR_ORIGIN_X/Y 平移换算。监控端若想画在飞机同一张图上，需要自己加这个偏移
//    (值见 params.hpp 的 CAR_ORIGIN_X / CAR_ORIGIN_Y)。这样设计是因为"原始值"
//    对排查标定问题更有用——标定错了原始值仍然对。
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
namespace udp_tlm {

constexpr uint32_t MAGIC        = 0x544C4D31u;  // 'TLM1'
constexpr int      DEFAULT_PORT = 9872;         // ★与 9870/9871 分开★

// 飞行状态编码(与需求一致；0 留给"其它/未开始")
enum Status : int32_t {
    ST_NONE   = 0,   // 其它状态(检测/待机/已结束…)
    ST_TAKEOFF = 1,  // 起飞(含起飞后悬停)
    ST_TRACK   = 2,  // 追踪小车
    ST_DROP    = 3,  // 视觉锁定投掷
    ST_LAND    = 4,  // 降落(含投掷后返航→降落)
};

// flags 位定义
constexpr int32_t F_CAR_OK   = 1 << 0;   // 小车位姿数据新鲜(否则 car_x/y 是旧值/0)
constexpr int32_t F_CV_OK    = 1 << 1;   // 视觉 dx/dy 数据新鲜
// ★bit2 飞机位姿有效★：无位姿时 drone_x/y/z 会是 0 —— 监控端必须靠这个位区分
//   "飞机真的在原点" 还是 "SLAM 挂了/没启动，坐标无意义"。没有它会把 0 当真实坐标画。
constexpr int32_t F_POSE_OK  = 1 << 2;

#pragma pack(push, 1)
struct Packet {
    uint32_t magic;
    uint32_t seq;
    double   stamp;
    double   drone_x, drone_y, drone_z;
    double   car_x, car_y;        // ★原始 B 系★
    int32_t  status;
    int32_t  flags;
};
#pragma pack(pop)

static_assert(sizeof(Packet) == 64, "遥测包布局变了，监控端会读到错位数据");

// ============================================================================
//  发送端(飞机)。用法：
//      UdpTelemetrySender tx("192.168.12.187", 9872);
//      ...每拍或限频...
//      tx.send(px,py,pz, cx,cy, status, flags);
//
//  ★绝不阻塞控制循环★：非阻塞 socket + 不重试。发送失败只返回 false，
//    由调用方节流打日志——遥测丢几包无所谓，绝不能拖慢 50Hz 主循环。
// ============================================================================
class UdpTelemetrySender
{
public:
    UdpTelemetrySender(const char* dest_ip, int port = DEFAULT_PORT)
    { open(dest_ip, port); }

    ~UdpTelemetrySender() { if (fd_ >= 0) ::close(fd_); }

    bool ok() const { return fd_ >= 0; }

    bool send(double px, double py, double pz,
              double cx, double cy,
              int32_t status, int32_t flags)
    {
        if (fd_ < 0) return false;
        Packet p{};
        p.magic  = MAGIC;
        p.seq    = ++seq_;
        p.stamp  = now_mono();
        p.drone_x = px; p.drone_y = py; p.drone_z = pz;
        p.car_x   = cx; p.car_y   = cy;
        p.status  = status;
        p.flags   = flags;
        const ssize_t n = ::sendto(fd_, &p, sizeof(p), 0,
                                   reinterpret_cast<sockaddr*>(&dest_), sizeof(dest_));
        if (n == static_cast<ssize_t>(sizeof(p))) { ++sent_; return true; }
        ++failed_;
        return false;
    }

    uint32_t seq()    const { return seq_; }
    uint32_t sent()   const { return sent_; }
    uint32_t failed() const { return failed_; }

    static double now_mono()
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
    }

private:
    bool open(const char* dest_ip, int port)
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return false;
        // 非阻塞：内核发送缓冲满时立即返回 EAGAIN，绝不挂住控制循环
        const int fl = ::fcntl(fd_, F_GETFL, 0);
        if (fl >= 0) ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);
        dest_ = sockaddr_in{};
        dest_.sin_family = AF_INET;
        dest_.sin_port   = htons(static_cast<uint16_t>(port));
        if (::inet_pton(AF_INET, dest_ip, &dest_.sin_addr) != 1) {
            ::close(fd_); fd_ = -1;
            return false;                       // IP 字符串非法
        }
        return true;
    }

    int         fd_ = -1;
    sockaddr_in dest_{};
    uint32_t    seq_ = 0, sent_ = 0, failed_ = 0;
};

}  // namespace udp_tlm
}  // namespace fly_mission
