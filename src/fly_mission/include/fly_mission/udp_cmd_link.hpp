#pragma once

// ============================================================================
//  udp_cmd_link.hpp  ── 跨机【命令】传输(UDP 单播)：给主控发启动指令等
//
//  ★与 udp_pose_link.hpp 的分工★：
//    · udp_pose_link.hpp = 【位姿流】高频(30Hz)、丢一帧无所谓、只取最新值
//    · 本文件            = 【命令】低频/单次、★丢了就误事★、要"收到一次即锁定"
//    两者性质不同，故分开：命令包更小(16B)、带 cmd 编号、发送端应【重复发】防丢。
//
//  ★为什么不用 ROS 话题★：见 udp_pose_link.hpp 顶部——两台机器同 ROS domain 时
//    Fast DDS 组播发现会打爆 WiFi、拖垮飞机 mavros 的串口实时性。命令虽然流量极小，
//    但只要还用 ROS 话题跨机，DDS 发现就仍然存在，问题照旧。所以命令也走 UDP。
//
//  ★可靠性策略★：UDP 不保证送达，而"启动指令"丢了就起飞不了。对策是
//    【发送端重复发】(比如 1s 发 1 次，一直发到你自己确认飞机响了第二声为止)，
//    接收端【收到一次即锁定】(重复的直接忽略)。这与原来 ROS 话题版的用法完全一致
//    (原来也是 "ros2 topic pub -r 1" 持续发)。
//
//  ★★★ 二进制布局(小端, 16B)——两端必须一致，改一边就要改另一边 ★★★
//    偏移  类型      含义
//    0     uint32    magic = 0x434D4431 ('CMD1')，防误收其它 UDP 包
//    4     uint32    cmd   命令编号(见下面 Cmd 枚举)
//    8     int32     arg   命令参数(当前所有命令都不用，填 0)
//    12    uint32    seq   发送端每包 +1(仅用于日志/诊断，接收端不依赖它)
//
//  ★端口★：与位姿通道【必须不同】(位姿用 9870)。命令用 9871。
//    同一个端口不能被两个程序同时 bind，也不该把两种协议混在一个端口上。
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
namespace udp_cmd {

constexpr uint32_t MAGIC        = 0x434D4431u;  // 'CMD1'
constexpr int      DEFAULT_PORT = 9871;         // ★与位姿通道(9870)分开★
constexpr size_t   PACKET_SIZE  = 16;

// 命令编号。★加新命令只能往后追加，不要改已有值★(改了两端就对不上)。
enum Cmd : uint32_t {
    CMD_NONE          = 0,
    CMD_START         = 1,   // 任务启动指令(原 /mission/start data==1)
    CMD_TAKEOFF_AGAIN = 2,   // 二次起飞触发(原 /mission/takeoff_again data==true)
};

// ============================================================================
//  读端(飞机)：非阻塞收命令，"收到一次即锁定"。
//  用法：
//      UdpCmdReceiver rx;                  // 构造即 bind
//      ...每拍...
//      rx.poll();                          // 收干本拍所有包
//      if (rx.got(udp_cmd::CMD_START)) { ...启动... }   // 锁定式：一直为 true
//
//  线程：非线程安全，只在单一线程(主控定时器)里用。
// ============================================================================
class UdpCmdReceiver
{
public:
    explicit UdpCmdReceiver(int port = DEFAULT_PORT) { open(port); }
    ~UdpCmdReceiver() { if (fd_ >= 0) ::close(fd_); }

    bool ok() const { return fd_ >= 0; }

    // 收干本拍所有排队的包。返回 true = 本拍收到过【新的(之前没锁定过的)】命令。
    //   重复发的同一命令不算"新"，返回 false，也不会刷日志。
    bool poll()
    {
        if (fd_ < 0) return false;
        bool fresh = false;
        uint8_t buf[PACKET_SIZE];
        for (int guard = 0; guard < 64; ++guard) {
            const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
            if (n < 0) break;                                       // EAGAIN：读干了
            if (n != static_cast<ssize_t>(PACKET_SIZE)) continue;    // 长度不符 → 丢弃

            uint32_t magic, cmd, seq;
            int32_t  arg;
            std::memcpy(&magic, buf + 0,  4);
            if (magic != MAGIC) continue;                            // 不是我们的包
            std::memcpy(&cmd,   buf + 4,  4);
            std::memcpy(&arg,   buf + 8,  4);
            std::memcpy(&seq,   buf + 12, 4);

            if (cmd >= MAX_CMD) continue;                            // 未知命令编号 → 丢弃

            ++rx_count_;
            last_seq_ = seq;
            last_arg_[cmd] = arg;
            if (!latched_[cmd]) {                                    // ★首次收到才算新★
                latched_[cmd] = true;
                last_new_cmd_ = cmd;
                fresh = true;
            }
        }
        return fresh;
    }

    // 某命令是否已收到过(锁定式：一旦收到就永远为 true)。
    bool got(uint32_t cmd) const
    {
        return (cmd < MAX_CMD) && latched_[cmd];
    }

    // 本拍新锁定的那个命令(配合 poll() 返回 true 时看)。
    uint32_t last_new_cmd() const { return last_new_cmd_; }
    // 某命令携带的参数(当前所有命令都不用参数)。
    int32_t  arg_of(uint32_t cmd) const { return (cmd < MAX_CMD) ? last_arg_[cmd] : 0; }
    // 累计收到的合法包数(含重复；用于确认"对方确实在发")。
    uint32_t rx_count() const { return rx_count_; }
    uint32_t last_seq() const { return last_seq_; }

    // 清掉某命令的锁定(需要重新触发时用，如二次起飞可能要多轮)。
    void reset(uint32_t cmd) { if (cmd < MAX_CMD) latched_[cmd] = false; }

private:
    static constexpr uint32_t MAX_CMD = 8;   // 命令编号上限(够用；加命令时同步调大)

    bool open(int port)
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return false;
        const int fl = ::fcntl(fd_, F_GETFL, 0);
        if (fl >= 0) ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);   // 非阻塞，绝不卡控制循环
        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port        = htons(static_cast<uint16_t>(port));
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        return true;
    }

    int      fd_ = -1;
    bool     latched_[MAX_CMD] = {};
    int32_t  last_arg_[MAX_CMD] = {};
    uint32_t last_new_cmd_ = CMD_NONE;
    uint32_t rx_count_ = 0;
    uint32_t last_seq_ = 0;
};

// ============================================================================
//  写端(发命令那台机器)：重复发同一命令防丢包。
// ============================================================================
class UdpCmdSender
{
public:
    UdpCmdSender(const char* dest_ip, int port = DEFAULT_PORT) { open(dest_ip, port); }
    ~UdpCmdSender() { if (fd_ >= 0) ::close(fd_); }

    bool ok() const { return fd_ >= 0; }

    bool send(uint32_t cmd, int32_t arg = 0)
    {
        if (fd_ < 0) return false;
        uint8_t buf[PACKET_SIZE];
        const uint32_t magic = MAGIC;
        ++seq_;
        std::memcpy(buf + 0,  &magic, 4);
        std::memcpy(buf + 4,  &cmd,   4);
        std::memcpy(buf + 8,  &arg,   4);
        std::memcpy(buf + 12, &seq_,  4);
        return ::sendto(fd_, buf, PACKET_SIZE, 0,
                        reinterpret_cast<sockaddr*>(&dest_), sizeof(dest_))
               == static_cast<ssize_t>(PACKET_SIZE);
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
            return false;
        }
        return true;
    }

    int         fd_ = -1;
    sockaddr_in dest_{};
    uint32_t    seq_ = 0;
};

}  // namespace udp_cmd
}  // namespace fly_mission
