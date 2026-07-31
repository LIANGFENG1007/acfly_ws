// ============================================================================
//  udp_cmd_send.cpp  ── 给飞机主控发命令(启动指令 / 二次起飞)。无 ROS 依赖。
//
//  ★用途★：替代原来的
//      ros2 topic pub -r 1 /mission/start std_msgs/msg/Int32 "{data: 1}"
//    改走 UDP(原因见 udp_cmd_link.hpp / params::CMD_USE_UDP：跨机 ROS 话题会让
//    DDS 组播发现打爆 WiFi、拖垮飞机 mavros 的串口实时性)。
//
//  ── 编译(哪台机器要发命令就在哪台编；也可在飞机上编来自测) ──
//    需要 udp_cmd_link.hpp。若在飞机上：
//      cd ~/acfly_ws/src/fly_mission
//      g++ -std=c++17 -O2 tools/udp_cmd_send.cpp -Iinclude -o /tmp/udp_cmd_send
//    若拷到别的机器：把 udp_cmd_link.hpp 和本文件放同一目录，然后
//      g++ -std=c++17 -O2 udp_cmd_send.cpp -I. -o udp_cmd_send
//    (注意：单文件方式下 #include 路径要改成 "udp_cmd_link.hpp"，见下方 include)
//
//  ── 用法 ──
//    ./udp_cmd_send <飞机IP> start          # 发启动指令，默认每秒1次持续发
//    ./udp_cmd_send <飞机IP> start 1        # 只发 1 次(不推荐，UDP 可能丢)
//    ./udp_cmd_send <飞机IP> again          # 二次起飞触发
//    ./udp_cmd_send <飞机IP> start 0 9871   # 指定端口(默认 9871)
//
//  ★为什么默认持续发★：UDP 不保证送达，启动指令丢了就起飞不了。
//    飞机端"收到一次即锁定"，重复发不会重复触发，所以放心一直发。
//    看到飞机响【第二声】蜂鸣就说明收到了，这时 Ctrl+C 停掉即可。
// ============================================================================

#include "fly_mission/udp_cmd_link.hpp"   // 拷到别的机器时改成 "udp_cmd_link.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

using namespace fly_mission::udp_cmd;

static volatile bool g_run = true;
static void on_sigint(int) { g_run = false; }

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::printf(
            "用法: %s <飞机IP> <命令> [次数=0表示一直发] [端口=%d]\n"
            "  命令: start  = 任务启动指令(BEEP①之后等的那个)\n"
            "        again  = 二次起飞触发\n"
            "例: %s 192.168.12.1 start        # 每秒发一次，直到 Ctrl+C\n"
            "    %s 192.168.12.1 start 1      # 只发一次\n",
            argv[0], DEFAULT_PORT, argv[0], argv[0]);
        return 1;
    }

    const char* ip   = argv[1];
    const char* name = argv[2];
    const int   times = (argc > 3) ? std::atoi(argv[3]) : 0;    // 0 = 无限
    const int   port  = (argc > 4) ? std::atoi(argv[4]) : DEFAULT_PORT;

    uint32_t cmd;
    if      (std::strcmp(name, "start") == 0) cmd = CMD_START;
    else if (std::strcmp(name, "again") == 0) cmd = CMD_TAKEOFF_AGAIN;
    else {
        std::printf("未知命令 \"%s\"(只支持 start / again)\n", name);
        return 1;
    }

    UdpCmdSender tx(ip, port);
    if (!tx.ok()) {
        std::printf("★socket 打开失败★(\"%s\" 是合法 IPv4 吗?)\n", ip);
        return 1;
    }

    std::signal(SIGINT, on_sigint);
    std::printf("向 %s:%d 发命令 \"%s\"(cmd=%u)%s\n",
                ip, port, name, cmd,
                times > 0 ? "" : " —— 每秒 1 次持续发，看到飞机响第二声后 Ctrl+C");

    int sent = 0;
    while (g_run && (times <= 0 || sent < times)) {
        if (tx.send(cmd)) {
            ++sent;
            std::printf("已发第 %d 次 (seq=%u)\n", sent, tx.seq());
        } else {
            std::printf("发送失败(网络不通? 目标不可达?)\n");
        }
        std::fflush(stdout);
        if (times > 0 && sent >= times) break;
        timespec ts{1, 0};                    // 1 秒一次
        nanosleep(&ts, nullptr);
    }
    std::printf("共发送 %d 次，退出。\n", sent);
    return 0;
}
