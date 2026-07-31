// ============================================================================
//  udp_pose_monitor.cpp  ── 【在飞机上跑】验证 UDP 位姿收得到、频率/丢包正常
//
//  用途：接进主控【之前】先用它确认链路通。无 ROS 依赖，一条 g++ 就能编。
//
//  ── 编译(飞机上) ──
//    cd ~/acfly_ws/src/fly_mission
//    g++ -std=c++17 -O2 tools/udp_pose_monitor.cpp -Iinclude -o /tmp/udp_pose_monitor
//
//  ── 运行 ──
//    /tmp/udp_pose_monitor            # 默认端口 9870
//    /tmp/udp_pose_monitor 9871       # 指定端口
//
//  ── 看什么 ──
//    · 有输出        = UDP 链路通
//    · Hz 稳定       = 发送端限频正常、网络没堵
//    · 丢包数不涨    = WiFi 质量够(UDP 不重传，偶尔丢几个正常)
//    · age 一直很小  = 数据新鲜
//    完全没输出 → 查：发送端 dest_ip 对不对 / 端口一致吗 / 防火墙
//                     (ufw 放行: sudo ufw allow 9870/udp)
// ============================================================================

#include "fly_mission/udp_pose_link.hpp"

#include <cmath>      // M_PI
#include <cstdio>
#include <cstdlib>
#include <csignal>

using fly_mission::udp_pose::UdpPoseReceiver;

static volatile bool g_run = true;
static void on_sigint(int) { g_run = false; }

int main(int argc, char** argv)
{
    const int port = (argc > 1) ? std::atoi(argv[1])
                                : fly_mission::udp_pose::DEFAULT_PORT;
    std::signal(SIGINT, on_sigint);

    UdpPoseReceiver rx(port);
    if (!rx.ok()) {
        std::printf("★bind 端口 %d 失败★(被别的程序占用了? 换个端口试试)\n", port);
        return 1;
    }
    std::printf("监听 UDP 端口 %d，等待位姿…(Ctrl+C 退出)\n", port);

    double   t_last_print = UdpPoseReceiver::now_mono();
    uint32_t pkt_in_window = 0;
    uint32_t last_lost = 0;

    while (g_run) {
        if (rx.poll()) ++pkt_in_window;

        const double now = UdpPoseReceiver::now_mono();
        if (now - t_last_print >= 1.0) {
            const double dt = now - t_last_print;
            if (rx.have()) {
                const auto& p = rx.latest();
                const uint32_t lost_now = rx.lost_count();
                std::printf("seq=%-8u  %.1f Hz  age=%.3fs  丢包累计=%u(+%u)  "
                            "pose=(%7.2f,%7.2f,%6.2f) yaw=%6.1f°\n",
                            p.seq, pkt_in_window / dt, rx.age_sec(),
                            lost_now, lost_now - last_lost,
                            p.x, p.y, p.z, p.yaw * 180.0 / M_PI);
                last_lost = lost_now;
            } else {
                std::printf("…还没收到任何包(发送端起了吗? dest_ip/端口对吗?)\n");
            }
            std::fflush(stdout);
            t_last_print  = now;
            pkt_in_window = 0;
        }

        // 1ms 轮询：UDP 非阻塞，靠这个避免空转吃满 CPU
        timespec ts{0, 1000000};
        nanosleep(&ts, nullptr);
    }
    std::printf("\n退出。\n");
    return 0;
}
