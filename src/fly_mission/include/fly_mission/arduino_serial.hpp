#pragma once

// ============================================================================
//  arduino_serial.hpp  ── 串口发指令给 Arduino(115200, 8N1)
//
//  用法(fly_mission_node 里)：
//      arduino_send("LED ON");            // 发 "LED ON\n" ×5 次(次数在 params)
//      arduino_send("LED OFF", 1);        // 指定只发 1 次
//  纯 POSIX termios，无 ROS 依赖(风格同 shm_mailbox.hpp)。
//
//  设计要点：
//    · ★懒打开★：构造不碰串口；第一次 send() 才 open。Arduino 没插/仿真机没这个
//      设备 → send() 返回 false 并打一次告警，【绝不影响飞行】。
//    · ★打开后保持★：Linux 打开串口默认拉 DTR 会复位 Arduino(UNO/Nano 复位后
//      bootloader 要 ~2s 才进 loop)。所以 fd 全程只 open 一次；首次 open 后等
//      ARDUINO_BOOT_WAIT_S 再发，防止"第一条指令永远丢"。
//    · 每次发送自动补 '\n'(Arduino 端 readStringUntil('\n') 好切包)；重复 N 次
//      之间隔 ARDUINO_SEND_GAP_MS，防 Arduino 端 64B 串口缓冲一次涌爆。
//    · send() 是【阻塞小 IO】：5 次×(几字节写+间隔) ≈ 几十 ms。在状态机 case 里
//      一次性动作(如到点后开灯)没问题；不要每拍(50Hz)都调。
// ============================================================================

#include <cstdio>
#include <cstring>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace fly_mission {

class ArduinoSerial
{
public:
    // dev: 串口设备路径。boot_wait_s: 首次打开后等 Arduino 复位重启的时间。
    explicit ArduinoSerial(const char* dev, double boot_wait_s = 2.0)
        : dev_(dev), boot_wait_s_(boot_wait_s) {}

    ~ArduinoSerial()
    {
        if (fd_ >= 0) ::close(fd_);
    }

    // 发 ASCII 指令(自动补 \n)，重复 times 次、每次间隔 gap_ms 毫秒。
    // 返回 false = 串口打不开/写失败(没插 Arduino 等)——调用方打日志即可，别中断任务。
    bool send(const std::string& text, int times = 1, int gap_ms = 20)
    {
        if (fd_ < 0 && !open_port()) return false;
        const std::string line = text + "\n";
        for (int i = 0; i < times; ++i) {
            if (i > 0 && gap_ms > 0) usleep(static_cast<useconds_t>(gap_ms) * 1000);
            const char* p = line.data();
            size_t left = line.size();
            while (left > 0) {                       // write 可能写一半(信号打断)，写完为止
                const ssize_t n = ::write(fd_, p, left);
                if (n < 0) {
                    ::close(fd_); fd_ = -1;          // 设备被拔等：关掉，下次 send 重试重开
                    return false;
                }
                p += n; left -= static_cast<size_t>(n);
            }
        }
        tcdrain(fd_);                                // 等内核发完(115200 下几字节≈亚毫秒)
        return true;
    }

    bool ok() const { return fd_ >= 0; }

private:
    bool open_port()
    {
        fd_ = ::open(dev_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) return false;

        termios tio{};
        if (tcgetattr(fd_, &tio) != 0) { ::close(fd_); fd_ = -1; return false; }
        cfmakeraw(&tio);                             // 原始模式：不做行处理/回显/流控
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tio.c_cflag |= (CLOCAL | CREAD);             // 忽略调制解调线，允许收(留双向余地)
        tio.c_cflag &= ~CRTSCTS;                     // 无硬件流控(Arduino 不接 RTS/CTS)
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) { ::close(fd_); fd_ = -1; return false; }
        tcflush(fd_, TCIOFLUSH);                     // 清掉打开瞬间的垃圾字节

        // 打开动作本身会复位 Arduino(DTR 脉冲)：等它重启进 loop，否则首批指令丢失
        if (boot_wait_s_ > 0) usleep(static_cast<useconds_t>(boot_wait_s_ * 1e6));
        return true;
    }

    std::string dev_;
    double      boot_wait_s_;
    int         fd_ = -1;
};

}  // namespace fly_mission
