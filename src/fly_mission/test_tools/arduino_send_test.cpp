// ============================================================================
//  arduino_send_test.cpp —— arduino_send 的独立测试(不起飞、不进状态机、无 ROS)
//
//  直接复用 arduino_serial.hpp(和飞行程序发的是同一份代码，测的就是真东西)。
//  用法：
//    ./arduino_send_test <设备路径> [内容] [次数]
//    ./arduino_send_test /dev/pts/5              → 发 "LED ON"×5(默认)
//    ./arduino_send_test /dev/ttyUSB0 "LED OFF" 3 → 真板子发 "LED OFF"×3
//  编译(见 test_tools/README 或直接)：
//    g++ -std=c++17 -I ../include arduino_send_test.cpp -o arduino_send_test
// ============================================================================
#include "fly_mission/arduino_serial.hpp"
#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("用法: %s <串口设备> [内容=\"LED ON\"] [次数=5]\n"
                    "  假串口测试: 先跑 python3 fake_arduino.py 拿到 /dev/pts/N\n", argv[0]);
        return 1;
    }
    const char* dev   = argv[1];
    const char* text  = (argc > 2) ? argv[2] : "LED ON";
    const int   times = (argc > 3) ? std::atoi(argv[3]) : 5;

    // 真板子(接在 /dev/tty_module_A)：打开串口时 DTR 脉冲会复位 Arduino，
    // 必须等它 bootloader 重启进 loop(~2s)，否则前几条指令会丢。
    // 若改用假串口(pty)测试，pty 不会被复位，可把 2.0 改回 0.0。
    fly_mission::ArduinoSerial ser(dev, /*boot_wait_s=*/2.0);
    std::printf("向 %s 发 \"%s\" ×%d (115200,8N1,每条自动补\\n)...\n", dev, text, times);
    const bool ok = ser.send(text, times, /*gap_ms=*/20);
    std::printf(ok ? "发送成功\n" : "发送失败(设备不存在/打不开?)\n");
    return ok ? 0 : 2;
}
