#!/usr/bin/env python3
"""
fake_arduino.py —— 假 Arduino(串口测试用，不用插真板子)

原理：os.openpty() 造一对虚拟串口(pty)。本脚本抱着"主端"当 Arduino，
打印出"从端"的设备路径(如 /dev/pts/5)——发送端(arduino_send / 测试程序)
把它当串口打开写入，写进来的每一行都会带时间戳打印在这里。

用法：
    终端A:  python3 fake_arduino.py          ← 显示 "假串口 = /dev/pts/N"，等着收
    终端B:  ./arduino_send_test /dev/pts/N   ← 发 "LED ON"×5
    回终端A看：应打出 5 行 [收到] LED ON
Ctrl+C 退出。
"""
import os
import sys
import time

master, slave = os.openpty()
path = os.ttyname(slave)
print(f'假串口 = {path}   (把它当 ARDUINO_DEV 用；Ctrl+C 退出)', flush=True)

buf = b''
count = 0
try:
    while True:
        data = os.read(master, 256)        # 阻塞等发送端写入
        if not data:
            continue
        buf += data
        while b'\n' in buf:                # 按行切(发送端每条指令带 \n，和真 Arduino 一样)
            line, buf = buf.split(b'\n', 1)
            count += 1
            print(f'[收到 #{count}] {line.decode(errors="replace")!r}  '
                  f'({time.strftime("%H:%M:%S")})', flush=True)
except KeyboardInterrupt:
    print(f'\n共收到 {count} 行，退出')
