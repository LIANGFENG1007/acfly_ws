# 跨机位姿传输(UDP)—— 照着敲就行

雷达机的位姿 → UDP → 飞机。绕开 DDS，解决"雷达机一开 mavros 就不传数据"。

---

## 0. 先记住两台机器怎么称呼

| 代号 | 是哪台 | IP | 作用 |
|---|---|---|---|
| **【飞机】** | 就是你现在这台 `nvidia-desktop` | `10.131.100.96` | 跑 mavros/主控，**收**位姿 |
| **【雷达机】** | 房间里另一台，装着第二个雷达 | 待你填 | 发 `/aft_mapped_to_init2`，**发**位姿 |

下面每条命令前面都标了 **【飞机】** 或 **【雷达机】**，照标签在对应机器上执行。

**UDP 端口统一用 `9870`**（两端必须一样）。

---

## 1. 要拷的文件：只有 2 个

| 文件 | 在【飞机】上的完整路径 | 拷到【雷达机】哪里 |
|---|---|---|
| `udp_pose_sender.cpp` | `/home/nvidia/acfly_ws/src/fly_mission/tools/udp_pose_sender.cpp` | `~/udp_ws/src/udp_pose/src/` |
| `udp_pose_link.hpp` | `/home/nvidia/acfly_ws/src/fly_mission/include/fly_mission/udp_pose_link.hpp` | `~/udp_ws/src/udp_pose/src/` |

**两个文件必须放在同一个目录**（`udp_pose_sender.cpp` 里写的是 `#include "udp_pose_link.hpp"`，
不带路径，所以必须同目录）。

另外两个文件**不用拷**，只在【飞机】上用：
- `tools/udp_pose_monitor.cpp` —— 飞机端验证工具
- `tools/README_udp_pose.md` —— 本文档

---

# 第一部分：【雷达机】上做的事

## 步骤 1-1 【雷达机】建目录

```bash
mkdir -p ~/udp_ws/src/udp_pose/src
cd ~/udp_ws/src/udp_pose/src
```

## 步骤 1-2 【雷达机】把两个文件拷过来

```bash
scp nvidia@10.131.100.96:/home/nvidia/acfly_ws/src/fly_mission/tools/udp_pose_sender.cpp .
scp nvidia@10.131.100.96:/home/nvidia/acfly_ws/src/fly_mission/include/fly_mission/udp_pose_link.hpp .
```

会要飞机的密码。拷完确认：

```bash
ls
# 应显示：udp_pose_link.hpp  udp_pose_sender.cpp
```

> 如果 scp 不通，也可以用 U 盘拷，或在【飞机】上执行反向推送：
> `scp <上面两个完整路径> <雷达机用户>@<雷达机IP>:~/udp_ws/src/udp_pose/src/`

## 步骤 1-3 【雷达机】建 package.xml

```bash
cat > ~/udp_ws/src/udp_pose/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>udp_pose</name>
  <version>0.0.0</version>
  <description>UDP pose sender (bypass DDS across machines)</description>
  <maintainer email="a@b.c">you</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>nav_msgs</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF
```

## 步骤 1-4 【雷达机】建 CMakeLists.txt

```bash
cat > ~/udp_ws/src/udp_pose/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(udp_pose)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
add_executable(udp_pose_sender src/udp_pose_sender.cpp)
target_include_directories(udp_pose_sender PRIVATE src)
ament_target_dependencies(udp_pose_sender rclcpp nav_msgs)
install(TARGETS udp_pose_sender DESTINATION lib/${PROJECT_NAME})
ament_package()
EOF
```

## 步骤 1-5 【雷达机】确认目录结构对了

```bash
find ~/udp_ws/src -type f
```

应该正好是这 4 个文件：

```
~/udp_ws/src/udp_pose/package.xml
~/udp_ws/src/udp_pose/CMakeLists.txt
~/udp_ws/src/udp_pose/src/udp_pose_sender.cpp
~/udp_ws/src/udp_pose/src/udp_pose_link.hpp
```

## 步骤 1-6 【雷达机】编译

```bash
cd ~/udp_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select udp_pose
```

成功标志：`Finished <<< udp_pose`

## 步骤 1-7 【雷达机】隔离 ROS 域 ★这步最关键，别漏★

```bash
export ROS_DOMAIN_ID=74
```

**为什么必须做**：飞机是 `ROS_DOMAIN_ID=73`。只要两台机器 domain 相同，DDS 就会继续
互相发现、继续打爆 WiFi 组播 —— 那么 UDP 传得再好也没解决问题。设成不同的数字，
两台机器的 ROS **彻底互不可见**，只剩我们自己的 UDP 通道。

建议永久写入，免得下次忘：

```bash
echo 'export ROS_DOMAIN_ID=74' >> ~/.bashrc
```

## 步骤 1-8 【雷达机】确认位姿话题真的有数据

```bash
source ~/udp_ws/install/setup.bash
ros2 topic list | grep aft_mapped        # 确认话题名对不对
ros2 topic hz /aft_mapped_to_init2       # 应显示稳定频率
```

## 步骤 1-9 【雷达机】启动发送（★正式运行的命令★）

```bash
cd ~/udp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=74

ros2 run udp_pose udp_pose_sender --ros-args \
  -p dest_ip:=10.131.100.96 \
  -p port:=9870 \
  -p in_topic:=/aft_mapped_to_init2 \
  -p rate_hz:=30.0
```

参数含义：

| 参数 | 填什么 |
|---|---|
| `dest_ip` | **飞机的 IP**，现在是 `10.131.100.96`（飞机换网络就要改这里） |
| `port` | `9870`，和飞机端保持一致 |
| `in_topic` | 雷达机上的位姿话题名 |
| `rate_hz` | 发送限频，`30.0` 够用。设 `0` = 不限频 |

启动成功后每 2 秒打一条：

```
[INFO] udp_pose_sender 已启动: /aft_mapped_to_init2 → UDP 10.131.100.96:9870 (限频 30Hz)
[INFO] 已发 seq=61  pose=(1.23, 4.56, 0.89) yaw=45.2°
```

看到 `已发 seq=` 在涨，发送端就没问题了。

---

# 第二部分：【飞机】上做的事

## 步骤 2-1 【飞机】编译监控工具（只需做一次）

```bash
cd /home/nvidia/acfly_ws/src/fly_mission
g++ -std=c++17 -O2 tools/udp_pose_monitor.cpp -Iinclude -o /tmp/udp_pose_monitor
```

> 已经帮你编译好了，`/tmp/udp_pose_monitor` 直接可用。
> 但 `/tmp` 重启会清空，重启后需要重新执行上面这条。

## 步骤 2-2 【飞机】跑监控，验证收得到

```bash
/tmp/udp_pose_monitor
```

（指定别的端口：`/tmp/udp_pose_monitor 9871`）

正常输出，每秒一条：

```
seq=123  30.0 Hz  age=0.012s  丢包累计=0(+0)  pose=(  1.23,  4.56,  0.89) yaw=  45.2°
```

怎么判读：

| 看哪一项 | 正常 | 不正常说明什么 |
|---|---|---|
| 有没有输出 | 有 = 链路通 | 没有 → 跳到第三部分排查 |
| `30.0 Hz` | 稳定 ≈ `rate_hz` | 忽高忽低 = 网络堵或上游不稳 |
| `丢包累计=0(+0)` | 括号里长期 `+0` | 一直 `+N` 增长 = WiFi 质量不足 |
| `age=0.012s` | < 0.1s | 一直变大 = 发送端停了 / 网断了 |

**先把这一步跑通，再做下一步。** 这样万一有问题，能立刻分清是网络问题还是代码问题。

## 步骤 2-3 【飞机】把数据接进主控（验证通了才做）

在你要用坐标的地方：

```cpp
#include "fly_mission/udp_pose_link.hpp"

// 类成员（构造即 bind 端口，失败也不抛异常）
fly_mission::udp_pose::UdpPoseReceiver rx_;

// 每拍调用（放在 20/50Hz 定时器里）
rx_.poll();                                   // 收干本拍所有包，只保留最新一帧
if (rx_.have() && rx_.age_sec() < 0.5) {      // ★必须判 age_sec★
    const auto& p = rx_.latest();
    // 这里可以用 p.x / p.y / p.z / p.yaw
} else {
    // 数据太旧或从未收到 → 自己兜底，千万别拿旧坐标当真
}
```

两个注意点：

- **必须判 `age_sec()`**。UDP 不重传，丢包是正常的，不能假定每帧都到。
- `UdpPoseReceiver` **非线程安全**，只在单一线程里用（比如主控定时器）。

---

# 第三部分：收不到数据时怎么查

按顺序来，每一步都能独立确认一件事。

## 查 1【雷达机】网络通不通

```bash
ping 10.131.100.96
```

不通 → 先解决网络/WiFi，不用往下查了。

## 查 2【飞机】端口在监听吗（monitor 要开着）

```bash
ss -ulnp | grep 9870
```

没输出 → monitor 没跑起来，或端口被别的程序占了（换个端口，**两端一起改**）。

## 查 3【飞机】包到底有没有到（★最关键的一步★）

```bash
sudo tcpdump -i wlP1p1s0 udp port 9870 -c 5
```

这一步不依赖我们写的任何程序，直接看网卡：

- **抓到包** → 网络完全没问题，是接收侧的事（回去看 monitor 参数、端口）
- **抓不到包** → 问题在发送侧或网络：
  - 【雷达机】`dest_ip` 填错了？必须是飞机 IP `10.131.100.96`
  - 两端 `port` 不一致？
  - 路由/隔离：有些 WiFi 路由器开了"AP 隔离"，会禁止客户端之间互通

## 查 4 防火墙

当前【飞机】实测 `ufw status` = **inactive**，不用处理。
如果以后开启了防火墙：

```bash
sudo ufw allow 9870/udp
```

## 查 5 两端端口是否真的一致

【雷达机】的 `-p port:=9870` 和【飞机】monitor 的端口参数必须相同。

---

# 附录 A：日常启动顺序（配置好之后每次这样跑）

**【飞机】**
```bash
# 1. 雷达 → 2. Point-LIO → 3. mavros → 4. 主控
# （和你原来的启动流程完全一样，不用改）
```

**【雷达机】**
```bash
cd ~/udp_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=74
ros2 run udp_pose udp_pose_sender --ros-args \
  -p dest_ip:=10.131.100.96 -p in_topic:=/aft_mapped_to_init2
```

顺序无所谓：UDP 没有"连接"概念，谁先起都行，后起的一方立刻就能收/发。
发送端断了再连也不用重启接收端。

---

# 附录 B：常见疑问

**Q: `stamp` 字段能算延迟吗？**
不能。它是**发送端**的 `CLOCK_MONOTONIC`，和飞机**不同源**（两台机器各自从开机计时），
相减没有物理意义。判数据新鲜度用 `age_sec()`——那个是**本机**收包时刻算出来的。

**Q: 丢包了要不要重传？**
不要，这是刻意设计的。位姿是高频流，30Hz 下丢一帧等 33ms 就有下一帧；重传反而引入
延迟和抖动。所以接收侧**靠 `age_sec()` 判超时**，而不是指望每帧都到。

**Q: 这样会不会又影响 mavros？**
不会。UDP 单播是点对点，没有组播、没有发现、没有心跳。整条链路只有"一次 sendto +
一次 recv"，几十字节。

**Q: 想多传速度/四元数怎么办？**
改 `udp_pose_link.hpp` 里的包布局，**两端必须同步改并重新编译**（协议表里标了每个
字段的偏移量）。

**Q: 飞机 IP 变了怎么办？**
只改【雷达机】启动命令里的 `dest_ip`。想固定下来，建议在路由器上给飞机绑定静态 IP。

---

# 附录 C：协议格式（两端必须一致）

小端，定长 **48 字节**：

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | uint32 | `magic` | `0x50534531`（'PSE1'），防误收其它 UDP 包 |
| 4 | uint32 | `seq` | 序号，发送端每包 +1（用来统计丢包） |
| 8 | double | `stamp` | 发送端 monotonic 秒（★不能当延迟用★） |
| 16 | double | `x` | SLAM 系，米 |
| 24 | double | `y` | |
| 32 | double | `z` | |
| 40 | double | `yaw` | 弧度 |

接收端会**丢弃**：长度 ≠ 48 的包、`magic` 不匹配的包。这两条已实测验证过。
