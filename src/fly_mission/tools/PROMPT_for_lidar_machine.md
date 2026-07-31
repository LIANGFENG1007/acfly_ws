# 交给【雷达机/小车机】上的 Claude 的提示词

把下面整段(从 `---8<---` 之间)复制给雷达机上的 Claude Code。

---8<--- 复制从这里开始 ---8<---

## 背景

我有两台机器，需要把这台机器（雷达机 / 小车机）的 SLAM 位姿通过 **UDP 单播**发给另一台飞机上的主控程序。

**为什么不用 ROS 话题跨机**：两台机器同一 ROS domain 时，Fast DDS 靠**组播**做发现，
且是"全连接"——每个节点都要和对面每个节点建连并周期心跳，连接数是**乘积关系**。
飞机上 mavros 有 50+ 插件节点，这台机器一开就把 WiFi 组播挤爆（WiFi 组播按最低速率
发送且不重传），DDS 线程还抢 CPU，导致**飞机上 mavros 的串口失去实时性、setpoint
发不出去**。实测现象就是"雷达机一启动，飞机的 mavros 就不传数据了"。
UDP 单播无发现、无心跳、点对点直达，彻底避开这个问题。

## 已确认的环境参数（不要改这些值）

| 项目 | 值 |
|---|---|
| 飞机（接收端）IP | `192.168.12.1` |
| UDP 端口 | `9870` |
| 本机要发的话题 | `/aft_mapped_to_init2` （nav_msgs/Odometry） |
| 飞机的 `ROS_DOMAIN_ID` | `73` |
| **本机必须设的 `ROS_DOMAIN_ID`** | **`74`**（与飞机不同，这样 DDS 互不可见） |
| ROS 版本 | humble |
| 发送限频 | 30 Hz |

## 我已经有的东西

飞机上已经写好了发送端程序，我会拷两个文件到这台机器：

- `udp_pose_sender.cpp` —— ROS 节点：订 `/aft_mapped_to_init2` → 发 UDP
- `udp_pose_link.hpp` —— 协议定义（收发端共用）

这两个文件**必须放在同一个目录**（cpp 里是 `#include "udp_pose_link.hpp"`，不带路径）。

`udp_pose_sender.cpp` 支持这些 ROS 参数：
`dest_ip`（飞机IP）、`port`、`in_topic`、`rate_hz`。

## 请你帮我做的事

### 任务 1：建一个最小 ROS2 包并编译通过

在 `~/udp_ws/src/udp_pose/` 下建包，结构如下：

```
~/udp_ws/src/udp_pose/
├── package.xml
├── CMakeLists.txt
└── src/
    ├── udp_pose_sender.cpp     ← 我拷进来的
    └── udp_pose_link.hpp       ← 我拷进来的
```

要求：
- `package.xml` 依赖 `rclcpp` 和 `nav_msgs`，构建类型 `ament_cmake`
- **maintainer 的 email 必须是合法邮箱格式**（比如 `me@example.com`）——
  `catkin_pkg` 会校验，写 `a@b.c` 这种顶级域只有一个字符的会直接报
  `Invalid email` 编译失败
- `CMakeLists.txt` 里要有 `target_include_directories(... PRIVATE src)`，
  否则找不到同目录的 `udp_pose_link.hpp`
- 编译命令：`cd ~/udp_ws && colcon build --packages-select udp_pose`
- 确认看到 `Finished <<< udp_pose`

### 任务 2：写一个一键启动脚本放到桌面

文件名：`~/桌面/启动雷达并转发.sh`（如果桌面目录是英文 `~/Desktop`，就放那里；
请先检查实际是哪个）

脚本要做的事，**按顺序**：

1. `source /opt/ros/humble/setup.bash`
2. `source ~/udp_ws/install/setup.bash`
3. **`export ROS_DOMAIN_ID=74`** ← 这步绝对不能漏，漏了整件事就白做了
4. 启动这台机器的**雷达驱动**（请先帮我查清楚本机雷达的启动命令是什么，
   可能是 livox / rslidar / velodyne 等，我不确定具体是哪个）
5. 启动 **SLAM**（能产出 `/aft_mapped_to_init2` 的那个，可能是 Point-LIO / FAST-LIO
   之类；也请你先查清楚本机实际用的是哪个、以及它发的话题名对不对）
6. **等 SLAM 真的开始发 `/aft_mapped_to_init2` 之后**，再启动转发节点：
   ```
   ros2 run udp_pose udp_pose_sender --ros-args \
     -p dest_ip:=192.168.12.1 -p port:=9870 \
     -p in_topic:=/aft_mapped_to_init2 -p rate_hz:=30.0
   ```

脚本的其他要求：

- **每一步之间要有等待和检查**，不要一股脑并行启动。特别是第 6 步，必须确认
  `/aft_mapped_to_init2` 真的有数据了再启动转发（可以用
  `ros2 topic hz /aft_mapped_to_init2` 或 `ros2 topic info` 判断），
  否则转发节点起来了但收不到任何位姿，白跑
- 每一步打印中文提示，让我知道现在进行到哪一步、成功还是失败
- 哪一步失败就明确报错并停下，不要继续往下跑
- 三个程序建议各开一个终端窗口（gnome-terminal / xterm 都行，看本机装了哪个），
  或者用 `tmux`；这样某个挂了我能单独看它的日志
- 脚本要 `chmod +x`，双击或终端都能运行
- 顺便在脚本开头用注释写明：飞机 IP、端口、domain id 这几个关键参数在哪一行，
  方便我以后改

### 任务 3：告诉我怎么验证

编译完、脚本写好后，告诉我：
- 怎么单独测试转发节点是否在发包（不依赖飞机那边）
- 怎么确认 `ROS_DOMAIN_ID=74` 真的生效了

## 注意事项

- 不要修改我拷过来的 `udp_pose_sender.cpp` 和 `udp_pose_link.hpp` 的**协议部分**
  （48 字节定长包、magic `0x50534531`），飞机那边是按这个格式解的，改了就对不上
- 如果本机雷达/SLAM 的启动命令你查不到，**先问我**，不要猜一个写进脚本
- 本机的位姿话题如果实际不叫 `/aft_mapped_to_init2`，告诉我实际名字，
  我需要同步确认飞机那边

---8<--- 复制到这里结束 ---8<---

<br>

# 附：怎么监听这个 UDP 协议的内容

四种方法，从简单到详细。**前两种在飞机上做，后两种任意机器都行。**

## 方法 1：用现成的监控工具（推荐，能看频率和丢包）

在**飞机**上：

```bash
cd /home/nvidia/acfly_ws/src/fly_mission
g++ -std=c++17 -O2 tools/udp_pose_monitor.cpp -Iinclude -o /tmp/udp_pose_monitor
/tmp/udp_pose_monitor            # 默认端口 9870
```

输出（每秒一条）：

```
seq=123  30.0 Hz  age=0.012s  丢包累计=0(+0)  pose=(  1.23,  4.56,  0.89) yaw=  45.2°
```

| 看哪项 | 正常 | 异常说明 |
|---|---|---|
| 有输出 | 链路通 | 无输出 → 用方法 2 确认包到没到 |
| `30.0 Hz` | 稳定 | 忽高忽低 = 网络堵/上游不稳 |
| `丢包累计(+N)` | 长期 `+0` | 持续增长 = WiFi 质量不足 |
| `age` | < 0.1s | 一直变大 = 发送端停了/网断了 |

**注意**：这个工具会 bind 9870 端口。如果主控 `fly_mission_node` 也在跑（它同样监听
9870），两者会抢端口 —— 同一台机器上**不要同时开**。想同时看，用方法 2（tcpdump 是
被动抓包，不占端口）。

## 方法 2：tcpdump —— 看包到底有没有到（不占端口，不依赖任何自研程序）

这是排查"到底是发的问题还是收的问题"最有效的一招：

```bash
# 飞机上（网卡名按实际改，飞机是 wlP1p1s0）
sudo tcpdump -i wlP1p1s0 udp port 9870 -c 10
```

- **抓到包** → 网络完全没问题，问题在接收侧（端口占用？程序没跑？）
- **抓不到包** → 问题在发送侧或网络：
  - 雷达机 `dest_ip` 填错了？必须是 `192.168.12.1`
  - 两端 `port` 不一致？
  - 路由器开了 "AP 隔离"（禁止无线客户端互通）—— 这个坑不少见

想看包的**内容**（十六进制）：

```bash
sudo tcpdump -i wlP1p1s0 udp port 9870 -X -c 3
```

开头应该能看到 magic `31 45 53 50`（小端存的 `0x50534531`，即 'PSE1' 倒序）。

## 方法 3：一行 Python 解包看数值（不用编译，最灵活）

任意机器上，**不占用**主控端口就换个端口测；要看真实流量就在飞机上停掉主控再跑：

```bash
python3 -c "
import socket, struct
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 9870))
print('监听 9870 …')
while True:
    d, addr = s.recvfrom(64)
    if len(d) != 48: continue
    magic, seq, stamp, x, y, z, yaw = struct.unpack('<IIdddd', d)
    if magic != 0x50534531: continue
    print(f'from {addr[0]}  seq={seq:<6} x={x:7.3f} y={y:7.3f} z={z:6.3f} yaw={yaw*57.2958:7.2f}°')
"
```

解包格式 `<IIdddd` 就是协议表：小端、2 个 uint32、4 个 double。

## 方法 4：netcat 快速确认端口有没有流量

```bash
# 只看有没有字节进来（会打乱码，因为是二进制）
nc -ul 9870 | xxd | head -20
```

粗糙但快，适合"就想确认有没有东西进来"。

---

# 附：协议格式（改字段时两端必须同步改）

小端，定长 **48 字节**：

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | uint32 | `magic` | `0x50534531`（'PSE1'），过滤误收的其它 UDP 包 |
| 4 | uint32 | `seq` | 序号，发送端每包 +1（用来算丢包） |
| 8 | double | `stamp` | **发送端** monotonic 秒 |
| 16 | double | `x` | 小车 SLAM 系，米 |
| 24 | double | `y` | |
| 32 | double | `z` | |
| 40 | double | `yaw` | 弧度 |

**`stamp` 不能用来算延迟** —— 它是发送端的 `CLOCK_MONOTONIC`，与飞机**不同源**
（两台机器各自从开机计时），相减没有物理意义。判数据新鲜度要用"本机收包时刻"
（`udp_pose_monitor` 的 `age`、`CarTracker` 里的 `node_->now()`）。

接收端会丢弃：长度 ≠ 48 的包、`magic` 不匹配的包。
