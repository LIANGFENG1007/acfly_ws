# 交给【发命令那台机器】上的 Claude 的提示词 —— 启动指令走 UDP

飞机主控在 BEEP① 之后要等一个"启动指令"才继续起飞。原来靠 ROS 话题
`/mission/start`（Int32 data=1），现在改成 **UDP**。

把下面 `---8<---` 之间整段复制给那台机器上的 Claude Code。

---8<--- 复制从这里开始 ---8<---

## 背景

我有一台飞机，它的主控程序在启动流程中有一步是"等待启动指令"：

```
飞控连接 + 雷达就绪 → 蜂鸣器响第一声(BEEP①) → ★等启动指令★
  → 收到后响第二声(BEEP②) → 等飞手拨 OFFBOARD → 自动解锁起飞
```

原来这个启动指令是通过 ROS 话题发的：
```bash
ros2 topic pub -r 1 /mission/start std_msgs/msg/Int32 "{data: 1}"
```

**现在要改成 UDP**。原因：两台机器在同一 ROS domain 时，Fast DDS 靠**组播**做发现，
且是"全连接"——每个节点都要和对面每个节点建连并周期心跳，连接数是**乘积关系**。
飞机上 mavros 有 50+ 插件节点，另一台机器一开就把 WiFi 组播挤爆（WiFi 组播按最低
速率发送且不重传），DDS 线程还抢 CPU，导致**飞机上 mavros 的串口失去实时性、
setpoint 发不出去**。所以只要还有任何一个 ROS 话题跨机，问题就还在——命令也必须
走 UDP。

## 已确认的环境参数（不要改这些值）

| 项目 | 值 |
|---|---|
| 飞机（接收端）IP | `192.168.12.1` |
| **命令 UDP 端口** | **`9871`** |
| 位姿 UDP 端口（另一条通道，别混） | `9870` |
| 飞机的 `ROS_DOMAIN_ID` | `73` |
| **本机应设的 `ROS_DOMAIN_ID`** | **`74`**（与飞机不同） |

## 协议格式（飞机端按这个解，不能改）

小端，定长 **16 字节**：

| 偏移 | 类型 | 字段 | 值 |
|---|---|---|---|
| 0 | uint32 | `magic` | `0x434D4431`（'CMD1'） |
| 4 | uint32 | `cmd` | `1` = 启动指令，`2` = 二次起飞 |
| 8 | int32 | `arg` | 填 `0`（当前命令都不用参数） |
| 12 | uint32 | `seq` | 每包 +1（仅用于日志，接收端不依赖） |

飞机端会**丢弃**：长度 ≠ 16 的包、`magic` 不匹配的包、`cmd` ≥ 8 的包。

**可靠性策略**：UDP 不保证送达，而启动指令丢了就起飞不了。所以
- 发送端要**每秒重复发一次**，一直发到我确认飞机响了第二声蜂鸣
- 飞机端"**收到一次即锁定**"，重复发的包会被忽略，**不会重复触发**，放心一直发

## 请你帮我做的事

### 任务 1：写一个发命令的小工具

我不需要 ROS 节点，一个独立的小程序就行（Python 或 C++ 都可以，**Python 更好**，
免编译、改起来方便）。要求：

- 用法类似：`./send_start.py <飞机IP> start` 或加个 `again` 参数发二次起飞
- **默认每秒发一次、一直发**，直到我 Ctrl+C
- 支持可选参数只发 N 次
- 每次发送打印一行（第几次、seq 多少），让我知道它真的在工作
- 端口默认 `9871`，可用参数覆盖
- 按上面协议表严格打包（Python 用 `struct.pack('<IIiI', magic, cmd, arg, seq)`）

### 任务 2：写一个桌面一键脚本

文件名：`~/桌面/发送启动指令.sh`（先检查桌面目录实际是中文 `桌面` 还是英文 `Desktop`）

内容就是调用任务 1 的工具，把飞机 IP `192.168.12.1` 写死在里面，双击就能跑。
脚本里用注释标明 IP 和端口在哪一行，方便我以后改。

**另外**：如果这台机器上已经有我之前让你做的"启动雷达并转发"脚本，
**不要动它**，这是独立的另一个脚本。

### 任务 3：告诉我怎么验证

- 怎么确认包真的发出去了（不依赖飞机那边）
- 如果飞机没反应，怎么排查是发送侧还是网络问题

## 注意事项

- **不要修改协议**（16 字节、magic、字段顺序），飞机端是按这个解的
- 命令端口是 `9871`，**不要用 9870**——那是位姿通道，两者不能混
- 如果本机没有 Python3 或缺 socket 权限，告诉我

---8<--- 复制到这里结束 ---8<---

<br>

# 飞机端已经改好了（这部分不用给对方）

## 改了什么

| 文件 | 改动 |
|---|---|
| `include/fly_mission/udp_cmd_link.hpp` | **新增**：命令协议 + 收发端（16B 包，独立于位姿的 48B 包） |
| `params.hpp` | 新增 `CMD_USE_UDP = true`、`CMD_UDP_PORT = 9871` |
| `fly_mission_node.cpp` | 按开关二选一：UDP 收命令 / 订 `/mission/start`；`on_timer` 里 `cmd_rx_->poll()` |
| `tools/udp_cmd_send.cpp` | 飞机端也能编一个发送工具，用于自测 |

**状态机逻辑一行没改**：UDP 收到 `CMD_START` 就置 `start_recv_`，
`BOOT_CHECK` 仍然只看这个标志，与 ROS 话题版行为完全一致。

`CMD_USE_UDP = false` 可一键回到 ROS 话题模式。

## 顺带也支持了二次起飞

原 `/mission/takeoff_again` 也走同一条 UDP 通道（`cmd=2`），
用不到就不会触发。

## 飞机端自测（不用等对方）

```bash
# 终端1：编译发送工具
cd ~/acfly_ws/src/fly_mission
g++ -std=c++17 -O2 tools/udp_cmd_send.cpp -Iinclude -o /tmp/udp_cmd_send

# 终端2：跑主控（会停在"等启动指令"）
ros2 run fly_mission fly_mission_node

# 终端1：给自己发
/tmp/udp_cmd_send 127.0.0.1 start
```

主控应打印：
```
[启动] 收到启动指令(UDP seq=1，累计收包 1)（已锁定，后续重复发将忽略）
```

## 已验证的行为（19 项测试全过）

- 锁定语义：收到一次后 `got()` 恒为 true
- **重复发 5 次不会重复触发**（这是最关键的一条——对方会持续发）
- `start` 和 `again` 两个命令互不干扰
- 脏包全部丢弃：magic 错、长度错、未知 cmd 编号
- 脏包不计入收包统计
- `reset()` 后可重新触发（二次起飞多轮用）
- 命令端口 9871 与位姿 9870 隔离、包大小 16B vs 48B 不同

## 监听命令通道的内容

```bash
# 方法1：tcpdump（不占端口，能和主控同时用）
sudo tcpdump -i wlP1p1s0 udp port 9871 -X -c 5
# 开头应看到 magic：31 44 4d 43（小端存的 'CMD1' 倒序）

# 方法2：Python 解包（主控没在跑时用，会占端口）
python3 -c "
import socket, struct
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 9871))
print('监听命令端口 9871 …')
while True:
    d, addr = s.recvfrom(32)
    if len(d) != 16: continue
    magic, cmd, arg, seq = struct.unpack('<IIiI', d)
    if magic != 0x434D4431: continue
    name = {1:'START', 2:'TAKEOFF_AGAIN'}.get(cmd, f'未知({cmd})')
    print(f'from {addr[0]}  cmd={name}  arg={arg}  seq={seq}')
"
```

**注意**：主控 `fly_mission_node` 运行时会 bind 9871，方法 2 会抢端口，
两者不要同时开。要边跑主控边看就用 tcpdump。

## 现在飞机端监听两个 UDP 端口

| 端口 | 用途 | 包大小 | 谁发 |
|---|---|---|---|
| `9870` | 小车位姿（追踪用） | 48B | 雷达机的 `udp_pose_sender` |
| `9871` | 启动指令 / 二次起飞 | 16B | 任意机器的 `udp_cmd_send` |

两条通道完全独立，端口和协议都不同，不会互相干扰。
