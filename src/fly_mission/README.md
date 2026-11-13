# fly_mission：单次测绘任务主控

`fly_mission_node` 只执行一次任务：等待人工放行 → 起飞并稳定 → 水平探索测绘 → AUTO.LAND。上锁且高度回到起飞点附近，进程正常退出；飞手接管则中止退出（状态码 2）。主控不请求解锁，也不请求切入 OFFBOARD。探索的 goal、context、control 和 finished_goal 四个 DDS 对端均已匹配后，本次进程必须在地面看见一次已上锁、定位新鲜且飞控报告 ON_GROUND，才可开始零速度预热并等待飞手解锁；已解锁时才启动的进程不会接管。任务不包含串口、二次起飞、穿门或移动目标跟踪。

| 要找的内容 | 位置 |
|---|---|
| 起飞高度、返航点、选用 ROS 启动信号 | `config/mission.yaml` |
| 速度、位置保持、取样、容差 | `config/flight_control.yaml` |
| 定位、飞控、指令超时，以及降落交接重试 | `config/watchdogs.yaml` |
| 状态切换与飞手接管 | `src/mission_node.cpp` |
| 起飞、探索、降落三段 | `src/takeoff.cpp`、`src/survey.cpp`、`src/landing.cpp` |
| 位姿与速度估算、位置保持、MAVROS 输出 | `src/flight_feedback.cpp`、`src/position_hold.cpp`、`src/mavros_link.cpp` |
| 同探索规划器的任务握手 | `src/exploration_link.cpp` |

包安装后 `ros2 run fly_mission fly_mission_node` 读取三份 YAML。可在启动时用 `--ros-args -p 参数名:=值` 或 `--params-file 文件路径` 覆盖；配置只读，修改在节点下次启动时生效。使用普通复制安装时，编辑源码配置后需要重新安装；使用软链接安装时确认安装文件确实指向源码。

主控根据 `/mavros/state`、`/mavros/extended_state` 的实时地面标志、`/aft_mapped_to_init` 和飞手实际解锁与模式判断是否可起飞。`/mission/state`（String，transient local）可观察阶段。若另开启动信号门槛，`/mission/start` 的 Int32(1) 只放行本任务，不能替代飞手授权。

主控同时发布 `/exploration/context` 与 `/exploration/goal`（transient local）。两条消息共享 `camera_init` 坐标系及同一任务时间戳。`/exploration/control` 必须携带该任务标识、递增时间戳与 `base_link` 平面速度。主控锁定本次巡航高度；陈旧/无指令时保持当前位置，里程计过期时发零速度，不续用末次指令。只有 `/exploration/finished_goal` 回传本任务 header 才进入降落；确认 AUTO.LAND 后立即停发 OFFBOARD setpoint。重启前应先确认旧任务已结束。

旧版无任务标识的速度流、动态通道路线、载荷接口及旧 UDP 协议不属于本包接口。主控与规划器须使用同一版 acfly_interfaces；旧安装空间中的可执行文件不会因源码变化而自动升级。
