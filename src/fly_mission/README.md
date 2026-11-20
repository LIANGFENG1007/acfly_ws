# fly_mission

默认任务是人工放行后起飞，进入设定圆周，完成指定角度并在圆弧终点停稳，然后请求 `AUTO.LAND`。主控启动后需先收到新鲜的里程计、MAVROS 地面状态和至少一次地面上锁状态。飞手解锁并切入 `OFFBOARD` 后才起飞。可用 `/mission/state` 查看 `preflight → takeoff → orbit → landing → finished`。

## 绕圈参数

打开 [orbit.yaml](config/orbit.yaml)，每个字段均有注释。圆心与 `/aft_mapped_to_init` 的位置使用同一坐标系（Point-LIO 默认是 `camera_init`）。

| 调整内容 | 参数 |
| --- | --- |
| 圆心和半径，单位米 | `orbit_center_x_m`、`orbit_center_y_m`、`orbit_radius_m` |
| 俯视转角，正值逆时针，负值顺时针 | `orbit_angle_deg`，例如 `90.0` 或 `-360.0` |
| 水平速度和速度指令的变化上限 | `orbit_speed_mps`、`orbit_accel_mps2` |
| 机头朝向 | `orbit_face_center: true` 朝向圆心，`false` 保持起飞稳定后的航向 |
| 入圈与结束判据 | 半径、角度和航向误差，实测停稳速度与持续时间 |
| 动作超时 | `orbit_timeout_s`，包括入圈、绕行和停稳 |

起飞后先到达离当前位置最近的圆周点。恰在圆心时，沿起飞航向选入圈点。到达圆周并停稳才开始按实测里程计的累计角度计算进度。在目标角度附近减速并停稳后垂直降落。绕圈路径由场地和参数确定，仿真请选无障碍区域。定位断流或动作超时进入 `orbit_hold`，停下绕行并等待飞手接管。

## 启动和维护

仿真定位和 MAVROS 就绪后，使用 `ros2 run fly_mission fly_mission_node`。任务选择由 [mission.yaml](config/mission.yaml) 的 `mission_action` 决定。平时直接修改 [orbit.yaml](config/orbit.yaml)。单次覆盖参数可在命令末尾加 `--ros-args --params-file /absolute/path/orbit_override.yaml`。

`orbit.launch.py` 是可选的启动封装，启动同一个主控并指定绕圈模式，接受 `mission_params` 传入的参数文件。

| 位置 | 职责 |
| --- | --- |
| [mission_node.cpp](src/mission_node.cpp) | 调度起飞、动作、降落和人工接管 |
| [preflight.cpp](src/preflight.cpp) | 起飞前检查、放行条件和等待原因 |
| [takeoff.cpp](src/takeoff.cpp) | 爬升、停稳和任务交接 |
| [orbit.cpp](src/orbit.cpp) | 入圈、沿圆弧的速度控制、角度累计和停稳 |
| [landing.cpp](src/landing.cpp) | 降落模式交接、上锁确认和退出 |
| [config.cpp](src/config.cpp) | 读取和校验分类 YAML |
| [flight_control.yaml](config/flight_control.yaml) | 控制周期、位置保持增益和速度上限 |
| [watchdogs.yaml](config/watchdogs.yaml) | 飞控和定位新鲜度、接管与降落请求时序 |

参数在启动时读取。使用符号链接安装时修改源码 YAML 后重启节点即可生效。使用复制安装时先更新安装空间，或显式传入覆盖文件。

自检会在同一行列出当前未就绪的项目。定位和地面已就绪后，会提示飞手解锁、当前模式，以及是否还在等 `/mission/start`。原因变化时立即提示，原因未变时按 `preflight_log_period_s` 提示一次。

## 区域测绘

使用 `ros2 launch acfly_bringup survey.launch.py` 启动主控和探索规划器。测绘预检需四个探索 DDS 对端都已匹配。可选的 `/mission/start` Int32(1) 信号只放行已配置的任务，飞手仍需授权起飞。

主控同时发布 `/exploration/context` 与 `/exploration/goal`（transient local）。两条消息共享 `camera_init` 坐标系及同一任务时间戳。`/exploration/control` 必须携带该任务标识、递增时间戳与 `base_link` 平面速度。主控锁定本次巡航高度；陈旧/无指令时保持当前位置，里程计过期时发零速度，不续用末次指令。只有 `/exploration/finished_goal` 回传本任务 header 才进入降落；确认 AUTO.LAND 后立即停发 OFFBOARD setpoint。重启前应先确认旧任务已结束。

旧版无任务标识的速度流、动态通道路线、载荷接口及旧 UDP 协议不属于本包接口。主控与规划器须使用同一版 acfly_interfaces；旧安装空间中的可执行文件不会因源码变化而自动升级。
