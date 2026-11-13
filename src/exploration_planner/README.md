# exploration_planner：区域测绘与返航

这个 ROS 2 节点读取 Point-LIO 的里程计和点云，从当前场地的未覆盖区域选择观察位置，沿已检查的路径绕开障碍。覆盖率达到阈值后，返回主控提供的终点。任务期间保持主控提供的巡航高度；主控负责起飞和落地。节点本身不连接飞控执行接口。

## 参数入口

`config/` 中的五个文件会在节点启动时自动加载。每个参数只有一个默认配置位置。可在启动时叠加 ROS 的 `--params-file` 或 `-p`；拼错参数、缺少必填项或关键数值越界时，初始化失败并报错。参数为启动期只读，修改后需要重新启动节点。普通安装使用安装目录的配置副本；开发工作区用 `--symlink-install` 或重新安装配置后才会反映源码修改。

| 文件 | 内容 |
| --- | --- |
| [mapping.yaml](config/mapping.yaml) | 场界、栅格、视场、点云高度过滤、障碍轮廓和机体半径 |
| [coverage.yaml](config/coverage.yaml) | 全场归航阈值、前沿选点评分、重规划、来路记录 |
| [motion.yaml](config/motion.yaml) | 跟踪、预测、转弯、A* 间距与安全余量、横漂补偿 |
| [return.yaml](config/return.yaml) | 返航路径、末段安全入口、到点与停稳门槛 |
| [runtime.yaml](config/runtime.yaml) | 控制周期、诊断频率、数据新鲜度和仿真时钟匹配 |

`coverage_thresh` 是一个大格中多少比例的小格已扫描才算这个大格完成；`search_coverage` 是全场已完成大格的比例。当前场界是 `camera_init` 下的 `[-1, 19] × [-1, 19] m`，覆盖目标为 95%。新的飞行任务不会清空同进程已经累计的地图；需要全新的场地测绘时，重启节点。

## 代码定位

- [mission.cpp](src/node/mission.cpp)：接收带任务标识的终点、选择覆盖或返航阶段、核对数据有效性和发布任务完成。
- [sensors.cpp](src/node/sensors.cpp)、[obstacle_map.cpp](src/obstacle_map.cpp)、[grid_map.cpp](src/grid_map.cpp)：解锁建图、跨时域门限、里程计/点云过滤、障碍聚类与覆盖记账。
- [coverage_search.cpp](src/node/coverage_search.cpp)、[coverage_planner.cpp](src/coverage_planner.cpp)：前沿选点与重规划；[coverage_path.cpp](src/node/coverage_path.cpp)、[coverage_execution.cpp](src/node/coverage_execution.cpp)：验证、接续和跟踪覆盖路线。
- [global_planner.cpp](src/global_planner.cpp)、[bezier.cpp](src/bezier.cpp)、[trajectory_tracker.cpp](src/trajectory_tracker.cpp)：全局绕障、曲线采样与轨迹跟踪；[path_guard.cpp](src/node/path_guard.cpp)、[recovery.cpp](src/node/recovery.cpp)：预测制动、可行性复核与受限的脱困。
- [home.cpp](src/node/home.cpp)、[home_approach.hpp](include/exploration_planner/home_approach.hpp)、[home_execution.cpp](src/node/home_execution.cpp)：从安全入口返回指定终点；任一时刻发现末段被新障碍挡住，会撤销该段并重新规划。
- [interfaces.cpp](src/node/interfaces.cpp)、[configuration.cpp](src/node/configuration.cpp)、[telemetry.cpp](src/node/telemetry.cpp)：ROS 话题、参数加载、状态与 RViz 可视化数据。`src/main.cpp` 只负责初始化和运行节点。

## 握手与故障定位

| 话题 | 方向 / 约定 |
| --- | --- |
| `/aft_mapped_to_init`、`/cloud_registered` | 输入：Point-LIO 里程计、世界系 XYZ 点云；可靠更新后才前进 |
| `/mavros/state`、`/clock` | 输入：第一次连接且解锁时才开放累计；仿真点云时钟与 ROS 时间不同时，用新鲜且与点云匹配的 `/clock` 确定解锁门限 |
| `/exploration/goal`、`/exploration/context` | 输入：瞬态本地；终点和高度上下文的任务 Header 须相同，坐标系为 `camera_init`，标识时间戳非零 |
| `/exploration/control` | 发给主控：`base_link` 下的水平速度/航向角速度，加本次任务巡航高度 |
| `/exploration/finished_goal` | 发给主控：在通过路径检查的精确终点达到距离、水平速度门槛后发一次，沿用原任务 Header；瞬态本地 |
| `/exploration/diagnostics`、`/exploration/coverage` | 诊断与覆盖率。先看状态 `waiting_for_pose`、`waiting_for_cloud` 或 `waiting_for_mission`，再看恢复原因、最后一次规划原因和路径阻塞原因 |
| `/exploration/active_path`、`/exploration/obstacle_cloud`、`/exploration/field_boundary` | RViz 观测：当前采用的路径、实际计入的占据格和场界 |
| `/exploration/cmd_vel`、`/exploration/finished` | 便于只读监视的速度和完成标志；任务身份以带 Header 的握手接口为准 |

定位失效时不再发任务控制命令；若定位仍有效而有效点云超时，发送零水平速度并冻结覆盖更新。点云重复时间戳不重复积累。地图采用累计证据，没有障碍点并不表示看见了自由空间。

## 验证

[planner_contract_test.cpp](tests/planner_contract_test.cpp) 直接在进程内调用真实节点的回调并拦截发布，覆盖首次解锁、异域仿真时钟、过期点云、任务握手、先探索后返航、精确终点及完成去重。其他 CTest 覆盖前沿、栅格可见性、路径净空及跟踪。轨迹回归使用冻结的历史场景增益；它们不替代当前场景的闭环仿真与实飞验证。
