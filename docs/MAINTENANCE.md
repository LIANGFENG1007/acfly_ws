# 模块与故障定位

完整逐文件说明见 [首页文件导览](../README.md#文件导览)。本页从运行现象反查节点、接口和参数。

先看哪个节点在等待什么，再改对应分类的 YAML。雷达驱动和 Point-LIO 不在本仓库；三个几何检测器是已配准点云的消费者，并不产生定位。

| 现象 | 先核对 | 源码或配置 |
|---|---|---|
| 不进入起飞或中途退出 | 主控日志逐项显示自检等待原因，包括缺少或超时的话题、飞控模式、地面状态与人工放行。用 /mission/state 确认阶段 | fly_mission/src/preflight.cpp、mission_node.cpp，mission.yaml、watchdogs.yaml |
| 未收到 /aft_mapped_to_init | 核对 Point-LIO 是否仍在运行，以及 /lidar/points、/imu 和里程计是否持续出帧。仿真重启前先结束旧的 Point-LIO 及其桥接，避免重复发布和时间戳回退 | 外部 Point-LIO 仿真启动日志，fly_mission/src/flight_feedback.cpp |
| 绕圈未开始或停在 orbit_hold | 核对入圈位置、定位新鲜度、超时和飞控模式。开启面向圆心时还需满足入圈朝向容差 | fly_mission/src/orbit.cpp，[orbit.yaml](../src/fly_mission/config/orbit.yaml) |
| 有定位但不更新障碍图 | /cloud_registered 的坐标系、消息时间，是否已在解锁后收到新帧，仿真 /clock 是否与点云同域 | exploration_planner/src/node/sensors.cpp，mapping.yaml、runtime.yaml |
| 覆盖率不涨或路径改变 | /exploration/coverage、/exploration/active_path、/exploration/diagnostics 和 /exploration/obstacle_cloud | exploration_planner/src/node/coverage_search.cpp、path_guard.cpp，coverage.yaml、motion.yaml |
| 完成探索但不降落 | 是否到达返回点并停稳，/exploration/finished_goal 的 header 是否与 /exploration/goal 完全相同 | exploration_planner/src/node/home.cpp，fly_mission/src/exploration_link.cpp、landing.cpp |
| 检测节点有输入却无结果 | 里程计、点云字段/坐标系与裁剪区域，再看裁剪点云和检测阈值 | 对应包 config 下的 input.yaml、region.yaml、detection.yaml |
| 改了参数没生效 | 实际 source 的安装前缀、有效参数、文件节点名，确认节点已重启 | acfly_config/include/acfly_config/defaults.hpp、各包 config/ |

常用只读检查：`ros2 topic info -v /exploration/control` 检查发布/订阅两方与 QoS，`ros2 topic echo --once --qos-durability transient_local /mission/state` 看当前阶段，`ros2 topic hz /aft_mapped_to_init` 看定位频率。覆盖达标仅表示探索开始返航；它与主控已经完成降落是两个事件。

## 接口约定

| 话题 | 方向、类型与用途 |
|---|---|
| /exploration/goal | 主控 → 探索，geometry_msgs/PointStamped；camera_init 中的返回点，header 标识本次任务 |
| /exploration/context | 主控 → 探索，acfly_interfaces/ExplorationContext；同一 header、起飞与巡航高度 |
| /exploration/control | 探索 → 主控，acfly_interfaces/ExplorationCommand；带本次标识、指令时间和 base_link 速度 |
| /exploration/finished_goal | 探索 → 主控，std_msgs/Header；回传完成任务的标识 |

目标、上下文和完成回执使用 transient local QoS，控制流使用 volatile。只由主控向 MAVROS 交付控制输出。协议变化要一起重建发布方和订阅方，并运行 bringup 包的跨节点测试。

## 后续改动约定

- 新功能归到拥有该职责的包。可独立验证的算法与 ROS 回调分开，main 只做配置加载和节点生命周期管理。
- 包、文件、话题和参数用 snake_case；类型用 PascalCase。对外参数名尽量保留，新数值字段在名称或注释里说明单位、坐标系和有效范围。
- 新运行参数只在归属类别的一份 YAML 写默认值；节点声明类型、检查取值，在启动时报出文件或键名错误。固定协议和数学转换常数留在代码。
- 新增运行分类时，将文件纳入包安装规则和加载清单；新增有配置的包时，在根 config/ 建相对符号链接并更新索引。运行结构检查、受影响的包测试及跨包接口测试。
- 提交源码与可重现的小型测试，不提交可执行文件、构建目录、录包、机器配置或私人启动笔记。发布前由项目所有者确认许可证与维护者信息。
