# 20 × 20 m 区域测绘场景

场景文件是 [arena_20x20.sdf](../tools/sim/worlds/arena_20x20.sdf)。世界坐标地板为 [0, 20] × [0, 20] m，历史场景的机体出生点为 (1, 1, 0)、yaw=0。以这里为 Point-LIO 原点时，对应的测绘坐标就是 [-1, 19] × [-1, 19] m。地图范围和雷达有效高度见 [mapping.yaml](../src/exploration_planner/config/mapping.yaml)，全场覆盖目标见 [coverage.yaml](../src/exploration_planner/config/coverage.yaml)，返回点见 [mission.yaml](../src/fly_mission/config/mission.yaml)。

PX4/Gazebo、MAVROS 和 Point-LIO 是外部系统，不由测绘启动文件启动。沿用已有 20 × 20 世界和上述出生点；只启动一份 /clock 桥、一套里程计和一套飞控桥。在本机旧 Point-LIO 仿真 launch 中请指定 `dense_cloud_adapter:=false`：新版测绘节点使用 /cloud_registered，不依赖旧隧道稠密点云适配器。仿真解锁时间与点云不在同一时域时需提供与点云匹配的 /clock。

上述系统就绪后，用 `ros2 run fly_mission fly_mission_node` 启动默认的绕圈任务，圆心和半径见 [orbit.yaml](../src/fly_mission/config/orbit.yaml)。需要联动启动区域测绘时，可用 `ros2 launch acfly_bringup survey.launch.py`。旧版场景制作步骤见 [2026-11-03 实验记录](history/arena-20x20-20261103.md)。
