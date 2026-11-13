# ACFly 区域测绘工作空间

## 文件导览（纯文本）

文档和配图都从文件结构出发。先按下面的外层到内层分支定位职责，再进入具体包；完整逐文件说明见 [docs/FILE_GUIDE.md](docs/FILE_GUIDE.md)。

```text
acfly_ws/
├── config/                    运行参数的统一编辑索引
├── src/                       ROS 2 包：接口、配置、主控、探索和感知
│   ├── acfly_interfaces/      主控与探索的消息协议
│   ├── acfly_config/          YAML 默认值和参数校验
│   ├── acfly_bringup/         统一 launch 入口
│   ├── fly_mission/           起飞 → 探索交接 → 降落的单次主控
│   ├── exploration_planner/   建图、覆盖、避障、返航和轨迹跟随
│   └── perception/            公共点云库、单杆、多杆和圆环检测
├── docs/                      项目说明、维护手册、历史证据和图表源
└── tools/                     结构检查、隔离测试、仿真和配图导出
```

ACFly 是一个基于 ROS 2 Humble 的无人机区域测绘项目。它把主控、区域探索、点云检测、消息接口和运行参数拆成独立 ROS 包，便于新成员理解代码、定位故障和替换单个功能。

默认任务由主控和探索规划器协作完成。主控负责地面自检、人工放行、起飞、巡航高度保持、探索交接和降落。探索规划器负责障碍地图、区域覆盖、路径跟随、避障和返航。任务完成后主控请求 AUTO.LAND，上锁后进程结束。

![ACFly 模块架构](docs/assets/architecture.svg)

图表的可编辑源文件是 [architecture.mmd](docs/assets/architecture.mmd)；配置图的源文件是 [config-layers.mmd](docs/assets/config-layers.mmd)。修改图时编辑 `.mmd`，再运行 `tools/render_diagrams.sh` 更新 SVG 预览。

## 从这里开始

- [完整项目概览](docs/PROJECT_OVERVIEW.md)
- [仓库结构与维护手册](docs/MAINTENANCE.md)
- [参数配置索引](config/README.md)
- [20 × 20 m 仿真场景说明](docs/ARENA_20X20_SIM.md)
- [历史资料说明](docs/history/README.md)
- [配图源文件与导出说明](docs/assets/README.md)

## 包和目录

| 路径 | 职责 |
| --- | --- |
| src/fly_mission | 单次任务主控、飞控桥、人工接管和降落生命周期 |
| src/exploration_planner | 点云建图、覆盖规划、避障、轨迹跟随和安全返航 |
| src/perception | 单杆、多杆、圆环检测和公共点云处理库 |
| src/acfly_interfaces | 主控和探索之间的 ROS 消息 |
| src/acfly_config | 分类 YAML 加载、参数类型和未知参数检查 |
| src/acfly_bringup | 主控与探索的统一启动入口 |
| config | 各包配置文件的编辑索引 |
| docs | 项目概览、维护手册、仿真说明和历史记录 |
| tools | 结构检查、隔离构建和现场准备工具 |

## 快速构建

~~~bash
cd ~/acfly_ws
source /opt/ros/humble/setup.bash

CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths src \
  --build-base build/survey \
  --install-base install/survey \
  --log-base log/survey \
  --symlink-install \
  --executor sequential

source install/survey/setup.bash
ros2 launch acfly_bringup survey.launch.py
~~~

外部定位、飞控和传感器需要先按现场环境启动。统一入口负责主控和探索节点。场景参数可以通过两个独立的启动参数覆盖。

~~~bash
ros2 launch acfly_bringup survey.launch.py \
  planner_params:=/absolute/path/planner_override.yaml \
  mission_params:=/absolute/path/mission_override.yaml
~~~

三个点云检测器可以独立启动。

~~~bash
ros2 run pole_detector pole_detector_node
ros2 run multi_pole_detector multi_pole_detector_node
ros2 run ring_detector ring_detector_node
~~~

## 参数和验证

运行参数按功能写在 YAML 文件中。请从 [config/README.md](config/README.md) 找到需要修改的文件。修改后重启对应节点。

~~~bash
python3 tools/check_workspace.py
tools/test_workspace.bash
~~~

tools/test_workspace.bash 会在临时工作区构建和测试，不会改动当前安装空间。完整接口、配置、故障定位和验证范围见 [项目概览](docs/PROJECT_OVERVIEW.md)。

## 当前工作区状态

源码结构已经完成本机整理，默认值已经归类到 YAML，主控默认任务是一次区域测绘任务。新接口需要主控和探索规划器一起构建。Git 提交、远程仓库发布和现场部署由项目维护者在审阅后执行。
