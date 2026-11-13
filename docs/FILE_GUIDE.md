# ACFly 文件导览（从外到内）

这份说明使用纯文本从工作空间外层逐级展开到包内文件。它是后续维护和交接的入口，配图只用于帮助理解模块关系，不替代文件说明。新增或移动文件时，同步更新本页和对应包的 README。

## 1. 工作空间根目录

```text
acfly_ws/
├── AGENTS.md                         本地协作规则、证据要求和本轮任务边界
├── README.md                         GitHub 首页；先看这里了解项目定位、入口和验证命令
├── .clang-format                     C/C++ 统一格式规则
├── .editorconfig                     编辑器缩进、换行和编码规则
├── .gitignore                        忽略构建产物、个人环境和大文件
├── .vscode/settings.json             VS Code 的 ROS 2、CMake 和格式化工作区设置
├── 启动命令.txt                       现场启动笔记；保留给操作者，不作为默认代码入口
├── config/                           所有运行 YAML 的统一编辑索引
├── src/                              ROS 2 源码和接口包
├── docs/                             项目说明、维护手册、历史证据和配图源文件
├── tools/                            结构检查、隔离测试、仿真资源和配图导出工具
└── build/ install/ log/              本机生成目录和验证证据，不作为源码入口
```

`build/`、`install/` 和 `log/` 里的文件属于本机产物或证据。修改功能时先看 `src/`、`config/` 和 `docs/`，不要直接改安装副本。

## 2. `config/`：从统一入口找到参数原文件

```text
config/
├── README.md                         参数分类、覆盖优先级和修改方法
├── mission/                          指向 src/fly_mission/config/
├── exploration/                      指向 src/exploration_planner/config/
├── pole_detector/                    指向 src/perception/pole_detector/config/
├── multi_pole_detector/              指向 src/perception/multi_pole_detector/config/
└── ring_detector/                    指向 src/perception/ring_detector/config/
```

符号链接目录不保存第二份参数。具体 YAML 以及每个键的归属见对应包的 `config/` 和 [参数索引](../config/README.md)。

## 3. `docs/`：说明、证据与配图源

```text
docs/
├── FILE_GUIDE.md                     本文件；从外到内解释当前源码文件
├── PROJECT_OVERVIEW.md               完整项目概览、接口、配置、构建和验证边界
├── MAINTENANCE.md                    故障定位、接口约定和后续改动规则
├── ARENA_20X20_SIM.md                20×20 m 仿真场景说明
├── ARMED_OBSTACLE_MAPPING.md         解锁后开始障碍建图的行为说明
├── VISION_RANGE_FILTER.md             视觉距离过滤的历史行为说明
├── NX_HOVER_DIAGNOSIS_20261014.md    历史 NX 停滞只读诊断
├── SPEED_TUNING_CLOSEOUT_20261014.md 历史提速任务收尾记录
├── THREAD_RECOVERY_20261014.md       历史会话恢复和证据交接
├── EXPLORATION_SPEED_TUNING_MEMORY.md 历史调参记忆；旧结论不能直接当作新版参数
├── history/                           历史场景和资料索引
│   ├── README.md                     历史资料边界和使用方式
│   └── arena-20x20-20261103.md       20×20 m 场景的历史记录
└── assets/                            可编辑配图源和提交预览
    ├── README.md                     图表编辑、导出和维护说明
    ├── architecture.mmd              模块架构 Mermaid 源文件，后续改图编辑它
    ├── architecture.svg              模块架构 GitHub 预览
    ├── config-layers.mmd              配置分层 Mermaid 源文件，后续改图编辑它
    └── config-layers.svg              配置分层 GitHub 预览
```

## 4. `src/`：ROS 2 源码总入口

```text
src/
├── acfly_interfaces/                 主控和探索之间的消息协议
├── acfly_config/                     默认 YAML、参数类型和范围校验
├── acfly_bringup/                    主控与探索的统一启动入口
├── fly_mission/                      起飞、探索交接、降落的单次任务主控
├── exploration_planner/              建图、覆盖、避障、返航和轨迹跟随
└── perception/                       点云公共库和三个几何检测器
```

每个 ROS 包都在包根放 `package.xml` 和 `CMakeLists.txt`。`package.xml` 声明包名和依赖，`CMakeLists.txt` 决定编译、安装、测试和配置文件的去向。下面按包继续展开。

### 4.1 `src/acfly_interfaces/`：消息协议

```text
src/acfly_interfaces/
├── package.xml                       消息包元数据和 rosidl 依赖
├── CMakeLists.txt                    生成并安装消息接口
└── msg/
    ├── ExplorationCommand.msg       探索发给主控的速度、巡航高度和任务标识
    └── ExplorationContext.msg       主控发给探索的起飞高度、巡航高度和任务上下文
```

消息字段和任务 Header 是跨进程契约。改动这里后必须同时重建主控、探索规划器和跨节点测试。

### 4.2 `src/acfly_config/`：参数基础层

```text
src/acfly_config/
├── package.xml                       参数工具包元数据
├── CMakeLists.txt                    编译库并安装头文件
├── include/acfly_config/
│   ├── defaults.hpp                  分类 YAML 默认值和参数声明接口
│   └── initialize.hpp                节点初始化、覆盖和校验入口
└── test/
    ├── config_probe.cpp              运行时读取参数的测试节点
    └── fixtures/
        ├── label.yaml                字符串类型覆盖夹具
        ├── override.yaml              数值和布尔覆盖夹具
        └── rate.yaml                 频率类型覆盖夹具
```

算法包使用这个基础层加载自己的 YAML；这里不放任务算法。

### 4.3 `src/acfly_bringup/`：统一启动层

```text
src/acfly_bringup/
├── package.xml                       启动包元数据和运行依赖
├── CMakeLists.txt                    安装 launch、测试和场景夹具
├── launch/survey.launch.py           同时启动主控和探索，并接收两份覆盖文件
└── test/
    ├── stationary_survey.yaml        跨节点测试的静止小场景参数
    ├── survey_cycle.py               ROS launch 测试入口
    └── survey_graph.py               节点、话题和生命周期图测试
```

### 4.4 `src/fly_mission/`：主控层

```text
src/fly_mission/
├── package.xml                       主控包元数据和 ROS 依赖
├── CMakeLists.txt                    编译主控、安装 YAML、注册测试
├── README.md                         主控行为、话题、参数和故障定位
├── config/
│   ├── mission.yaml                  起飞高度、返回点和任务放行选项
│   ├── flight_control.yaml           控制周期、速度上限、位置保持和容差
│   └── watchdogs.yaml                定位、飞控、指令和降落请求有效期
├── include/fly_mission/
│   ├── config.hpp                    主控配置结构和读取接口
│   ├── exploration_completion.hpp    探索完成 Header 的匹配规则
│   ├── exploration_link.hpp          goal、context、control、finished 链路
│   ├── flight_feedback.hpp           位姿、速度和飞控反馈缓存
│   ├── mavros_link.hpp               MAVROS 状态和控制输出接口
│   └── mission_node.hpp              主控状态机声明
├── src/
│   ├── main.cpp                      创建节点并启动单次任务
│   ├── config.cpp                    读取三类 YAML 并校验参数
│   ├── mission_node.cpp              状态切换、接管和生命周期
│   ├── takeoff.cpp                   地面自检、人工放行和起飞段
│   ├── survey.cpp                    探索交接和巡航高度保持
│   ├── landing.cpp                   完成回执后的降落、上锁和退出
│   ├── exploration_link.cpp          与探索规划器交换任务消息
│   ├── flight_feedback.cpp           订阅位姿、飞控状态和高度反馈
│   ├── mavros_link.cpp                发布 MAVROS setpoint、模式和降落请求
│   └── position_hold.cpp             无有效探索指令时的位置保持
└── tests/mission_lifecycle_test.cpp  起飞、探索、降落、接管和正常退出回归
```

主控对外只有一次任务生命周期。新增控制行为时优先拆到对应阶段文件，`main.cpp` 不承载算法。

### 4.5 `src/exploration_planner/`：探索算法层

```text
src/exploration_planner/
├── package.xml                       探索包元数据和依赖
├── CMakeLists.txt                    编译库、节点、配置、RViz 和测试
├── README.md                         探索职责、参数、接口、代码定位和验证
├── config/
│   ├── mapping.yaml                  场界、栅格、点云过滤和障碍累计
│   ├── coverage.yaml                 覆盖率、前沿评分和重规划
│   ├── motion.yaml                   速度、加减速、转弯和避障
│   ├── return.yaml                   返航、终点和停稳判据
│   └── runtime.yaml                  周期、传感器超时和诊断频率
├── rviz/exploration.rviz             探索路径、障碍、覆盖和诊断显示配置
├── include/exploration_planner/
│   ├── bezier.hpp                    曲线采样和曲率相关工具
│   ├── cloud_xyz.hpp                 PointCloud2 XYZ 字段读取
│   ├── coverage_planner.hpp          覆盖候选和覆盖路线规划接口
│   ├── global_planner.hpp            全局绕障路径接口
│   ├── grid_map.hpp                  栅格、覆盖和障碍状态
│   ├── home_approach.hpp             返回终点的安全入口策略
│   ├── obstacle_map.hpp              点云障碍累计和查询
│   ├── position_hold.hpp              定位或传感器异常时的零速/保持接口
│   ├── route_rejoin.hpp              受阻后重新接入覆盖路线
│   ├── sensor_freshness.hpp          定位、点云和仿真时钟新鲜度判定
│   ├── trajectory_tracker.hpp        路径跟随和速度输出
│   ├── types.hpp                     节点内部的阶段、路径和几何类型
│   └── velocity_slew.hpp              速度变化率限制
├── src/
│   ├── main.cpp                      创建并运行探索节点
│   ├── bezier.cpp                    曲线采样实现
│   ├── coverage_planner.cpp          覆盖候选和路径生成实现
│   ├── global_planner.cpp            全局 A* / 绕障实现
│   ├── grid_map.cpp                  栅格和覆盖记账实现
│   ├── obstacle_map.cpp              点云障碍地图实现
│   └── trajectory_tracker.cpp        轨迹跟随实现
├── src/node/
│   ├── exploration_node.hpp          ROS 节点状态和成员声明
│   ├── exploration_node.cpp          节点构造、周期和生命周期骨架
│   ├── configuration.cpp              参数读取和配置检查
│   ├── interfaces.cpp                ROS 发布、订阅和任务 Header 接口
│   ├── sensors.cpp                   里程计、点云、解锁和时钟输入
│   ├── mission.cpp                   任务阶段、覆盖/返航切换和完成回执
│   ├── coverage_search.cpp            前沿搜索和候选点评分
│   ├── coverage_path.cpp              覆盖路径验证、接续和重规划
│   ├── coverage_execution.cpp         覆盖路径执行和控制输出
│   ├── path_guard.cpp                 路径净空、预测制动和安全复核
│   ├── recovery.cpp                   受限脱困和恢复策略
│   ├── home.cpp                       返航状态和终点完成判定
│   ├── home_execution.cpp             返航路径跟随和末段停稳
│   └── telemetry.cpp                  诊断、覆盖率和 RViz 数据发布
└── tests/
    ├── planner_contract_test.cpp     节点回调、任务握手和关键生命周期契约
    ├── reference_gains.hpp            轨迹回归使用的冻结参考增益
    ├── coverage_blacklist_test.cpp   覆盖候选黑名单回归
    ├── frontier_observation_test.cpp 前沿观察和覆盖记账
    ├── global_path_escape_test.cpp   全局路径脱离障碍
    ├── global_path_progress_test.cpp 全局路径进展
    ├── global_path_strict_test.cpp   严格净空约束
    ├── global_required_path_test.cpp 必须存在的安全路径
    ├── obstacle_visibility_test.cpp  障碍可见性和累计
    ├── sensor_freshness_test.cpp     传感器新鲜度和仿真时钟
    ├── trajectory_flow_test.cpp      轨迹阶段流转
    ├── trajectory_overshoot_test.cpp 轨迹过冲回归
    └── turn_rounding_test.cpp        转弯圆滑和速度边界
```

探索算法的 ROS 节点逻辑放在 `src/node/`，可复用几何和规划算法放在 `src/` 与 `include/`。改参数先改 `config/`，改协议先改 `acfly_interfaces`。

### 4.6 `src/perception/`：感知层

```text
src/perception/
├── README.md                         三个检测器的输入、输出、配置和定位方法
├── tests/test_detectors.py           安装后用合成点云验证三个节点
├── acfly_perception_common/
│   ├── package.xml                   公共感知库元数据
│   ├── CMakeLists.txt                编译公共库和测试
│   ├── include/acfly_perception_common/
│   │   ├── cloud_window.hpp          点云滑窗缓存
│   │   ├── detector_node.hpp         检测节点公共 ROS 基类
│   │   ├── pole_fit.hpp              单杆几何拟合接口
│   │   └── settings.hpp              感知参数结构和读取接口
│   ├── src/
│   │   ├── cloud_window.cpp          点云格式检查、裁剪和滑窗
│   │   ├── detector_node.cpp         公共订阅、发布和参数初始化
│   │   ├── pole_fit.cpp              杆体中心、半径和方向拟合
│   │   └── settings.cpp              公共参数解析
│   └── tests/cloud_window_test.cpp   点云字段、端序、填充和滑窗回归
├── pole_detector/
│   ├── package.xml                   单杆检测包元数据
│   ├── CMakeLists.txt                编译节点、安装参数和 RViz
│   ├── config/input.yaml             输入点云、里程计和坐标系
│   ├── config/region.yaml            单杆三维裁剪范围
│   ├── config/detection.yaml         单杆滑窗、点数和半径阈值
│   ├── config/display.yaml           裁剪点云和 Marker 显示
│   ├── rviz/pole_detector.rviz       单杆检测 RViz 配置
│   └── src/
│       ├── pole_detector_node.cpp    单杆订阅、拟合和结果发布
│       ├── markers.hpp                单杆 Marker 声明
│       ├── markers.cpp                单杆 Marker 发布
│       └── （公共点云处理来自 acfly_perception_common）
├── multi_pole_detector/
│   ├── package.xml                   多杆检测包元数据
│   ├── CMakeLists.txt                编译节点、安装参数和 RViz
│   ├── config/input.yaml             输入点云、里程计和坐标系
│   ├── config/region.yaml            多杆三维裁剪范围
│   ├── config/detection.yaml         聚类、拟合和编号参数
│   ├── config/display.yaml           多杆 Marker 和裁剪点云显示
│   ├── rviz/multi_pole_detector.rviz 多杆检测 RViz 配置
│   └── src/
│       ├── multi_pole_detector_node.cpp 多杆订阅、聚类和结果发布
│       ├── clustering.hpp              水平距离聚类接口
│       ├── clustering.cpp               聚类实现
│       ├── pole_ids.hpp                  静态杆体编号接口
│       ├── pole_ids.cpp                  编号分配实现
│       ├── markers.hpp                   多杆 Marker 声明
│       └── markers.cpp                   多杆 Marker 发布
└── ring_detector/
    ├── package.xml                   圆环检测包元数据
    ├── CMakeLists.txt                编译节点、安装参数和 RViz
    ├── config/input.yaml             输入点云、里程计和坐标系
    ├── config/region.yaml            圆环三维裁剪范围
    ├── config/detection.yaml         圆环滑窗、拟合和半径参数
    ├── config/display.yaml           圆环 Marker 和裁剪点云显示
    ├── rviz/ring_detector.rviz       圆环检测 RViz 配置
    └── src/
        ├── ring_detector_node.cpp    圆环订阅、拟合和结果发布
        ├── ring_fit.hpp               平面圆环拟合接口
        ├── ring_fit.cpp               平面圆环拟合实现
        ├── markers.hpp                圆环 Marker 声明
        └── markers.cpp                圆环 Marker 发布
```

三个检测器各自拥有输入、区域、算法、显示四类 YAML；公共库只放可复用的点云和拟合基础，不把某一种检测器的参数写进去。

## 5. `tools/`：检查、验证与仿真辅助

```text
tools/
├── check_workspace.py                检查包名、目录、YAML 键和根配置索引
├── test_workspace.bash               复制源码到临时目录并隔离构建测试
├── audit_tuning_memory.py            审计历史调参记忆；新版布局下只作历史差异提示
├── render_diagrams.sh                用 Mermaid CLI 将 .mmd 导出为 .svg
├── setup/install_geographiclib_datasets.sh 安装 GeographicLib 数据集的现场辅助脚本
└── sim/worlds/arena_20x20.sdf        20×20 m 仿真场景和静态障碍物
```

工具脚本默认不启动飞控、不连接 NX。运行前先看脚本帮助和对应文档。

## 6. 修改顺序

1. 从 `README.md` 和本页确定所属包。
2. 运行参数只改对应 `config/*.yaml`，并在 `config/README.md` 更新索引。
3. ROS 消息先改 `src/acfly_interfaces/msg/`，再同步编译依赖方。
4. 节点算法改对应包的 `src/`、`include/` 和测试；`main.cpp` 只保留生命周期装配。
5. 配图改 `docs/assets/*.mmd`，运行 `tools/render_diagrams.sh` 更新预览。
6. 运行 `python3 tools/check_workspace.py`、受影响的 CTest 和 `git diff --check`。
7. 将行为、参数、验证边界和未部署事实写入对应 README 或 `docs/`，最后再交给维护者审阅和提交。
