# 参数索引

这个目录里的入口是指向各 ROS 包 config 目录的相对符号链接。通过这里编辑和通过包目录编辑是同一份文件。默认值只由读取它的节点拥有；多个节点都有的名称也不会互相覆盖。

| 调整什么 | 编辑哪一份 |
|---|---|
| 起飞相对高度、返回点、起飞放行选项 | [mission/mission.yaml](mission/mission.yaml) |
| 主控周期、悬停控制、速度上限及容差 | [mission/flight_control.yaml](mission/flight_control.yaml) |
| 定位和控制消息有效期、接管与降落等待 | [mission/watchdogs.yaml](mission/watchdogs.yaml) |
| 场地边界、栅格、雷达视野、障碍累计和机体半径 | [exploration/mapping.yaml](exploration/mapping.yaml) |
| 探索终止覆盖率、前沿选点和重规划 | [exploration/coverage.yaml](exploration/coverage.yaml) |
| 跟随速度、加减速、转弯和避障 | [exploration/motion.yaml](exploration/motion.yaml) |
| 返回终点的路线、减速、到达和停止判据 | [exploration/return.yaml](exploration/return.yaml) |
| 探索周期、传感器超时、诊断频率 | [exploration/runtime.yaml](exploration/runtime.yaml) |
| 单杆、多杆、圆环检测 | [pole_detector](pole_detector/)、[multi_pole_detector](multi_pole_detector/)、[ring_detector](ring_detector/)；每个目录内 input.yaml 管话题与坐标系，region.yaml 管裁剪范围，detection.yaml 管算法，display.yaml 管显示 |

参数在进程启动时读取。推荐使用符号链接安装后改源码 YAML、重启对应节点；普通复制安装应更新安装空间，或用 ROS 参数文件显式覆盖。可用 `ros2 pkg prefix --share fly_mission` 检查实际读取的包，`ros2 param dump /fly_mission_node` 查看运行中的有效值。优先级由低到高为包内默认文件、使用者按顺序传入的 ROS 命令行参数；同一键且类型一致时后写覆盖先写。主控与探索的整数和小数类型按 YAML 区分（如 1 与 1.0）；在多份覆盖来源中不要改变同一键的类型。

需要单次场景覆盖时，给目标节点单独建文件，根键写具体节点名，避免把一个包的参数传给另一个包：

```yaml
/exploration_planner_node:
  ros__parameters:
    search_coverage: 0.95
```

程序的运行状态和算法中间值由节点维护。ROS 消息字段、枚举值、坐标约定和数学常数由接口或代码定义，不作为现场调参项。不要在测试夹具、历史参数或 install/ 复制件里改项目默认值。
