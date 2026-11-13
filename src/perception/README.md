# 激光雷达几何检测

这组 ROS 2 节点从已配准点云测量静态杆体和圆环。收到点云后，节点用配置中的三维包围盒裁剪点，合并最近若干帧，再拟合几何形状。三个检测器可独立运行。

| 包 | 几何处理 | 默认测量话题与数组顺序 |
| --- | --- | --- |
| `pole_detector` | 估计单根杆的水平圆心和半径，取高度中值 | `/pole_detector/center`，`[x, y, z, yaw_error_deg, radius]` |
| `multi_pole_detector` | 按水平距离聚类、逐簇拟合杆体，并按静态地图位置分配编号 | `/multi_pole_detector/center`，每根 `[id, x, y, z, yaw_error_deg, radius]`，按编号升序 |
| `ring_detector` | 估计环面并在平面内拟合圆 | `/ring_detector/center`，`[x, y, z, yaw_error_deg, ring_yaw_deg]` |
| `acfly_perception_common` | 库。负责点云布局检查、裁剪、滑窗、单杆拟合及 ROS 参数读取 | 三个节点通过这个库共享输入处理 |

测量均为 `std_msgs/msg/Float64MultiArray`，坐标和半径单位为 m，角度单位为度。各包的 `src/markers.cpp` 负责 RViz 图形。多杆的聚类和编号分别在 `clustering.cpp` 和 `pole_ids.cpp`，圆环的平面拟合在 `ring_fit.cpp`。

## 配置与运行

每个检测包的 `config/` 有四份 ROS 参数文件，启动时自动加载，修改后重启相应节点。通过普通复制安装时应同步安装目录，通过 `--symlink-install` 安装时可直接修改源 YAML。

| 文件 | 调整内容 |
| --- | --- |
| `input.yaml` | 点云、里程计和测量话题，世界坐标系、队列深度和日志间隔 |
| `region.yaml` | 六个米制边界。每个轴的下界应小于上界 |
| `detection.yaml` | 滑窗帧数、点数、半径筛选，多杆的聚类与静态编号 |
| `display.yaml` | 裁剪点云和 Marker 话题、队列、尺寸、颜色 |

以单杆为例，已安装的工作区可直接运行以下命令：

```bash
ros2 run pole_detector pole_detector_node
rviz2 -d "$(ros2 pkg prefix --share pole_detector)/rviz/pole_detector.rviz"
```

也可以用 `--ros-args --params-file /absolute/path/site.yaml` 指定场地配置，或用 `-p accum_frames:=5` 做一次性调整。各节点沿用原包名和可执行名，标准 ROS 重映射可用于话题名。

## 定位问题

- 确认 `input.yaml` 中的点云话题收到 `sensor_msgs/msg/PointCloud2`。输入点云和里程计应以同一个世界坐标系发布，默认 `camera_init`。坐标系不同或 XYZ 字段不是单个 FLOAT32 时，日志会说明原因。节点接受带行填充、字段重排以及大端或小端的点云。
- 先在 RViz 查看 `cropped_cloud`。裁剪后的点少时检查 `region.yaml`，以及 `detection.yaml` 中的滑窗和点数。滑窗有效期按收到的帧数计算，适用于静态场地。
- 检测失败时没有新的 `center` 消息。使用测量值的程序应记录接收时间。多杆编号只在本次启动内有效。收到第一条有效里程计前，角度计算沿用初始位置和航向零值。
- 从源代码定位时，输入帧验证在 `acfly_perception_common/src/cloud_window.cpp`；节点发布和订阅在 `detector_node.cpp` 和各包的 `*_node.cpp`。改动话题名或坐标系后要同步修改 RViz 显示项。

启用 `BUILD_TESTING` 可用 CTest 运行共享点云格式与滑窗回归。完成隔离安装并加载该环境后，在工作区根目录运行 `python3 src/perception/tests/test_detectors.py -v`，将用合成点云验证三个节点的坐标、输出字段顺序、配置文件和命令行参数覆盖。
