# 仿真迭代交付记录

用户已要求结束持续迭代。本文件记录最终配置、已完成修复、验证证据和复现方式。

启动方式已按用户后续要求恢复为原来的分终端命令。已验证原样执行 `ros2 launch point_lio mapping_sim.launch.py` 和 `ros2 run exploration_planner exploration_planner_node` 可启动节点、启用完整机体系点云、建立唯一的稠密点云适配器并正确读取默认参数；匹配时间戳的点云转换检查通过。此项验证没有启动飞行任务，检查进程已关闭。

上一轮收尾版本已编译安装，并通过 `20260909_seed28_final_recheck` 完整仿真：探索、两道门、H 点降落及自动上锁均完成。该轮实际运行的规划器副本 SHA-256 为：`843291239cc9e12c4080604030944a9e42877a54077105135986dcbe24fb941d`。本轮结束后，已确认 PX4、Gazebo、MAVROS、Point-LIO、主控、规划器和相关适配器均已关闭。

最终一轮从起飞到 H 完成约 135.15 秒，探索及归航到交接点约 82.30 秒，走廊含入口转向至 H 约 47.10 秒。最小圆柱监控净空为 0.04659 m；独立部件投影净空为探索 0.39367 m、走廊 0.05260 m，无记录样本发生包络重叠。飞行效率仍因障碍布局而变化，不能据此宣称所有地图都达到最优路线。

## 已完成的主要修复

- 走廊使用 Point-LIO 原始稠密机体系点云，通过时间戳匹配的完整位姿转换到规划坐标系；探索地图继续使用原点云处理。
- 探索路径按连续线段进度校验，统一搜索、简化、平滑与执行检查；被障碍占用的探索目标使用安全观察位置。
- 普通候选搜索默认间隔 0.60 秒；紧急障碍、失效目标、任务切换和末端接力不受这项延迟限制。条带状态只在实际采用或明确放弃时提交。
- 不可达目标黑名单按裁剪后的实际飞行航点检查，修复同一角点不断重选的问题。
- 起点已在虚拟边界内缩区外时，允许沿边界完成经过逐段校验的单调入场绕行，不再局限于起点周围 0.6 m 方框；无候选目标时不再对当前位置重复空搜索。
- 超过 120° 的近反向换线不再构造短 Bézier 小钩，使用原有经过校验的路线和稳定转向控制；普通 45°、90° 过渡保留。
- 走廊连续对中，过门速度上限 0.30 m/s，出门后直行 1.00 m 再横移。预测横向漂移使用 0.40 秒，机头保持 -90°。
- 归航和 POI 使用独立沿线跟踪器；主路线及末端连接完整验障，删除近目标直切捷径。目标被占用时安全悬停，不能把代理点当成真正到达。
- 归航检查指令和实际速度的刹停投影，并在停稳持续阻塞后重新规划，避免安全检查自锁。
- 自动测试保存世界、参数、实际二进制及其哈希、真值轨迹、状态和结果。测试中使用过的统一 launch 保留为内部工具；用户启动方式已恢复为原来的分终端命令。

## 结果口径

同地图 seed 15 的场景文件哈希完全一致，旧版到探索交接点用时 98.60 秒，严格路径版本为 58.56 秒；探索近零指令时间从 33.60 秒降到 4.50 秒，路径采用从 229 次降到 21 次。这个对照验证的是中间已保存版本，不能当成所有地图都固定提速 40.6% 的保证。

同地图 seed 20 中，预测时间由 0.25 秒改为 0.40 秒后，第一道门释放后的横漂从 3.54 厘米降到 0.26 厘米，独立机体部件最小投影净空从 3.51 厘米提高到 4.72 厘米；走廊用时增加约 0.76 秒，两次对中都没有持续 0.4 秒以上的近零指令段。

seed 22 的交接点位于柱子实体内，seed 25 的交接点距柱面仅约 0.266 m。已将这类场景保留为阻塞回归，正确结果是安全悬停并报告 `required_goal_blocked`，不能完成该不可达任务，也不会自动篡改目标或地图。

近掉头修复的固定闭环场景使用同样的动力学响应和初始速度：旧小钩路径用时 27.54 秒，新路径 14.88 秒；实际速度低于 0.05 m/s 的时间从 16.62 秒降到 4.06 秒。普通 45°、90° Bézier 轨迹逐点保持一致。该结果属于复现场景回归，不是全任务耗时。

首次 seed 26 的制动自锁属于进度失败；修复后同地图已完成归航、两道门、H 点降落及自动上锁，独立机体部件最小净空为探索 0.394 m、走廊 0.055 m。seed 27 的正式默认配置也完成了全程。

完整逐轮数据见 `sim_runs/comparisons/iteration_summary.md`、`iteration_summary.json`。圆柱监控指标与按 SDF 旋翼扫掠、机身及实际姿态计算的部件净空分开列出；两者都是离线几何指标，不是接触传感器或连续时间碰撞证明。历史失败、阻塞和人工中止记录均保留。

## 检查状态

编译和安装完成，19 个功能 CTest 目标及 6 个稠密点云适配器测试均通过；Python 语法、flake8、cppcheck、pep257、xmllint 和对项目 CMake 源文件的检查通过。

全包 `lint_cmake` 仍扫描到包内历史 `build/` 生成文件并报错，实际项目 CMakeLists 检查通过。全包 `uncrustify` 风格检查未通过，包含既有源码风格和新增测试的格式差异；没有进行大范围自动格式化，也没有禁用检查。功能验证与风格检查结果分开记录。

## 正式启动方式

按用户原来的命令分终端启动，不需要替换 launch 命令或额外传入参数文件。

PX4 / Gazebo：

```bash
cd ~/uav_sim_env/PX4-Autopilot && PX4_GZ_MODEL_POSE="-4,0,0,0,0,0" PX4_GZ_WORLD=edc_arena make px4_sitl gz_x500_lidar_3d
```

随机四个障碍物：

```bash
cd ~/uav_sim_env
python3 randomize_arena.py 4
```

生成文件的变化会在 Gazebo 下次加载该世界时生效。

Point-LIO：

```bash
ros2 launch point_lio mapping_sim.launch.py
```

MAVROS：

```bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

主控程序：

```bash
ros2 run fly_mission fly_mission_node
```

探索程序：

```bash
ros2 run exploration_planner exploration_planner_node
```

OFFBOARD：

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

已有的 `mapping_sim.launch.py` 默认开启稠密机体系点云并启动走廊适配器，规划器直接使用头文件中的已验证默认参数。原命令即可使用走廊优化，轨迹窗口仍包含走廊。

`competition_sim.launch.py` 保留为可选内部测试工具，已有试验仍按其实际启动方式记录；它不再作为用户正式启动入口，也不要与上面的独立节点重复运行。

## 参数与文件

- 入口、H 点和探索交接点：`src/fly_mission/include/fly_mission/params.hpp`。当前磁盘配置为交接点 `(7.0,4.25)`、入口 `(8.1,4.25)`、H `(8.1,-4.0)`，均为 camera_init 坐标。
- 算法及默认控制变量：`src/exploration_planner/include/exploration_planner/params.hpp`。已验证的仿真设置直接作为默认值，包括完整机宽 0.64 m、对中容差 0.015 m、过门速度 0.30 m/s、横漂预测时间 0.40 秒和出门直行 1.00 m。
- Point-LIO 启动扩展：`/home/yk/ws_point/src/Point-LIO_ROS2/launch/mapping_sim.launch.py`。默认开启稠密点云和适配器，保留原来的启动命令。
- ROS 参数覆盖为可选功能；`tools/sim/profiles/physical_corridor.yaml` 保留供实验使用，正式启动不必加载它。修改头文件后需重新编译对应包。

```bash
cd /home/yk/acfly_ws
colcon build --packages-select exploration_planner fly_mission --cmake-args -DBUILD_TESTING=ON
```

下述自动复现命令属于内部测试方式，保留 seed 28 历史试验使用的 `--stack-launch` 模式。

自动复现某一轮：

```bash
/usr/bin/python3 /home/yk/acfly_ws/tools/sim/run_trial.py \
  --seed 28 --duration 480 --domain 106 --headless --stack-launch \
  --output /home/yk/acfly_ws/sim_runs/manual_seed28
```

输出目录必须是新的，单次运行会在结束后清理自己的仿真进程。当前交付不设置定时器或后台持续迭代任务。
