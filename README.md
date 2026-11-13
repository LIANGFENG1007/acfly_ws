# ACFly 空间介绍

## 文件导览

<details open>
<summary><strong><code>acfly_ws/</code></strong></summary>
<ul>
  <li><details>
    <summary><code>.vscode/</code> VS Code 工作区设置</summary>
    <ul>
      <li><code>settings.json</code> — 设置 CMake Tools 的源码目录，当前指向旧台架工程</li>
    </ul>
  </details></li>
  <li><details>
    <summary><code>config/</code> 统一参数入口；链接指向对应包的 config 目录</summary>
    <ul>
      <li><details>
        <summary><code>exploration/</code> → <code>../src/exploration_planner/config</code></summary>
        <ul>
          <li><code>coverage.yaml</code> — 覆盖率、前沿评分和重规划</li>
          <li><code>mapping.yaml</code> — 场界、栅格、点云过滤和障碍累计</li>
          <li><code>motion.yaml</code> — 速度、加减速、转弯和避障</li>
          <li><code>return.yaml</code> — 返航、终点和停稳判据</li>
          <li><code>runtime.yaml</code> — 周期、传感器超时和诊断频率</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>mission/</code> → <code>../src/fly_mission/config</code></summary>
        <ul>
          <li><code>flight_control.yaml</code> — 控制周期、速度上限、位置保持和容差</li>
          <li><code>mission.yaml</code> — 起飞高度、返回点和任务放行选项</li>
          <li><code>watchdogs.yaml</code> — 定位、飞控、指令和降落请求有效期</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>multi_pole_detector/</code> → <code>../src/perception/multi_pole_detector/config</code></summary>
        <ul>
          <li><code>detection.yaml</code> — 聚类、拟合和编号参数</li>
          <li><code>display.yaml</code> — 多杆 Marker 和裁剪点云显示</li>
          <li><code>input.yaml</code> — 设置多杆检测的点云、里程计、测量话题和坐标系</li>
          <li><code>region.yaml</code> — 多杆三维裁剪范围</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>pole_detector/</code> → <code>../src/perception/pole_detector/config</code></summary>
        <ul>
          <li><code>detection.yaml</code> — 单杆滑窗、点数和半径阈值</li>
          <li><code>display.yaml</code> — 裁剪点云和 Marker 显示</li>
          <li><code>input.yaml</code> — 设置单杆检测的点云、里程计、测量话题和坐标系</li>
          <li><code>region.yaml</code> — 单杆三维裁剪范围</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>ring_detector/</code> → <code>../src/perception/ring_detector/config</code></summary>
        <ul>
          <li><code>detection.yaml</code> — 圆环滑窗、拟合和半径参数</li>
          <li><code>display.yaml</code> — 圆环 Marker 和裁剪点云显示</li>
          <li><code>input.yaml</code> — 设置圆环检测的点云、里程计、测量话题和坐标系</li>
          <li><code>region.yaml</code> — 圆环三维裁剪范围</li>
        </ul>
      </details></li>
      <li><code>README.md</code> — 参数分类、覆盖优先级和修改方法</li>
    </ul>
  </details></li>
  <li><details>
    <summary><code>docs/</code> 项目说明、维护手册、配图与历史记录</summary>
    <ul>
      <li><details>
        <summary><code>assets/</code> 架构图与配置图的可编辑源文件和预览图</summary>
        <ul>
          <li><code>architecture.mmd</code> — 模块架构 Mermaid 源文件，后续改图编辑它</li>
          <li><code>architecture.svg</code> — 模块架构 GitHub 预览</li>
          <li><code>config-layers.mmd</code> — 配置分层 Mermaid 源文件，后续改图编辑它</li>
          <li><code>config-layers.svg</code> — 配置分层 GitHub 预览</li>
          <li><code>README.md</code> — 图表编辑、导出和维护说明</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>history/</code> 旧版本的场景说明和归档索引</summary>
        <ul>
          <li><code>arena-20x20-20261103.md</code> — 20×20 m 场景的历史记录</li>
          <li><code>README.md</code> — 历史资料边界和使用方式</li>
        </ul>
      </details></li>
      <li><code>ARENA_20X20_SIM.md</code> — 仿真场地坐标、外部依赖和测绘配置的对应关系</li>
      <li><code>ARMED_OBSTACLE_MAPPING.md</code> — 解锁后开始障碍建图的行为说明</li>
      <li><code>EXPLORATION_SPEED_TUNING_MEMORY.md</code> — 历次调参与验证的交接记录，区分旧版结论和当前结构</li>
      <li><code>FILE_GUIDE.md</code> — 按维护任务查找入口：配置、接口、节点、配图和验证</li>
      <li><code>MAINTENANCE.md</code> — 故障定位、接口约定和后续改动规则</li>
      <li><code>NX_HOVER_DIAGNOSIS_20261014.md</code> — 历史 NX 停滞只读诊断</li>
      <li><code>PROJECT_OVERVIEW.md</code> — 模块职责、话题发布和订阅关系、启动方式及验证范围</li>
      <li><code>SPEED_TUNING_CLOSEOUT_20261014.md</code> — 历史提速任务收尾记录</li>
      <li><code>THREAD_RECOVERY_20261014.md</code> — 历史会话恢复和证据交接</li>
      <li><code>VISION_RANGE_FILTER.md</code> — 视觉距离过滤的历史行为说明</li>
    </ul>
  </details></li>
  <li><details>
    <summary><code>src/</code> ROS 2 功能包、公共库和消息定义</summary>
    <ul>
      <li><details>
        <summary><code>acfly_bringup/</code> 主控与探索的统一启动入口</summary>
        <ul>
          <li><details>
            <summary><code>launch/</code> ROS 2 启动文件</summary>
            <ul>
              <li><code>survey.launch.py</code> — 启动主控和规划器，分别传入各自的参数覆盖文件</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>test/</code> 测试程序与输入样本</summary>
            <ul>
              <li><code>stationary_survey.yaml</code> — 调低测试场景的覆盖门槛，供静止任务回归使用</li>
              <li><code>survey_cycle.py</code> — 用模拟传感器和飞控服务验证任务从放行到降落退出</li>
              <li><code>survey_graph.py</code> — 核对四个任务话题的收发节点、消息类型与 QoS</li>
            </ul>
          </details></li>
          <li><code>CMakeLists.txt</code> — 安装启动文件，注册通信接口和任务周期两项测试</li>
          <li><code>package.xml</code> — 启动包元数据和运行依赖</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>acfly_config/</code> 加载分类 YAML，检查参数类型和键名</summary>
        <ul>
          <li><details>
            <summary><code>include/</code> 供其他源文件引用的头文件</summary>
            <ul>
              <li><details>
                <summary><code>acfly_config/</code> acfly_config 的公共接口</summary>
                <ul>
                  <li><code>defaults.hpp</code> — 读取分类 YAML，声明参数，报告重复、缺失或未使用的键</li>
                  <li><code>initialize.hpp</code> — 初始化 ROS 前，检查多个覆盖来源对同一参数的类型是否一致</li>
                </ul>
              </details></li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>test/</code> 测试程序与输入样本</summary>
            <ul>
              <li><details>
                <summary><code>fixtures/</code> 数据样本，供 config_probe 验证参数加载</summary>
                <ul>
                  <li><code>label.yaml</code> — 提供测试字符串 label，验证多个分类文件可以一起加载</li>
                  <li><code>override.yaml</code> — 覆盖测试用 sample_rate，用于比较文件与命令行的优先级</li>
                  <li><code>rate.yaml</code> — 提供整数 sample_rate 的测试默认值</li>
                </ul>
              </details></li>
              <li><code>config_probe.cpp</code> — 验证默认值、覆盖优先级和错误参数的报错行为</li>
            </ul>
          </details></li>
          <li><code>CMakeLists.txt</code> — 导出公共头文件，构建参数测试程序并注册测试</li>
          <li><code>package.xml</code> — 参数工具包元数据</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>acfly_interfaces/</code> 主控和探索之间的消息协议</summary>
        <ul>
          <li><details>
            <summary><code>msg/</code> ROS 自定义消息</summary>
            <ul>
              <li><code>ExplorationCommand.msg</code> — 规划器发给主控：任务标识、机体速度、航向角速度、高度及垂直限速</li>
              <li><code>ExplorationContext.msg</code> — 主控发给规划器：任务标识、起飞点高度、巡航高度和高度容差</li>
            </ul>
          </details></li>
          <li><code>CMakeLists.txt</code> — 生成并安装消息接口</li>
          <li><code>package.xml</code> — 消息包元数据和 rosidl 依赖</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>exploration_planner/</code> 建图、覆盖、避障、返航和轨迹跟随</summary>
        <ul>
          <li><details>
            <summary><code>config/</code> 按功能拆分的节点默认参数</summary>
            <ul>
              <li><code>coverage.yaml</code> — 覆盖率、前沿评分和重规划</li>
              <li><code>mapping.yaml</code> — 场界、栅格、点云过滤和障碍累计</li>
              <li><code>motion.yaml</code> — 速度、加减速、转弯和避障</li>
              <li><code>return.yaml</code> — 返航、终点和停稳判据</li>
              <li><code>runtime.yaml</code> — 周期、传感器超时和诊断频率</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>include/</code> 供其他源文件引用的头文件</summary>
            <ul>
              <li><details>
                <summary><code>exploration_planner/</code> exploration_planner 的公共接口</summary>
                <ul>
                  <li><code>bezier.hpp</code> — 定义轨迹采样点、曲率及折线路径平滑接口</li>
                  <li><code>cloud_xyz.hpp</code> — 校验 PointCloud2 的字段、行跨度和字节序并读取 XYZ</li>
                  <li><code>coverage_planner.hpp</code> — 定义覆盖候选、观察点选择及覆盖路径接口</li>
                  <li><code>global_planner.hpp</code> — 全局绕障路径接口</li>
                  <li><code>grid_map.hpp</code> — 定义大小两层栅格、扫描标记和覆盖率查询接口</li>
                  <li><code>home_approach.hpp</code> — 返回终点的安全入口策略</li>
                  <li><code>obstacle_map.hpp</code> — 点云障碍累计和查询</li>
                  <li><code>position_hold.hpp</code> — 转向时保持水平位置，并在带速进入时处理惯性滑行</li>
                  <li><code>route_rejoin.hpp</code> — 受阻后重新接入覆盖路线</li>
                  <li><code>sensor_freshness.hpp</code> — 用源时间戳判别新帧，用本机接收时间判断数据是否过期</li>
                  <li><code>trajectory_tracker.hpp</code> — 路径跟随和速度输出</li>
                  <li><code>types.hpp</code> — 定义二维点、折线路径和圆形障碍物等共享类型</li>
                  <li><code>velocity_slew.hpp</code> — 速度变化率限制</li>
                </ul>
              </details></li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>rviz/</code> 节点的 RViz 显示配置</summary>
            <ul>
              <li><code>exploration.rviz</code> — RViz 中的里程计、当前路径、障碍点云和场界显示项</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>src/</code> 算法和节点实现</summary>
            <ul>
              <li><details>
                <summary><code>node/</code> 探索 ROS 节点：连接传感器、规划算法与任务接口</summary>
                <ul>
                  <li><code>configuration.cpp</code> — 参数读取和配置检查</li>
                  <li><code>coverage_execution.cpp</code> — 覆盖路径执行和控制输出</li>
                  <li><code>coverage_path.cpp</code> — 检查并接纳新覆盖路线，处理路径拼接、平滑与观察结束</li>
                  <li><code>coverage_search.cpp</code> — 选择探索目标，搜索可通行路线，必要时转向重新寻找</li>
                  <li><code>exploration_node.cpp</code> — 装配参数和通信接口，注册探索节点的周期回调</li>
                  <li><code>exploration_node.hpp</code> — ROS 节点状态和成员声明</li>
                  <li><code>home.cpp</code> — 选取返航入口，生成到达返回点的末段路线和目标速度</li>
                  <li><code>home_execution.cpp</code> — 执行返航指令，核对是否沿有效路径到达终点并停稳</li>
                  <li><code>interfaces.cpp</code> — ROS 发布、订阅和任务 Header 接口</li>
                  <li><code>mission.cpp</code> — 任务阶段、覆盖/返航切换和完成回执</li>
                  <li><code>path_guard.cpp</code> — 路径净空、预测制动和安全复核</li>
                  <li><code>recovery.cpp</code> — 处理贴障后退、沿来路回退以及重新接入路线</li>
                  <li><code>sensors.cpp</code> — 里程计、点云、解锁和时钟输入</li>
                  <li><code>telemetry.cpp</code> — 发布覆盖率、当前路径、障碍点云、场界和诊断信息</li>
                </ul>
              </details></li>
              <li><code>bezier.cpp</code> — 平滑折线与拐角，按弧长采样并计算轨迹曲率</li>
              <li><code>coverage_planner.cpp</code> — 覆盖候选和路径生成实现</li>
              <li><code>global_planner.cpp</code> — 全局 A* / 绕障实现</li>
              <li><code>grid_map.cpp</code> — 标记扫描与遮挡区域，统计大小栅格的覆盖情况</li>
              <li><code>main.cpp</code> — 创建并运行探索节点</li>
              <li><code>obstacle_map.cpp</code> — 累计点云命中，聚类占据格并估计障碍物位置与半径</li>
              <li><code>trajectory_tracker.cpp</code> — 根据轨迹、位姿及速度估计，生成平移和转向指令</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>tests/</code> 回归测试</summary>
            <ul>
              <li><code>coverage_blacklist_test.cpp</code> — 验证排除受阻候选后仍能选出可用的未扫描区域</li>
              <li><code>frontier_observation_test.cpp</code> — 前沿观察和覆盖记账</li>
              <li><code>global_path_escape_test.cpp</code> — 验证从障碍物或场界余量不足的起点脱离时的路径约束</li>
              <li><code>global_path_progress_test.cpp</code> — 验证路径进度与前方净空判断，覆盖稀疏航点情形</li>
              <li><code>global_path_strict_test.cpp</code> — 验证严格净空下的绕障和拒绝不可行路线</li>
              <li><code>global_required_path_test.cpp</code> — 验证必达终点的路径、净空和简化后是否仍能到点</li>
              <li><code>obstacle_visibility_test.cpp</code> — 验证遮挡、最近回波和稀疏点云对可见区域的影响</li>
              <li><code>planner_contract_test.cpp</code> — 节点回调、任务握手和关键生命周期契约</li>
              <li><code>reference_gains.hpp</code> — 轨迹回归使用的冻结参考增益</li>
              <li><code>sensor_freshness_test.cpp</code> — 验证重复帧、时钟回退、超时与后续新帧恢复</li>
              <li><code>trajectory_flow_test.cpp</code> — 用连续拐弯场景检查到达、停车和重新对准航向的行为</li>
              <li><code>trajectory_overshoot_test.cpp</code> — 比较跟踪场景中的越线、反向修正和终点误差</li>
              <li><code>turn_rounding_test.cpp</code> — 检查圆角轨迹、曲率采样与转弯时的限速</li>
            </ul>
          </details></li>
          <li><code>CMakeLists.txt</code> — 构建规划器和测试，安装节点、分类参数及 RViz 配置</li>
          <li><code>package.xml</code> — 探索包元数据和依赖</li>
          <li><code>README.md</code> — 探索职责、参数、接口、代码定位和验证</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>fly_mission/</code> 起飞、探索交接、降落的单次任务主控</summary>
        <ul>
          <li><details>
            <summary><code>config/</code> 按功能拆分的节点默认参数</summary>
            <ul>
              <li><code>flight_control.yaml</code> — 控制周期、速度上限、位置保持和容差</li>
              <li><code>mission.yaml</code> — 起飞高度、返回点和任务放行选项</li>
              <li><code>watchdogs.yaml</code> — 定位、飞控、指令和降落请求有效期</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>include/</code> 供其他源文件引用的头文件</summary>
            <ul>
              <li><details>
                <summary><code>fly_mission/</code> fly_mission 的公共接口</summary>
                <ul>
                  <li><code>config.hpp</code> — 主控配置结构和读取接口</li>
                  <li><code>exploration_completion.hpp</code> — 按坐标系和时间戳比对完成回执与本次任务</li>
                  <li><code>exploration_link.hpp</code> — 定义目标与上下文的发布接口、控制指令与完成回执的接收接口</li>
                  <li><code>flight_feedback.hpp</code> — 定义位姿、速度及里程计反馈的读取接口</li>
                  <li><code>mavros_link.hpp</code> — MAVROS 状态和控制输出接口</li>
                  <li><code>mission_node.hpp</code> — 主控状态机声明</li>
                </ul>
              </details></li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>src/</code> 算法和节点实现</summary>
            <ul>
              <li><code>config.cpp</code> — 读取三类 YAML 并校验参数</li>
              <li><code>exploration_link.cpp</code> — 与探索规划器交换任务消息</li>
              <li><code>flight_feedback.cpp</code> — 校验里程计时间戳和位姿，估算平滑速度并判断定位是否过期</li>
              <li><code>landing.cpp</code> — 完成回执后的降落、上锁和退出</li>
              <li><code>main.cpp</code> — 创建节点并启动单次任务</li>
              <li><code>mavros_link.cpp</code> — 接收飞控状态和里程计，发布速度指令并请求降落模式</li>
              <li><code>mission_node.cpp</code> — 状态切换、接管和生命周期</li>
              <li><code>position_hold.cpp</code> — 根据位置、高度和航向误差计算保持指令，限制速度幅度</li>
              <li><code>survey.cpp</code> — 探索交接和巡航高度保持</li>
              <li><code>takeoff.cpp</code> — 地面自检、人工放行和起飞段</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>tests/</code> 回归测试</summary>
            <ul>
              <li><code>mission_lifecycle_test.cpp</code> — 起飞、探索、降落、接管和正常退出回归</li>
            </ul>
          </details></li>
          <li><code>CMakeLists.txt</code> — 编译主控、安装 YAML、注册测试</li>
          <li><code>package.xml</code> — 主控包元数据和 ROS 依赖</li>
          <li><code>README.md</code> — 主控行为、话题、参数和故障定位</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>perception/</code> 点云公共库和三个几何检测器</summary>
        <ul>
          <li><details>
            <summary><code>acfly_perception_common/</code> 点云裁剪、滑窗、杆体拟合和参数读取公共库</summary>
            <ul>
              <li><details>
                <summary><code>include/</code> 供其他源文件引用的头文件</summary>
                <ul>
                  <li><details>
                    <summary><code>acfly_perception_common/</code> acfly_perception_common 的公共接口</summary>
                    <ul>
                      <li><code>cloud_window.hpp</code> — 点云滑窗缓存</li>
                      <li><code>detector_node.hpp</code> — 检测节点公共 ROS 基类</li>
                      <li><code>pole_fit.hpp</code> — 声明杆体几何结果，以及整组点或指定点簇的拟合接口</li>
                      <li><code>settings.hpp</code> — 感知参数结构和读取接口</li>
                    </ul>
                  </details></li>
                </ul>
              </details></li>
              <li><details>
                <summary><code>src/</code> 算法和节点实现</summary>
                <ul>
                  <li><code>cloud_window.cpp</code> — 点云格式检查、裁剪和滑窗</li>
                  <li><code>detector_node.cpp</code> — 订阅点云与里程计、检查坐标系，发布裁剪点云并调用各检测算法</li>
                  <li><code>pole_fit.cpp</code> — 从水平投影估算杆体圆心和半径，以点的高度中值作为中心高度</li>
                  <li><code>settings.cpp</code> — 读取检测器配置，校验数值、数量、颜色和文本参数</li>
                </ul>
              </details></li>
              <li><details>
                <summary><code>tests/</code> 回归测试</summary>
                <ul>
                  <li><code>cloud_window_test.cpp</code> — 点云字段、端序、填充和滑窗回归</li>
                </ul>
              </details></li>
              <li><code>CMakeLists.txt</code> — 编译公共库和测试</li>
              <li><code>package.xml</code> — 公共感知库元数据</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>multi_pole_detector/</code> 从多个点簇拟合杆体，并为静态杆体分配编号</summary>
            <ul>
              <li><details>
                <summary><code>config/</code> 按功能拆分的节点默认参数</summary>
                <ul>
                  <li><code>detection.yaml</code> — 聚类、拟合和编号参数</li>
                  <li><code>display.yaml</code> — 多杆 Marker 和裁剪点云显示</li>
                  <li><code>input.yaml</code> — 设置多杆检测的点云、里程计、测量话题和坐标系</li>
                  <li><code>region.yaml</code> — 多杆三维裁剪范围</li>
                </ul>
              </details></li>
              <li><details>
                <summary><code>rviz/</code> 节点的 RViz 显示配置</summary>
                <ul>
                  <li><code>multi_pole_detector.rviz</code> — 多杆检测 RViz 配置</li>
                </ul>
              </details></li>
              <li><details>
                <summary><code>src/</code> 算法和节点实现</summary>
                <ul>
                  <li><code>clustering.cpp</code> — 聚类实现</li>
                  <li><code>clustering.hpp</code> — 水平距离聚类接口</li>
                  <li><code>markers.cpp</code> — 生成杆体中心和编号文字的 RViz 标记</li>
                  <li><code>markers.hpp</code> — 声明多杆的显示参数及 RViz 标记生成接口</li>
                  <li><code>multi_pole_detector_node.cpp</code> — 多杆订阅、聚类和结果发布</li>
                  <li><code>pole_ids.cpp</code> — 编号分配实现</li>
                  <li><code>pole_ids.hpp</code> — 静态杆体编号接口</li>
                </ul>
              </details></li>
              <li><code>CMakeLists.txt</code> — 编译节点、安装参数和 RViz</li>
              <li><code>package.xml</code> — 多杆检测包元数据</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>pole_detector/</code> 从指定区域的点云测量单根杆体</summary>
            <ul>
              <li><details>
                <summary><code>config/</code> 按功能拆分的节点默认参数</summary>
                <ul>
                  <li><code>detection.yaml</code> — 单杆滑窗、点数和半径阈值</li>
                  <li><code>display.yaml</code> — 裁剪点云和 Marker 显示</li>
                  <li><code>input.yaml</code> — 设置单杆检测的点云、里程计、测量话题和坐标系</li>
                  <li><code>region.yaml</code> — 单杆三维裁剪范围</li>
                </ul>
              </details></li>
              <li><details>
                <summary><code>rviz/</code> 节点的 RViz 显示配置</summary>
                <ul>
                  <li><code>pole_detector.rviz</code> — 单杆检测 RViz 配置</li>
                </ul>
              </details></li>
              <li><details>
                <summary><code>src/</code> 算法和节点实现</summary>
                <ul>
                  <li><code>markers.cpp</code> — 生成中心点和圆柱的 RViz 标记</li>
                  <li><code>markers.hpp</code> — 声明单杆的显示参数及 RViz 标记生成接口</li>
                  <li><code>pole_detector_node.cpp</code> — 单杆订阅、拟合和结果发布</li>
                </ul>
              </details></li>
              <li><code>CMakeLists.txt</code> — 编译节点、安装参数和 RViz</li>
              <li><code>package.xml</code> — 单杆检测包元数据</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>ring_detector/</code> 从指定区域的点云拟合圆环及其朝向</summary>
            <ul>
              <li><details>
                <summary><code>config/</code> 按功能拆分的节点默认参数</summary>
                <ul>
                  <li><code>detection.yaml</code> — 圆环滑窗、拟合和半径参数</li>
                  <li><code>display.yaml</code> — 圆环 Marker 和裁剪点云显示</li>
                  <li><code>input.yaml</code> — 设置圆环检测的点云、里程计、测量话题和坐标系</li>
                  <li><code>region.yaml</code> — 圆环三维裁剪范围</li>
                </ul>
              </details></li>
              <li><details>
                <summary><code>rviz/</code> 节点的 RViz 显示配置</summary>
                <ul>
                  <li><code>ring_detector.rviz</code> — 圆环检测 RViz 配置</li>
                </ul>
              </details></li>
              <li><details>
                <summary><code>src/</code> 算法和节点实现</summary>
                <ul>
                  <li><code>markers.cpp</code> — 生成圆心、轮廓和法向箭头的 RViz 标记</li>
                  <li><code>markers.hpp</code> — 声明圆环的显示参数及 RViz 标记生成接口</li>
                  <li><code>ring_detector_node.cpp</code> — 圆环订阅、拟合和结果发布</li>
                  <li><code>ring_fit.cpp</code> — 平面圆环拟合实现</li>
                  <li><code>ring_fit.hpp</code> — 平面圆环拟合接口</li>
                </ul>
              </details></li>
              <li><code>CMakeLists.txt</code> — 编译节点、安装参数和 RViz</li>
              <li><code>package.xml</code> — 圆环检测包元数据</li>
            </ul>
          </details></li>
          <li><details>
            <summary><code>tests/</code> 回归测试</summary>
            <ul>
              <li><code>test_detectors.py</code> — 安装后用合成点云验证三个节点</li>
            </ul>
          </details></li>
          <li><code>README.md</code> — 三个检测器的输入、输出、配置和定位方法</li>
        </ul>
      </details></li>
    </ul>
  </details></li>
  <li><details>
    <summary><code>tools/</code> 仓库检查、隔离测试、仿真场景和文档工具</summary>
    <ul>
      <li><details>
        <summary><code>setup/</code> 现场环境依赖的安装脚本</summary>
        <ul>
          <li><code>install_geographiclib_datasets.sh</code> — 安装 GeographicLib 数据集的现场辅助脚本</li>
        </ul>
      </details></li>
      <li><details>
        <summary><code>sim/</code> 仿真资源</summary>
        <ul>
          <li><details>
            <summary><code>worlds/</code> Gazebo 场景模型</summary>
            <ul>
              <li><code>arena_20x20.sdf</code> — 20×20 m 仿真场景和静态障碍物</li>
            </ul>
          </details></li>
        </ul>
      </details></li>
      <li><code>audit_tuning_memory.py</code> — 对照旧版源码快照与实验记录，报告缺失或已变化的证据</li>
      <li><code>check_workspace.py</code> — 检查包名、目录、YAML 键和根配置索引</li>
      <li><code>render_diagrams.sh</code> — 用 Mermaid CLI 将 .mmd 导出为 .svg</li>
      <li><code>test_workspace.bash</code> — 复制源码到临时目录并隔离构建测试</li>
    </ul>
  </details></li>
  <li><code>.clang-format</code> — C/C++ 排版规则：缩进、行宽、指针和头文件顺序</li>
  <li><code>.editorconfig</code> — 编辑器通用规则：字符编码、换行、缩进与行末空白</li>
  <li><code>.gitignore</code> — 指定 Git 忽略的构建产物、录包和本机文件</li>
  <li><code>AGENTS.md</code> — 本仓库的协作约定、任务交接和证据核对要求</li>
  <li><code>README.md</code> — 项目首页：文件导览、构建、启动和验证命令</li>
  <li><code>启动命令.txt</code> — 当前启动文档入口及旧版现场笔记的归档位置</li>
</ul>
</details>

`build/` 存放本机构建与实验记录，`install/` 存放安装结果，`log/` 存放日志；这三个目录均不随仓库分发。

## 项目简介

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
