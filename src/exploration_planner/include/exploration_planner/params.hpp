// ============================================================================
//  params.hpp  ── 探索算法可调常量默认值
//  说明：这些是"编译期默认值"。运行期都通过 ROS 参数(declare_parameter)覆盖，
//        ros2 run ... --ros-args -p name:=value 即可改，不必重编译。
// ============================================================================

#pragma once

#include <cmath>

namespace exploration {
namespace params {

// ---------------------------------------------------------------------------
// ★ 场地四面墙范围（SLAM/camera_init 系，原点=起飞点）★  ← 改这里
//   这是"飞机起飞点为原点"算出来的场地边界。起飞点不在场地中心时要换算：
//   边界(SLAM) = 边界(世界) - 起飞点(世界)。
//   例：起飞点世界 (-4,0)，10x10 场地世界 x,y ∈ [-5,5]
//       → SLAM x ∈ [-5-(-4), 5-(-4)] = [-1, 9]，y ∈ [-5, 5]
// ---------------------------------------------------------------------------
inline constexpr double FIELD_MIN_X = -0.5;
inline constexpr double FIELD_MIN_Y = -2.0;
inline constexpr double FIELD_MAX_X =  4.4;
inline constexpr double FIELD_MAX_Y =  2.0;

// ★ 离墙安全内缩 (m) ★  ← 改这里
//   牛耕车道/掉头点离四面墙至少留这么远，飞机绝不贴墙飞（防撞）。
//   墙根的格子不靠飞机走过去，而是靠 3m 视野"看"过去覆盖。
//   只要 WALL_MARGIN < 视野单侧可靠覆盖宽度(≈2.6m)，墙根就能被看到。
inline constexpr double WALL_MARGIN = 1.00;

// ---------------------------------------------------------------------------
// 双层栅格
//   大格"探索量"边长 0.5m；小格"完程度"边长 0.05m（每大格 10x10=100 小格）
//   大格内被扫小格占比 >= COVERAGE_THRESH 即判定该大格"已探索"
// ---------------------------------------------------------------------------
inline constexpr double BIG_CELL        = 0.50;   // 大格边长 (m)
inline constexpr double SMALL_CELL      = 0.05;   // 小格边长 (m)
inline constexpr double COVERAGE_THRESH = 0.90;   // 大格完成阈值（占比）

// ---------------------------------------------------------------------------
// 可视区（几何模型，第一版不用点云）：前方 150° 开角、3m 半径的扇形
//   开角 120° → 半开角 ±60°
// ---------------------------------------------------------------------------
inline constexpr double FOV_DEG   = 100.0;   // 总开角 (度)
inline constexpr double FOV_RANGE = 2.0;     // 可视半径 (m)

// ---------------------------------------------------------------------------
// 覆盖路径规划（牛耕往返）
//   关键：飞机不必走遍每个格子，靠 120°/3m 视野"扫过去"覆盖。
//   一条直线车道，扇形向两侧各可靠覆盖约 2.6m（= fov_range*sin60° - 大格半宽）。
//   保守取重叠：车道间距 ≈ 单侧覆盖 ×约1.15，相邻车道视野有重叠，防漏扫。
//   场地越大，车道数越少、飞行路程越短。
// ---------------------------------------------------------------------------
inline constexpr double LANE_SPACING = 3.00;   // 相邻车道间距 (m)（旧牛耕用，动态版已不用）

// ---------------------------------------------------------------------------
// ★ 动态前沿覆盖（竞速版·分层）★  ← 路线不再一成不变，且全局更高效
//   分层：上层把可达区按 BAND_WIDTH(≈视野宽) 沿 y 切成扫描条带，蛇形顺序逐带推进
//         (带迟滞，不折返不横跳)；下层在当前条带内朝最近未扫前沿贪心串链，并用
//         ALONG_BONUS 鼓励顺着条带主方向(x)横扫。下层是未来接【避障】【必经点】的落点。
//   每拍把 150°/3m 视野标进栅格；偏离轨迹或周期到点就从当前位置重规划。
//   完程度达标后转归航 PD 刹停（见下方"完成判定+归航"）。
// ---------------------------------------------------------------------------
inline constexpr double REPLAN_PERIOD_S    = 0.20;  // 周期性重规划间隔 (s)，缩短→路线更跟手、减少视觉延后
inline constexpr double REPLAN_DEV_M       = 0.20;  // 偏离当前轨迹超过此距离立即重规划 (m)，偏一点就重画
inline constexpr double FRONTIER_NEAR_W    = 1.40;  // ★邻近系数★：距离项权重，越大越优先去【最近】的未扫区域
inline constexpr double FRONTIER_TURN_PEN  = 1.70;  // 选目标的转向代价 (m/rad)，越大越爱直行少掉头
inline constexpr double FRONTIER_CLUSTER_W = 1.80;  // 未扫邻居加成 (m/个)，越大越优先大片未知区(别追孤格)
inline constexpr double EXPLORE_HORIZON_M  = 8.00;  // 单次规划链路总长 (m)，分层后可略长，少重算
inline constexpr double CHAIN_GAP_M        = 1.00;  // 链路相邻目标点最小间隔 (m)，按视野铺开

// 分层·上层条带 + 下层顺扫
inline constexpr double BAND_WIDTH      = 4.00;  // 条带宽度 (m)，≈视野单侧覆盖，取 LANE_SPACING
inline constexpr double BAND_TOL        = 0.40;  // 当前条带上下容差 (m)，略出带的格也纳入选点
inline constexpr double ALONG_BONUS     = 1.00;  // 沿条带主方向推进加成 (m/m)，越大越坚定横扫少犹豫
inline constexpr int    BAND_CLEAR_CNT  = 6;     // 剩余未扫格 ≤ 此数才推进下一带(迟滞)，放宽免为残角折返

// ---------------------------------------------------------------------------
// 贝塞尔轨迹采样
// ---------------------------------------------------------------------------
inline constexpr double ARC_SAMPLE_DS = 0.05;  // 沿弧长采样步长 (m)

// ---------------------------------------------------------------------------
// 轨迹跟踪 + PID（机体系输出：前进 v_fwd / 横向纠偏 v_lat / yaw_rate）
//   v_fwd 上限 0.8，跟随曲率动态降：v_fwd = V_MAX / (1 + K_CURV*|κ|)
//   横向只做低限纠偏，主转向靠 yaw_rate
// ---------------------------------------------------------------------------
inline constexpr double V_MAX        = 0.50;   // 前进速度上限 (m/s)
inline constexpr double V_MIN        = 0.00;   // 前进速度下限 (m/s)，防止过弯停死
inline constexpr double K_CURV       = 0.40;   // 曲率降速系数（越大过弯越慢）。1.0→0.4:绕障弧κ≈2时 v 从0.167提到0.28,过弯快约1.7倍;偏切外就往回调
inline constexpr double LOOKAHEAD    = 0.40;   // pure-pursuit 前瞻距离 (m)
inline constexpr double ENDPOINT_SLOW_R = 0.50;// 距终点此半径内开始线性降速 (m)

inline constexpr double KP_YAW       = 1.60;   // 朝向误差 → yaw_rate 的 P
inline constexpr double KD_YAW       = 0.50;   // yaw_rate 的 D
inline constexpr double MAX_YAW_RATE = 1.80;   // yaw_rate 限幅 (rad/s)

// ★先转再走·朝向门控★：防"规划出身后/大角度路径时,机头还没转过来飞机就带着机体平移甩出线外撞柱"。
//   机头到目标方向的偏差 |e_yaw| > HEADING_GATE_DEG → 前进+横向全清零,只转 yaw_rate(原地转身);
//   阈值内用 cos(e_yaw) 平滑门控(乘到前进和横向上)。正常巡航 e_yaw 很小≈不影响,只大角度才压。
//   ★前进和横向都乘这个门(关键)★:光压前进不压横向,飞机仍会沿线法向侧移甩出去。
inline constexpr double HEADING_GATE_DEG = 65.0; // 机头偏离超此角度则原地转身(度)。调小→更早原地转更稳;调大→更敢边转边走

inline constexpr double KP_LAT       = 0.80;   // 横向偏差 → v_lat 的 P
inline constexpr double KD_LAT       = 0.10;   // v_lat 的 D
inline constexpr double MAX_V_LAT    = 0.20;   // 横向纠偏速度上限 (m/s)，压很低

inline constexpr double V_EST_ALPHA  = 0.30;   // 位置差分估速度低通系数（同主控思路）

// ---------------------------------------------------------------------------
// 完成判定 + 归航刹停
//   覆盖率达标 → 进入"归航"：算法侧对到终点的位置误差跑 PD，平滑刹停，
//   精确停在目标点（不再夹安全区）。停稳(到点+速度够小) → finished。
// ---------------------------------------------------------------------------
inline constexpr double DONE_COVERAGE   = 0.90;  // 完程度(已探索大格占比)达此值即"扫完"→归航
inline constexpr double GOAL_TOL_XY     = 0.10;  // 到终点位置容差 (m)，精确停所以收紧
inline constexpr double GOAL_STOP_V     = 0.05;  // 停稳速度阈值 (m/s)，到点且慢于此才算停稳

// 归航 PD（机体系输出 v_fwd/v_lat 直奔终点；D 项吃惯性 → 不冲过头）
inline constexpr double KP_GOAL     = 1.20;   // 位置误差 → 速度的 P (1/s)
inline constexpr double KD_GOAL     = 0.60;   // 速度阻尼 D（越大刹得越稳、越不冲）
inline constexpr double V_GOAL_MAX  = 0.60;   // 归航段速度上限 (m/s)，比巡航略低更好停

// ---------------------------------------------------------------------------
// 主循环 / 看门狗
// ---------------------------------------------------------------------------
inline constexpr int    TIMER_PERIOD_MS = 20;    // 50Hz 算速度
inline constexpr int    VIZ_PERIOD_MS   = 50;    // 20Hz 刷新 OpenCV 弹窗（提高→曲线/飞机更跟手）

// ===========================================================================
// ★ 雷达感知（点云聚类拟合圆）★  ← 障碍检测调参都在这一段
//   流程：订阅 /cloud_registered(世界系点云) → 滤地面/自身回波 → 占据栅格累计
//         → 连通域聚类 → 每簇拟合成圆(圆心+半径)。障碍圆喂给全局 A*(见文件末)
//         在【障碍+墙】膨胀栅格上搜绕障路径——全场唯一避障手段。
// ===========================================================================

// ---- 飞机本体 ----
//   雷达为中心、半径 ROBOT_RADIUS 的圆都算飞机，绝不能碰障碍。
inline constexpr double ROBOT_RADIUS = 0.30;   // 飞机半径 (m)，碰撞判定+自身回波滤除都用它。真机实测 30cm

// ---- 点云预处理 / 聚类（obstacle_map）----
inline constexpr double GROUND_Z      = 0.60;   // ★点云高度窗口·下限(世界z,m)★ 低于此的点丢弃(地面/近地杂物)。飞行高1m→取0.5滤地面
inline constexpr double OBS_Z_MAX     = 1.60;   // ★点云高度窗口·上限(世界z,m)★ 高于此的点丢弃(天花板/高处墙面/吊挂物)。只保留[GROUND_Z,此]之间当障碍
inline constexpr double OBS_EDGE_IGNORE = 0.10; // 距场地四面墙此范围内的点视为"墙"，不当障碍物 (m)
inline constexpr double OBS_SELF_MARGIN = 0.15; // 自身回波余量：距飞机 < ROBOT_RADIUS+此值 的点丢弃 (m)
inline constexpr double OBS_CELL      = 0.15;   // 障碍占据栅格分辨率 (m) ★调大→远处稀疏点更易落同格成簇，但定位略糙
// ★滑动累计+衰减(时间累积)★：本帧命中格 +HIT_INC(封顶 HIT_MAX)，未命中格 -HIT_DECAY(到0消失)，
//   累计值≥HIT_THRESH 才算占据。稀疏小障碍(如3cm杆,单帧点极少、时有时无)靠此【跨帧累积】稳住：
//   高封顶+慢衰减 → 偶尔命中一次就存住不掉；THRESH>INC → 单帧噪声点不足以点亮(要累计≥2帧命中)，抗误检。
//   噪声随机、不重复命中同格→累计不到 THRESH 被自然滤掉；真杆静态、反复命中同格→累起来长期维持。
//   代价：真障碍离开视野后清得慢(50→8 约42帧≈2s)——静态场地(杆/凳不动)无所谓。
//   维持条件：命中率 > DECAY/(INC+DECAY)=1/6≈17%(每6帧点云至少命中1次)；再稀疏就上"latch永久累积"(见下)。
inline constexpr int    OBS_HIT_THRESH = 8;     // 占据判定阈值(累计值≥此算占据)。★>INC 使单帧噪声不点亮(需累计多帧)；误检多调高/漏检调低
inline constexpr int    OBS_HIT_INC   = 5;      // 本帧命中累计 +此值。命中一次顶得高→稀疏障碍也存得住(须 < THRESH 才有抗噪意义)
inline constexpr int    OBS_HIT_MAX   = 50;     // 累计封顶(越大越"黏",离开后清得越慢)。★大封顶=时间累积核心:稀疏点顶上来就长期维持
inline constexpr int    OBS_HIT_DECAY = 1;      // 本帧未命中累计 -此值(越大旧障碍消失越快)。保持1=最慢衰减,最利累积
inline constexpr int    OBS_MIN_CELLS = 1;      // 一个簇最少占据格数，少于视为噪声丢弃 ★调小→远处稀疏柱更易点亮，但易冒假障碍圆
inline constexpr double OBS_MIN_R     = 0.03;   // 拟合圆半径下限 (m)
inline constexpr double OBS_MAX_R     = 0.50;   // 拟合圆半径上限 (m)，超大簇多半是墙/误检，截断
inline constexpr double OBS_R_INFLATE = 0.10;   // 拟合半径额外充气 (m)，宁可估大一点更安全

// ★障碍时序平滑(跟踪)★：逐帧重新聚类拟合的圆会闪动/瞬移(圆心抖、半径跳、忽隐忽现)。
//   把本帧测量圆关联到上一帧的"同一个障碍"，对圆心/半径做指数滑动平均(EMA)——偏重最近的帧、
//   老数据自然淡出，所以"只按最近一段时间算"。最初照不全圆偏小、看全了平滑长到真实大小。
inline constexpr double OBS_TRACK_ALPHA      = 0.30; // EMA 系数(0~1)：越大越跟手(快但抖)，越小越平滑(稳但滞后)。闪→调小，跟不上真实移动→调大
inline constexpr double OBS_TRACK_ASSOC_DIST = 0.60; // 关联门限(m)：测量圆中心距某 track <此值才算同一障碍。两个真障碍靠太近会被并成一个→调小；同一障碍抖出多个圈→调大
inline constexpr int    OBS_TRACK_MAX_MISSES = 15;   // 连续多少帧没再看到就删该障碍(桥接短暂丢帧防闪)。杆子稀疏易丢帧→提到15多桥接(~1s);残留太久→调小；一闪就没→调大

// ★高角速度门控★：飞机快速旋转时 LiDAR 点云会【拖影】(去畸变/位姿滞后→点被甩到障碍真实位置外侧)，
//   这些虚点落进外围格、hit 达阈值 → 被 latch 永久钉成【虚胖大圆】。对策：机体 |yaw_rate| 超此阈值即
//   【整帧丢弃点云】(不加 hit、不 latch)——拖影帧根本不参与累积。旋转停下后正常帧会把真实边界补上。
//   yaw_rate 由节点对 SLAM yaw 数值差分+低通得到。设 <=0 关闭门控(退化老行为，允许拖影帧进入)。
inline constexpr double OBS_SKIP_YAW_RATE = 0.60;    // 丢弃点云的角速度阈值 (rad/s)≈34°/s。调小→更严(稍转就丢帧,更防拖影但累积更慢);调大→更宽松

// ★latch 智能回收(防幽灵)★：latch 是"永不撤销"，缺点是把【早已不存在/曾是噪声】的障碍永久固化成
//   幽灵圆(飞机看那片却无点，圆还在)。回收规则：latch 格【当前在视野(FOV)内】却【连续此帧数无点命中】
//   → 判为幽灵，撤销 latch。看不到的 latch 格(视野外)冻结计数不回收——真静态障碍飞机没在看它时不被误清。
//   真障碍:飞机看它时总命中→计数归零→永不回收(仍满足"不消失")。幽灵:视野扫到却空→清除。
inline constexpr int OBS_GHOST_CLEAR_FRAMES = 25;    // 视野内连续无命中达此帧数(≈2.5s@10Hz点云)→撤销latch。调小→更快清幽灵但真障碍偶尔漏检也易被误清;调大→更稳但幽灵留更久。<=0 关闭回收(退回纯永久latch)

// ===========================================================================
// ★ 全局点到点绕障（A*，全局层）★  ← 全场唯一避障手段
//   思路：探索/POI/归航都用它——在【障碍圆+四面墙】膨胀栅格上做分辨率完备的 A*
//         搜出一条绕过障碍、离墙到目标的折线，再 Catmull-Rom 平滑：探索交 tracker
//         跟随，POI/归航沿线取 carrot 用 PD 跟随并精确刹停。零新依赖(纯 std)。
//         过不去(被围死)直接返回无解 → 探索放弃该区跳带；POI/终点(必达)报警+悬停。
// ===========================================================================
inline constexpr double GLOBAL_CELL      = 0.04;  // A* 搜索栅格分辨率(m)：越小越贴墙/钻窄缝但越慢
inline constexpr double GLOBAL_MARGIN    = 0.30;  // 障碍额外安全余量(m)：障碍禁入半径 = 障碍r + ROBOT_RADIUS + 此值。
                                                  //   飞机【边缘】离障碍【边缘】的物理余量就是此值。2026-07 按需求定 0.30(飞机边缘距杆≥30cm)。
                                                  //   ★越大越早远绕越不贴障碍,但太大会把"柱墙之间的缝"吃掉→墙边柱子绕不过去★。
                                                  //   2026-06-04 曾因 0.3 致墙边卡死(缝被吃→A*无解→面墙卡住)，靠"后退避墙(RETREAT_*)+A*起点突围(relax)"
                                                  //   修复后当时回调 0.2。现重设 0.3=用户安全硬指标：真机杆间距若不足则宁可卡死报警也不贴撞；
                                                  //   墙边柱风险仍靠 RETREAT_* + relax 兜底。若墙边频繁卡死→优先降 GLOBAL_WALL_MARGIN，不要降此值。
inline constexpr double GLOBAL_WALL_MARGIN = 0.60;// 离墙安全距离(m)：cell 中心距任一场地边界<此值即禁入。
                                                  //   2026-06-04 0.80→0.60：原 0.8 偏宽,墙边柱子时把"柱墙之间能过的缝"吃掉→A*判无解、飞机
                                                  //   转头面向墙卡死。0.6 物理离墙余量=0.6-ROBOT_RADIUS(0.2)=0.4m 仍绝不撞墙；且 <WALL_MARGIN(1.0)
                                                  //   覆盖目标(离墙≥1.0m)仍直接可达。调大更躲墙但墙边缝更窄/近墙目标够不到。
inline constexpr double COMMIT_TARGET_TOL = 0.60; // ★路径承诺★目标移动超此距离(m)才重算A*换边。
                                                  //   核心防"绕障左右翻边"：旧路径仍无碰撞就续用,不因左右绕代价 near-tie 每拍翻边→撞柱。
                                                  //   仅【目标大幅移动】或【旧路径被挡(会撞)】才重算。调大→更"咬死"旧路径(更稳但换区迟钝);调小→更易换路。
inline constexpr double GLOBAL_LOOKAHEAD = 0.60;  // 沿绕障轨迹取 carrot 的前瞻距离(m)：越大越平顺但贴角余量小
inline constexpr double EXPLORE_TARGET_MIN_DIST = 1.50; // 探索挑 A* 终点的最小距离(m)：取覆盖路径上第一个
                                                  //   距当前≥此值的点当 A* 目标(而非最远点)——只绕近处障碍,路径短不易过期,到了周期重规划接力。

// ★换路评估(路径迟滞)★(2026-06-10)：COMMIT_TARGET_TOL 只在【目标几乎没动】时咬死旧路；一旦目标
//   漂移触发重算，旧逻辑会无条件采纳新 A* 结果 → 仍可能"一会想走左边、一会想走右边"反复横跳。
//   这里再加一道闸：重算出新路后，先给【旧路】和【新路】各打分(score_path: 剩余弧长=时间, 最小障碍
//   边距=安全)，只有新路【更安全】且/或【更快】超过下列比例，才真的换路；否则续用旧路。
//   关键：左右横跳的两条路是 near-tie(几乎等长、等安全) → 两项都达不到阈值 → 不换 → 横跳根除。
//   只有旧路【已被挡(会撞)】或【已快走完(剩余很短)】时跳过本评估，直接采纳新路(安全/进度优先)。
inline constexpr double PATH_SWITCH_SAFETY_GAIN  = 0.50; // 新路"最小障碍边距"需比旧路高 ≥此比例(20%)才算【更安全】。调大→更不爱为安全换路(更稳)
inline constexpr double PATH_SWITCH_TIME_GAIN    = 0.20; // 新路"剩余弧长"需比旧路短 ≥此比例(15%)才算【更快】。调大→更不爱为抄近路换路(更稳/更咬死旧路)
inline constexpr bool   PATH_SWITCH_REQUIRE_BOTH = true; // true=【更安全且更快】两者都满足才换(最稳/最不横跳)；false=任一满足即换(更看重效率,换路更勤)

// ★机头锥(2026-06-10)★：A* 起点段只许朝飞机机头方向延伸,根除"新规划路线从飞机侧后方起步→飞机
//   边转边往前(机头偏差在强制原地转角内)→机体平移横切撞旁边柱子"。仅约束起点附近(半径内)的扩展,
//   走出此半径恢复全向 A*(不扭曲远处绕障路径)。锥半角必须 < HEADING_GATE_DEG(65°),使被采纳路径的
//   起步方向落在 tracker 门控不触发区→飞机一采纳即可直接向前走,不再有"边转边走"窗口。
inline constexpr double GLOBAL_HEADING_CONE_DEG    = 22.0; // 机头锥半角(度)。调小→更严格只走正前方(更易触发原地转身找解)；调大→更宽容。须 < HEADING_GATE_DEG(65)
inline constexpr double GLOBAL_HEADING_CONE_RADIUS = 0.80; // 锥作用半径(m)。须 ≥ GLOBAL_LOOKAHEAD(0.6)使 carrot 落锥内；太大→近场路径被楔形过度扭曲。设 0 关闭锥(退化老行为)

// ★原地转身找解(2026-06-10)★：探索时机头锥内 A* 无解,但无锥 probe 有解(=有路只是不在机头方向)→
//   不前进/不后退/不跳带,原地转身改变机头朝向(锥随之转向)→某朝向锥内出解就沿它走;弹窗画旋转标志。
//   转满一圈(累计~360°)仍无解=目标真被围死→画红叉+拉黑+跳带去别处。区分"没路"(probe也无解,走后退/跳带)
//   与"朝向不对"(probe有解,转身),精准对应"正前方堵死就原地掉头直到出现解"。
inline constexpr double TURN_SOLVE_YAW_RATE       = 1.20; // 转身找解角速度(rad/s)。须 < MAX_YAW_RATE(1.8)；且 rate*0.05s*RESEARCH_EVERY 必须 < 锥半角,否则可能转过有效窗口才重搜
inline constexpr int    TURN_SOLVE_RESEARCH_EVERY = 1;    // 每隔几拍(20Hz)重搜一次(1=每拍)。A* 较贵时可调 2~3
inline constexpr double TURN_SOLVE_MAX_REV        = 1.0;  // 累计转过此圈数(1.0=360°)仍无解→判目标真被围死。略 >1 更保险
inline constexpr double TURN_SOLVE_TIMEOUT_S      = 20.0; // 兜底超时(s)：防 yaw 不更新等异常导致永久转身

// ★放弃区域·黑名单★：探索时某目标 A* 无解(被障碍/墙围死够不到) → 把它拉进"够不到黑名单",
//   plan_explore 选点时跳过黑名单附近的格,飞机直接去扫别处,【不再回头反复试这条死路】。
//   防永久误杀:覆盖率每涨过 UNREACH_CLEAR_STEP 台阶清一次 + 收到新 goal 清空(飞机换位/视角后
//   原够不到的可能又够得到,给"再试一次"机会)。
inline constexpr double UNREACH_BLOCK_R    = 0.80; // 拉黑邻域半径(m)：落此范围内的候选格都跳过。调大→整片拉黑更不横跳但易误杀;调小→更精准但相邻格可能又被选中。
inline constexpr double UNREACH_CLEAR_STEP = 0.05; // 覆盖率每涨过此台阶(5%)清空一次黑名单,给死路区重试机会。

// ★脱困后退★(2026-06-04)：避障中飞机若慢慢蹭近障碍、最后 A* 报无解卡死 → 不再"死在脸上",
//   先沿【远离最近障碍】方向低速后退一小段、边退边重算 A*：退出去后通道重新打开就接着绕；
//   退到 RETREAT_MAX_DIST 仍无解才判定真被围死(悬停报警/探索跳带)。三分支(探索/POI/归航)共用。
inline constexpr double RETREAT_TRIGGER_M  = 0.55; // 触发后退的贴障距离(m)：A*无解 且 机到最近障碍边缘<此 → 后退;
                                                   //   ≥此说明不是贴脸卡死(真被围死)→直接走原放弃/悬停逻辑。须 > 物理禁入(障碍r被snapshot给)余量。
inline constexpr double RETREAT_STEP_M     = 0.50; // 每次后退目标点距离(m)：朝远离障碍方向取此远的点当 PD 目标,低速退。
inline constexpr double RETREAT_MAX_DIST   = 1.20; // 累计后退超此距离(m)仍无解 → 放弃后退,判真被围死。防无限后退。
inline constexpr double RETREAT_V_MAX      = 0.35; // 后退限速(m/s)：低速蹭出来,别猛退甩飞。

}  // namespace params
}  // namespace exploration
