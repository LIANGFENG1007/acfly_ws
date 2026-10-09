// ============================================================================
//  params.hpp  ── 探索算法可调常量默认值
//  说明：这些是"编译期默认值"。运行期都通过 ROS 参数(declare_parameter)覆盖，
//        ros2 run ... --ros-args -p name:=value 即可改，不必重编译。
// ============================================================================

#pragma once

#include <cmath>

namespace exploration {
namespace params {

// ★ 起飞后是否先探索 ★（启动时可用 ROS 参数 exploration_enabled 覆盖）
//   true  = 先探索，覆盖率达标后去探索终点，再执行通道任务。
//   false = 跳过探索和视觉/插点访问，起飞后直接去探索终点，再执行通道任务。
//   直达终点仍使用原有避障与场地安全检查；场地边界请填写实际范围。
inline constexpr bool EXPLORATION_ENABLED = true;

// 探索终点到配置边界的最小距离(m)。0 = 终点中心可到边界，不能越界。
// 只放宽前往探索终点的末段；普通探索/视觉目标仍使用原安全边距。
// 配置边界作为任务范围；点云中的实体障碍仍按机体半径和安全余量避让。
inline constexpr double HOME_GOAL_WALL_MARGIN = 0.0;

// ---------------------------------------------------------------------------
// ★ 场地四面墙范围（SLAM/camera_init 系，原点=起飞点）★  ← 改这里
//   这是"飞机起飞点为原点"算出来的场地边界。起飞点不在场地中心时要换算：
//   边界(SLAM) = 边界(世界) - 起飞点(世界)。
//   例：起飞点世界 (-4,0)，10x10 场地世界 x,y ∈ [-5,5]
//       → SLAM x ∈ [-5-(-4), 5-(-4)] = [-1, 9]，y ∈ [-5, 5]
// ---------------------------------------------------------------------------
// 10x10 比赛场地(与提交 f2484b7「最新版本」一致)。实机小场地为 x∈[-0.3,4.86]、y∈[-2.7,2.4]。
inline constexpr double FIELD_MIN_X = -0.5;
inline constexpr double FIELD_MIN_Y = -3.8;
inline constexpr double FIELD_MAX_X = 9.0;
inline constexpr double FIELD_MAX_Y = 4.6;
// ★自主探索·车式前进模式★（仅影响 exploration_planner 的轨迹跟踪）
//   true  = 巡航以前进/yaw跟踪为主，允许受避障检查的横漂补偿；转向时保持XY。
//   false = 保留旧的全向跟踪方式（允许 v_lat 横向纠偏）。
inline constexpr bool EXPLORATION_CARLIKE_MODE = true;

// ★视觉目标 ID 黑名单★：SHM 里这些类别 ID 的目标一读到就丢弃，不参与确认/排队/投放，飞机不会飞过去。
//   SHM 里 ID 是 int32 整数(有效 1~6)，视觉端显示的 "005" 就是整数 5。这里必须写十进制 5、2，
//   不要写 005/002(C++ 以 0 开头是八进制，008 会编译失败、010 会变成 8)。
//   运行期可覆盖：-p vision_id_blacklist:="[2,5]"；清空黑名单：-p vision_id_blacklist:="[0]"。
inline constexpr int VISION_ID_BLACKLIST[] = {3, 4, 5};

// 视觉SHM目标确认、排队和投放流程。
// 视觉投放：在目标周围1m圆周上选可直线斜降的入口；正常直达优先，受阻才换圆周方向。锁XY/Z/yaw采样后下降。
// 高度=主控起飞雷达Z+HEIGHT_M+已启用的Z补偿；Z容差使用主控起飞TOL_Z；释放后锁XY等待RELEASE_WAIT_S再回原高度。
inline constexpr bool VISUAL_DROP_ENABLED = true;        // true=启用三包裹投放；false=旧的视觉到点悬停流程，不执行投放
inline constexpr double VISUAL_DROP_ENTRY_M = 1.0;       // 采样入口圆半径(m)，圆心是视觉目标。直达/绕障都在这一圈选点；已在圈内且可直降时不倒退凑距离
inline constexpr double VISUAL_DROP_APPROACH_SLOW_M = 1.20; // 距采样点此范围内逐步减速(m)；受阻选点仅用原GLOBAL_MARGIN，不再额外要求远离障碍
inline constexpr double VISUAL_DROP_ENTRY_SPEED_MPS = 0.15; // 接近安全点时的指令速度上限(m/s)，仍持续XY纠偏；到点后采样1秒，再核准直线并下降
inline constexpr double VISUAL_DROP_SAMPLE_S = 1.0;      // 锁定位置后的采样时长(s)：新帧能收多少收多少，加权平均；无有效帧则用粗坐标
inline constexpr double VISUAL_DROP_HEIGHT_M = 0.03;     // 投放时雷达相对起飞位置的高度(m)：目标Z=起飞雷达Z+此值，不是离目标物表面高度
// ★投掷高度 SLAM Z 误差补偿★(2026-09-24 实测包 z_drift_0924_1941：雷达贴地、物理高度不变拖遍全场)
//   SLAM 的 Z 读数随位置线性偏移 err = X·x + Y·y(相对起飞点，拟合残差 RMS 0.4 cm)；x 方向每米低 3.4 cm，8.5 m 处低约 29 cm。
//   投掷目标 Z = 起飞Z + VISUAL_DROP_HEIGHT_M + err(目标位置)，让远处投掷的真实离地高度也是 0.15 m。
//   ★只作用于投掷这一个高度★(下降对准/释放)；巡航、采样、回升等其他高度一律不矫正(用户要求)。
//   前提：起飞点与起飞机头朝向与实测时一致(SLAM 坐标系随起飞位姿建立)。换场地/换起飞朝向要重新录包拟合。全置 0 = 不补偿
//   ★fly_mission params.hpp 的 VISION_Z_SLOPE_X/Y(发给视觉的 z 矫正)必须与 SLAM_Z_DRIFT_X/Y 同值，改这里要一起改★
inline constexpr double SLAM_Z_DRIFT_X = -0.0338;         // SLAM Z 实测漂移：每米 x 的 Z 读数误差(m/m)。投掷高度补偿与走廊点云地面保护共用
inline constexpr double SLAM_Z_DRIFT_Y = 0.0016;          // 同上，每米 y
inline constexpr bool   VISUAL_DROP_Z_COMP_ENABLED = true; // 投掷高度是否按上面的漂移补偿(false=投掷用原 0.15 不补偿；不影响走廊地面保护)
inline constexpr double VISUAL_DROP_Z_COMP_MAX = 0.40;    // 补偿量限幅(m)，防坐标异常时下降过猛；场地内最大约 0.31 m
inline constexpr double VISUAL_DROP_XY_TOL_M = 0.10;     // 包裹水平到位容差(m)：经安装偏移补偿后的包裹XY距视觉目标≤此值即到位，不要求速度降到零
inline constexpr double VISUAL_DROP_STABLE_S = 2.0;      // 提前投放所需连续稳定时间(s)：包裹XY和雷达Z均到位；超出任一容差则重新计这段时间
inline constexpr double VISUAL_DROP_TIMEOUT_S = 4.0;      // 强制投放等待上限(s)：包裹XY首次到位即开始，漂移不重置；到时即发送，不再等待Z/稳定
inline constexpr double VISUAL_DROP_OFFSET_M = 0.28;     // 包裹距雷达的水平安装距离(m)：第1个正前、第2个正左、第3个正右，按实际yaw旋转补偿
inline constexpr double VISUAL_DROP_DESCEND_MPS = 0.60;  // 投放下降速度上限(m/s)：根据剩余XY距离估计到达时间并调整限速，实际Z速度由主控PD计算
inline constexpr double VISUAL_DROP_CLIMB_MPS = 0.60;    // 释放后回升速度上限(m/s)：锁定释放时XY，回到开始采样时的高度；持续输出XY纠偏速度
inline constexpr double VISUAL_DROP_CLIMB_RELEASE_M = 0.50; // 0.35→0.5(2026-09-24 仿真第 15 轮：回升每局再省 1.8 s，倾角无变化)。 回升到离返回高度 <此值(m)就恢复探索，剩余高度边飞边爬(障碍点云按世界高度截取、柱子全高，低 0.35 m 不影响避障)。原来锁 XY 等高度进容差，每次约多等 2 s。0=旧行为
inline constexpr double VISUAL_DROP_RELEASE_WAIT_S = 1.0; // 串口写出完成(或失败)后，锁XY并保持投放高度等待此时长(s)再回升：给电磁铁断电、包裹脱离留时间。
                                                          //   计时从主控回报串口结果那一拍起算，是固定物理等待，漂移/定位中断都不重置。0=写出后立即回升(旧行为)
// Z到位容差使用主控的params::TOL_Z；下降/回升速度上限当前均不得超过0.60m/s。

inline constexpr bool VISION_SHM_ENABLED = true;
inline constexpr const char* VISION_SHM_PATH = "/dev/shm/uav_cv_out";
inline constexpr double VISION_SHM_MAX_AGE_S = 0.50; // 图像采集单调时间距当前最大年龄(s)
inline constexpr double VISION_SHM_FUTURE_TOL_S = 0.02; // 同机时钟转换容许的未来偏差(s)
inline constexpr int VISION_FAR_CONFIRM_FRAMES = 10; // 同一区域连续新帧，远处不使用类别ID关联
inline constexpr double VISION_ASSOC_RADIUS_M = 0.60; // 位置关联半径(m)，与完成黑名单独立
inline constexpr double VISION_WEIGHT_MIN_DISTANCE_M = 0.50; // 距离平方反比权重的最小距离(m)，防近零距离过大权重
inline constexpr double VISION_FRAME_GAP_TIMEOUT_S = 0.50; // 超过此时间无有效观测，中断未完成的连续确认
inline constexpr double VISION_TENTATIVE_TIMEOUT_S = 2.00; // 未确认候选失去观测后保留时间(s)
inline constexpr double VISION_POSE_TIMEOUT_S = 0.30; // 位姿过期时不累计视觉坐标
inline constexpr int VISION_MAX_CANDIDATES = 64; // 内部候选总容量，不是单帧目标数；不静默挤掉已确认目标
inline constexpr int VISION_QUEUE_CAPACITY = 64; // 当前、等待、暂缓目标合计容量
inline constexpr double VISION_BLACKLIST_RADIUS_M = 0.60; // 完成后以最终精确坐标为中心屏蔽(m)
inline constexpr int VISION_HINT_LIMIT = 6; // 视觉线索回访上限：远处断续看到(连续≥3帧未确认)的位置恢复为未扫、安排近看，每次任务最多这么多处。0=关闭
inline constexpr double VISION_HINT_PRIORITY_M = 6.0; // 视觉线索优先：线索 0.8 m 内未扫格的选点代价减此值(m 当量)，看到疑似目标就先去近看确认(r128：线索被搁置，先去了远处，最后长途折返)
// 以下旧近距确认/悬停参数仅在 VISUAL_DROP_ENABLED=false 时控制旧访问流程。
inline constexpr int VISION_NEAR_CONFIRM_FRAMES = 10; // 近处独立的新帧窗口；ID票数平票继续采样
inline constexpr double VISION_NEAR_DISTANCE_M = 2.00; // 小于此水平距离开始近距确认(m)
inline constexpr double VISION_NEAR_SPEED_MPS = 0.40; // 近距请求速度上限，待飞行执行接入后消费(m/s)
inline constexpr double VISION_NEAR_TIMEOUT_S = 3.00; // 近距精确坐标收集窗口(s)，不足10帧则使用已有平均
inline constexpr double VISION_RETRY_DELAY_S = 5.00; // 暂缓后冷却时间，且冷却后必须重新看到才重排(s)

inline constexpr double VISION_HOVER_S = 3.00; // 首次进入目标位置容差后计时(s)，不等待停稳，偏移继续矫正但不重置
// 视觉临时目标最后这段距离停止主动转头，改为水平XY打点；悬停完成后恢复。
// 0 = 关闭；直线被障碍挡住时继续走原避障路径，直线可通行后再切换。
inline constexpr double VISION_STRAIGHT_APPROACH_M = 0.40;
inline constexpr double VISION_TARGET_TOL_M = 0.10; // 精确目标到位位置容差(m)
inline constexpr double VISION_TARGET_STOP_SPEED_MPS = 0.05; // 兼容旧配置保留，已不用于视觉目标悬停计时条件

// 探索终点后的走廊任务。入口/H 坐标由 fly_mission 的参数区发送，均为 camera_init 系。
// true: 到探索终点等待路线，然后转向、进入、穿门、到 H 才 finished。
// false: 保留到探索终点即完成的流程。通道控制接口由 CORRIDOR_POSITION_CONTROL 选择。
inline constexpr bool CORRIDOR_ENABLED = true;
inline constexpr bool CORRIDOR_POSITION_CONTROL = true; // 降高和转正完成后改发真实位置接口；探索/终点转向仍用速度
inline constexpr double CORRIDOR_POSITION_LEAD_M = 0.60; // 动态位置目标的最大XY领先距离(m)，给飞控保留前进拉力；墙/门会进一步限制
inline constexpr double CORRIDOR_POSITION_ACCEL_MPS2 = 0.30; // 位置参考加速度上限(m/s²)：入口/对门/出门时连续改变推进方向；实际纠偏由飞控负责
inline constexpr double CORRIDOR_POSITION_LATERAL_GAIN = 1.00; // 横向偏离时，目标向中线另一侧额外延伸的比例；1=偏右4cm时请求向左补偿4cm
inline constexpr double CORRIDOR_POSITION_LATERAL_MAX_M = 0.18; // 上述额外侧向补偿上限(m)，实际还会收紧到门宽减机宽后的余量
inline constexpr double CORRIDOR_POSITION_LATERAL_BRAKE_S = 0.60; // 按已有横向速度提前收回补偿的时间(s)，减少回到中线后冲过头
inline constexpr bool CORRIDOR_NEAREST_CARDINAL_YAW = true; // true: 到终点后转到最近的0/±90/180度，选一次并保持到H；false: 使用下面固定角度
inline constexpr double CORRIDOR_INITIAL_YAW_DEG = -90.0; // 仅关闭最近转正时使用的固定航向(度)
inline constexpr double CORRIDOR_WIDTH = 1.5; // 两侧墙搜索范围(m)，不把道路中线当成门中心
inline constexpr double CORRIDOR_ROBOT_WIDTH = 0.64; // x500仿真含桨完整外廓(m)，实机按测量值调整
// 实机 Point-LIO 默认输出；仿真 launch 在启动稠密转换节点后单独覆盖此参数。
inline constexpr const char* CORRIDOR_CLOUD_TOPIC = "/cloud_registered";
inline constexpr double CORRIDOR_MIN_GAP_EXTRA = 0.0; // 机宽以外额外要求(m)，默认不额外膨胀
// 位置接口下这是位置参考的推进速度(m/s)，实际速度与加减速由飞控位置环控制。
inline constexpr double CORRIDOR_ENTRY_SPEED = 0.30;
inline constexpr double CORRIDOR_CRUISE_SPEED = 0.30;
// 下列速度下限/横向速度及加速度只用于旧速度接口；位置接口不在本机计算纠偏速度。
inline constexpr double CORRIDOR_MIN_FORWARD_SPEED = 0.30; // ★隧道里前进速度下限(m/s)★ 2026-09-26 用户定(0.2→0.3)：整个隧道都不低于它(涡流里太慢推不动)；只在快到 H、门还没确认时减速
inline constexpr double CORRIDOR_LATERAL_SPEED = 0.45;     // ★隧道里横向纠偏速度上限(m/s)★ 和前进分开算；门前 0.6 m 内要横移到门中心，门偏得多要调大
inline constexpr double CORRIDOR_LATERAL_ACCEL = 0.60;     // 隧道里横向指令斜坡(m/s²)
inline constexpr double CORRIDOR_H_APPROACH_M = 1.20;      // 离 H 只剩这么远(m)时直接朝 H 飞：隧道墙可能在 H 前就结束(实机 H 前 0.17 m 墙就没了)
inline constexpr double CORRIDOR_GATE_SPEED = 0.30; // 正对门中心穿越的速度上限(m/s)
inline constexpr double CORRIDOR_ALIGN_SPEED = 0.30;
inline constexpr double CORRIDOR_YAW_KP = 1.60;
inline constexpr double CORRIDOR_MAX_YAW_RATE = 0.60;
inline constexpr double CORRIDOR_YAW_ACCEL = 1.20;
inline constexpr double CORRIDOR_ACCEL = 0.30;
inline constexpr double CORRIDOR_POSITION_KP = 0.80; // 本机XY保持/旧速度接口增益(1/s)；切入真实位置接口后不使用
inline constexpr double CORRIDOR_VELOCITY_KD = 0.35; // 本机XY保持/旧速度接口阻尼；切入真实位置接口后不使用
inline constexpr double CORRIDOR_VELOCITY_FILTER_S = 0.15; // 平移速度估计滤波(s)，用于阻尼和停稳判定
inline constexpr double CORRIDOR_ARRIVAL_HYSTERESIS = 1.50; // 到点后偏离超过此倍容差才重新对位
inline constexpr double CORRIDOR_YAW_TOL_DEG = 5.0;
inline constexpr double CORRIDOR_HEADING_STOP_DEG = 15.0;
inline constexpr double CORRIDOR_POINT_TOL = 0.25;
inline constexpr double CORRIDOR_STOP_SPEED = 0.04; // 兼容旧配置保留，转向和H点交接不再等待此速度
inline constexpr double CORRIDOR_SETTLE_S = 0.25;
inline constexpr double CORRIDOR_LOOKAHEAD = 0.25;
inline constexpr double CORRIDOR_APPROACH_M = 0.80; // 旧速度接口开始对中/位置接口进入穿门判定区(m)；位置接口确认安全斜线后提前对中
inline constexpr bool CORRIDOR_CONTINUOUS_APPROACH = true; // 持续前进并对门中心，原启动命令直接生效
inline constexpr double CORRIDOR_CONTINUOUS_CENTER_RESERVE = 0.15; // 机体前缘之外的门前对中预留(m)
inline constexpr double CORRIDOR_CONTINUOUS_TIME_MARGIN = 0.50; // 横向对中时间估计的附加余量(s)
inline constexpr double CORRIDOR_MOVING_LOOKAHEAD = 0.80; // 穿门后接力用的移动前瞻(m)，避免追到临时点刹停
inline constexpr double CORRIDOR_CENTER_PREDICTION_S = 0.40; // 对中放行同时检查预测横向漂移(s)
inline constexpr double CORRIDOR_EXIT_M = 0.50; // 门后沿当前门中心直行此距离(m)再向下一门/H偏移；空间不足时缩短。
                                                // 1.0→0.5(2026-09-23)：开口靠墙时(0.75m 开口/0.64m 机宽)出门段贴墙，r01 实测机身离墙 2.7cm 飞了 1m；
                                                // 机身完整越过门板只需 半机宽+栅格≈0.36m，0.5m 仍是完整直线出门
inline constexpr double CORRIDOR_CENTER_TOL = 0.15; // 穿越前对门中心横向误差(m)
inline constexpr double CORRIDOR_GATE_ASSOC_M = 0.15;
inline constexpr int CORRIDOR_CONFIRM_FRAMES = 3;
inline constexpr double CORRIDOR_CLOUD_TIMEOUT_S = 0.50; // 本机连续未收到新点云帧的时长，不与传感器时钟相减
inline constexpr double CORRIDOR_CLOUD_WINDOW_S = 5.00; // 累计真实墙面点；新数据/门洞射线仍受独立超时检查
inline constexpr double CORRIDOR_POSE_TIMEOUT_S = 0.30; // 本机连续未收到新有效里程计帧的时长；探索/归航/通道共用
inline constexpr double CORRIDOR_Z_BELOW = 0.30; // 相对当前飞机高度的点云截取下界(m)
inline constexpr double CORRIDOR_Z_ABOVE = 0.30; // 相对当前飞机高度的点云截取上界(m)
// ★走廊低空穿越★：先到终点周围 HOME_STRAIGHT_APPROACH_M 半径上的安全入口，再直线斜降；
//   XY 到点立即按上述航向策略转正，转向期间继续降高并保持 XY 纠偏。
//   航向和高度都到位后才横移进入口、穿门、到 H；走廊全程保持该高度。
inline constexpr bool   CORRIDOR_LOWER_ENABLED = true;
inline constexpr double CORRIDOR_FLIGHT_Z = 0.05;      // 走廊飞行高度(绝对 SLAM Z，m)：用户从起点开雷达、带到终点实测的读数，已含该处漂移，不再矫正
inline constexpr double CORRIDOR_DESCEND_MPS = 0.50;   // 降到走廊高度的垂直限速(m/s)，用户定 0.5；主控上限 0.6
inline constexpr double CORRIDOR_Z_TOL_M = 0.05;       // 实际高度与走廊高度差小于此值(m)才开始横移进入口
inline constexpr double CORRIDOR_CLOUD_FLOOR_MARGIN_M = 0.10; // 走廊点云切片下界不低于"起飞雷达高度+该处 SLAM Z 漂移(SLAM_Z_DRIFT_X/Y)+此余量"：低空时不把地面当墙/门
inline constexpr double CORRIDOR_SELF_RADIUS = 0.10; // 仅去掉雷达自身近距离回波(m)
inline constexpr double CORRIDOR_SENSOR_RANGE = 6.0;
inline constexpr double CORRIDOR_CLOUD_MAX_YAW_RATE = 0.60; // 更快旋转时不更新走廊点云，防拖影
inline constexpr double CORRIDOR_CELL = 0.04;
inline constexpr double CORRIDOR_LOOKBEHIND = 0.80;
inline constexpr double CORRIDOR_WALL_SEARCH_M = 0.35;
inline constexpr double CORRIDOR_WALL_EXCLUSION_M = 0.08;
inline constexpr double CORRIDOR_WALL_MIN_SPAN = 0.40;
inline constexpr int CORRIDOR_WALL_MIN_POINTS = 3; // 仍须满足纵向跨度；0.3m体素下短视距不能要求6个独立格
inline constexpr double CORRIDOR_MAX_WALL_GAP = 0.55;
inline constexpr double CORRIDOR_GATE_MIN_SPAN = 0.18;
inline constexpr double CORRIDOR_GATE_DEPTH_CLUSTER = 0.12;
inline constexpr double CORRIDOR_GATE_MAX_DEPTH = 0.45;
inline constexpr double CORRIDOR_SURFACE_SAMPLE_GAP = 0.35; // 同一横档表面允许的采样间隔(m)，兼容上游0.3m体素；不是膨胀
inline constexpr int CORRIDOR_GATE_MIN_POINTS = 3;
inline constexpr int CORRIDOR_GATE_MIN_CLEAR_RAYS = 2; // 必须有本帧穿过门洞命中后方表面的光线，避免把缺点当开门
inline constexpr double CORRIDOR_ENDPOINT_EXCLUSION = 0.30;
inline constexpr double CORRIDOR_MIN_OBSERVED_AHEAD = 0.35;

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
// 可视区（几何模型，第一版不用点云）：前方 FOV_DEG 开角、FOV_RANGE 半径的扇形
//   当前 100° → 半开角 ±50°
// ---------------------------------------------------------------------------
inline constexpr double FOV_DEG   = 100.0;   // 总开角 (度)
inline constexpr double FOV_RANGE = 3.0;     // 可视半径 (m)
inline constexpr double SCAN_DWELL_S = 0.5; // 小格累计在视野内满此时长(s)才算已扫：视觉远距确认要连续 10 帧，掠视一拍不算看过。0=扫到即算(旧)
inline constexpr double SCAN_NEAR_RANGE_M = 2.0; // 此距离内在视时间全额计入 SCAN_DWELL_S，更远只计一半(远处检测断续/易认错)。0=不区分
inline constexpr int SCAN_HINT_DWELL_FACTOR = 3; // 视觉线索区的格要近处累计看满 SCAN_DWELL_S×此倍数(1.5 s)才算扫过，让飞机在疑似目标前多看一会儿凑满确认帧(差相机压力测试 r223)

// ---------------------------------------------------------------------------
// 覆盖路径规划（牛耕往返）
//   关键：飞机不必走遍每个格子，靠 FOV_DEG/FOV_RANGE 视野"扫过去"覆盖。
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
//   每拍把 FOV_DEG/FOV_RANGE 视野标进栅格；偏离轨迹或周期到点就从当前位置重规划。
//   完程度达标后转归航 PD 刹停（见下方"完成判定+归航"）。
// ---------------------------------------------------------------------------
inline constexpr double REPLAN_PERIOD_S    = 0.20;  // 周期性重规划间隔 (s)，缩短→路线更跟手、减少视觉延后
inline constexpr double REPLAN_DEV_M       = 0.10;  // 偏离超此距离(m)且剩余执行路线不安全时立即重规划；安全偏离由tracker纠正
inline constexpr double REPLAN_CANDIDATE_PERIOD_S = 0.60; // 安全旧路仍有收益时的候选搜索间隔(s)；0关闭，紧急换路/末端接力不等待
// ★选点参数 2026-09-24 重调★ NEAR_W 1.4→2.4、TURN_PEN 3.5→1.5、CLUSTER_W 1.8→0.8、GAIN_WEIGHT 0.15→0.5、STANDOFF 1.0→1.3、
//   CLEARANCE_WEIGHT 2.0→1.0、BAND_WIDTH 4→8(不再分条带)、ALONG_BONUS 1.0→0.5；CONTINUITY_TURN_PEN 保持 1.0
//   (28 布局×随机目标、代价=均值+0.3×p90+停转×3 s 复核已收敛；560 局中位 118→93 s、p90 141→115 s)(2026-09-24 离线探索台架随机目标扫参：8 裁判布局×10 组随机目标，代价=找全目标+归航用时+停车转向×3 s；合计 -23%、停转 -25%，20 个未参与布局验证 -17%)
inline constexpr double FRONTIER_NEAR_W    = 2.40;  // ★邻近系数★：距离项权重，越大越优先去【最近】的未扫区域
inline constexpr double FRONTIER_TURN_PEN  = 1.50;  // 选目标的转向代价 (m/rad)，越大越爱直行少掉头。1.7→3.5(2026-09-23 离线探索台架 24 布局：
                                                    //   用时 -8%、停车转向 35→11、最慢布局 72→52s；18 个变快，最多慢 3.8s)
inline constexpr double FRONTIER_CLUSTER_W = 0.80;  // 未扫邻居加成 (m/个)，越大越优先大片未知区(别追孤格)
inline constexpr bool FRONTIER_OBSERVATION_ENABLED = true; // 优先宽敞观察点；轨迹仍由连续曲线跟踪
inline constexpr int FRONTIER_SMALL_REGION_CELLS = 8; // 少于此格数的未知连通块按碎片处理，仍允许最后补扫
inline constexpr double FRONTIER_SMALL_REGION_PENALTY = 2.0; // 小碎片的有界选点代价(m)，不是禁飞区
inline constexpr double FRONTIER_OBSERVATION_STANDOFF = 1.3; // 在未知区域外侧寻找观察位置的距离(m)
inline constexpr double FRONTIER_PREFERRED_CLEARANCE = 0.95; // 观察点到障碍表面的期望距离(m)，硬安全距离由全局规划决定
inline constexpr double FRONTIER_GAIN_WEIGHT = 0.5; // 可见未扫大格收益权重
inline constexpr double FRONTIER_CLEARANCE_WEIGHT = 1.0; // 观察点空间不足的软惩罚权重
inline constexpr double FRONTIER_OBSERVATION_MIN_DISTANCE = 1.50; // 正常观察航点最小推进距离(m)，减少短段反复刹停
// ★收尾顺路★：覆盖率 ≥ START 后选点代价加 W×(去候选点让回红点多绕的路程)。离线台架 24 布局：各档都不变差，
//   W=1/START=0.7 总用时 -1.8%、归航路程 -6%；更强的"终点引导/先远后近"都对参数敏感，未采用。
inline constexpr double FRONTIER_GOAL_DETOUR_W = 1.0;
inline constexpr double FRONTIER_GOAL_DETOUR_START = 0.70;
inline constexpr double FRONTIER_CONTINUITY_TURN_PEN = 1.00; // 安全原路线换向的软代价(m/rad)，减少目标扫完后突然横切换区；0关闭此附加项
inline constexpr double FRONTIER_CONTINUITY_LOOKAHEAD_M = 1.00; // 从原执行轨迹前方取方向的距离(m)，兼顾已规划弯道而非只看当前机头
inline constexpr double EXPLORE_GUIDE_MIN_SEGMENT_M = 0.10; // 合并短于此长度的冗余引导段，合并后的直线仍须完整验障(m)
inline constexpr double EXPLORE_GUIDE_MAX_DEVIATION_M = 0.02; // 允许合并的小拐点距替代直线的最大偏差(m)，保留真正绕障拐点
inline constexpr double EXPLORE_PLAN_RESERVE_M = 0.08; // 探索新引导路预留的额外障碍距离(m)，减少贴判定边界反复失效；窄处回退原安全余量，走廊不使用
// ★孤格饥饿修复(2026-08)★ ← 解决"障碍遮挡出的小片未扫区，飞机走很远以后才回来扫"
//   成因：遮挡区在飞机路过时被挡住扫不到 → 周围格陆续扫完 → 它的未扫邻居数 nbr 从 8 掉到
//   1~2 → 代价里 -FRONTIER_CLUSTER_W*nbr 从 -14.4 变成 -1.8，代价【跳升 12.6】
//   (≈要多绕 9m 才划算) → 排名从队首暴跌到队尾 → 再无机会被选中，直到全场只剩它们才
//   被迫回头。实测有格 t=0 就被看见、却拖到 47s 才扫完(全程仅 50s)。
//   对策：cluster 加成【饱和】—— 用 min(nbr, CAP) 代替 nbr，大片区不再线性碾压孤格。
//   实测(7 场景 × 4 起飞点 = 28 组)：总用时 -9.2%、里程 -8.9%、
//   滞后>10s 的格 1160→908、最大滞后 61.9s→58.8s，零超时。
//   ★不可调太小★：CAP=4 时有 2 组超时(+44.5%)——压太狠会让飞机放弃"优先大片"策略、
//   在零散格间乱跳。CAP=5 为 -3.7%，CAP=7 为 -7.3%，6 是实测最优。≥8 等于关闭。
inline constexpr int    FRONTIER_CLUSTER_CAP = 6;   // nbr 饱和上限（0 或 ≥8 = 关闭）
inline constexpr double EXPLORE_HORIZON_M  = 8.00;  // 单次规划链路总长 (m)，分层后可略长，少重算
inline constexpr double CHAIN_GAP_M        = 1.00;  // 链路相邻目标点最小间隔 (m)，按视野铺开

// 分层·上层条带 + 下层顺扫
inline constexpr double BAND_WIDTH      = 8.00;  // 条带宽度 (m)，≈视野单侧覆盖，取 LANE_SPACING
inline constexpr double BAND_TOL        = 0.40;  // 当前条带上下容差 (m)，略出带的格也纳入选点
inline constexpr double ALONG_BONUS     = 0.50;  // 沿条带主方向推进加成 (m/m)，越大越坚定横扫少犹豫
inline constexpr int    BAND_CLEAR_CNT  = 12;    // 剩余未扫格 ≤ 此数才推进下一带(迟滞)，放宽免为残角折返。6→12 与 FRONTIER_TURN_PEN 同批调整(台架)

// ★目标失效检测(2026-08)★：解决"明明已覆盖了未探索部分、再往前走也不增加进度，却还在往前走"。
//   根因：路径承诺/迟滞只看【目标动没动、路撞不撞】，从不问【这个目标还值不值得去】。
//   飞向目标的途中视野早把那片扫完了，承诺仍咬着它，一路飞到跟前才罢休。
//   对策：每次重规划时检查承诺目标【邻域 R 内是否还有未扫大格】；已无 → 立刻放弃承诺改投别处
//   (绕过 COMMIT_TARGET_TOL 与换路评估两道闸，否则识别了也会被打回)。
//   ★半径取值依据★(6 场景仿真，主指标=达 90% 覆盖用时，与触发条件无关不循环)：
//     基线 281.0s | R=0(仅看目标格自身) 283.3s 无改善 | R=1.0 242.3s(-13.8%，六场景全面更快)
//     R=1.5 250.2s | R=2.0 293.3s(比基线还慢——半径过大会过早放弃仍有价值的目标)
//   调小→更保守(接近旧行为)；调大→更早放弃目标，但过大反而变慢。0 = 关闭本检测。
inline constexpr double TARGET_STALE_R = 1.00;   // 目标失效判定的邻域半径 (m)

// ---------------------------------------------------------------------------
// 贝塞尔轨迹采样
// ---------------------------------------------------------------------------
inline constexpr double ARC_SAMPLE_DS = 0.05;  // 沿弧长采样步长 (m)

// ---------------------------------------------------------------------------
// 轨迹跟踪 + PID（机体系输出：前进 v_fwd / 横向纠偏 v_lat / yaw_rate）
//   v_fwd 上限 0.6，跟随曲率动态降：v_fwd = V_MAX / (1 + K_CURV*|κ|)
//   全向模式横向做低限纠偏；车式模式巡航关闭横移，原地转向允许XY定点补偿。
// ---------------------------------------------------------------------------
inline constexpr double V_MAX        = 0.8     //前进速度上限
;   // 探索前进速度上限 (m/s)，弯道按曲率和航向误差平滑降速
inline constexpr double V_MIN        = 0.00;   // 前进速度下限 (m/s)，防止过弯停死。
                                               //   ★当前为 0 = 该下限保护是空操作★(tracker 里那段 if 恒不生效)。
                                               //   若实测出现"过弯降速到几乎不动"，调到 0.05~0.10 即可启用兜底。
inline constexpr double K_CURV       = 0.40;   // 曲率降速系数（越大过弯越慢）。1.0→0.4:绕障弧κ≈2时 v 从0.167提到0.28,过弯快约1.7倍;偏切外就往回调
inline constexpr double CARLIKE_K_CURV = 1.00; // 车式模式曲率降速 v=V_MAX/(1+k·κ)。0.5→1.0(2026-09-23 V_MAX=0.8 台架 --real)：0.5 时弯道速度是 0.5 m/s 时代的 1.6 倍，弯道误差 0.20 m(0.5 时 0.07)、反推 17 次；1.0 时误差 0.14、反推 3 次，总用时仍比 0.5 快 7%。(早先 0.8→0.5 的依据是 V_MAX=0.6)
inline constexpr double LOOKAHEAD    = 0.40;   // pure-pursuit 前瞻距离 (m)
inline constexpr double CARLIKE_LOOKAHEAD = 0.50; // 车式探索前瞻下限(m)：急弯/低速用它，不切弯贴柱
inline constexpr double CARLIKE_LOOKAHEAD_TIME = 1.10; // 随速度前瞻 = 此时长(s)×上一拍前进指令，限在 [CARLIKE_LOOKAHEAD, CARLIKE_LOOKAHEAD_MAX]；0=固定前瞻
inline constexpr double CARLIKE_LOOKAHEAD_MAX = 0.90;  // 2026-09-24 台架：0.8 m/s 下固定 0.5 m 前瞻只剩 0.6 s；直线高速前瞻 0.9 时 --real 航向翻转 -38%、贴线误差 -38%，
                                                      //   但固定 0.9 会让急弯切内侧(单测 tight_arc 误差 0.19 超限)，所以随速度：急弯低速仍是 0.5
inline constexpr double ENDPOINT_SLOW_R = 0.50;// 距终点此半径内开始线性降速 (m)
inline constexpr double ENDPOINT_SLOW_V = 0.50; // 减速半径处速度(m/s)：半径内线性停车斜率=此值/ENDPOINT_SLOW_R，与 V_MAX 解耦(0.5 下已实机验证)；半径外按 CARLIKE_MAX_DECEL 反推提前减速

inline constexpr double KP_YAW       = 1.60;   // 朝向误差 → yaw_rate 的 P
// ★KD 口径修正(2026-08)★：此前 tracker 内把 D 项 dt 写死 0.05，而主循环真实周期是 0.02
//   (TIMER_PERIOD_MS=20) → 微分项恒为真值的 0.4 倍，KD_YAW=0.50 实际只等效 0.20。
//   现 dt 改为按 TIMER_PERIOD_MS 正确推导，KD 同步缩放 ×0.4 使【输出逐位不变】：
//     KD_YAW 0.50→0.20    KD_LAT 0.10→0.04
//   即这两个数只是把原先隐藏在错误 dt 里的真实增益显式写了出来，飞行手感与之前完全一致。
//   ★以后调参请以这两个新值为基准★；改 TIMER_PERIOD_MS 时阻尼会正确跟随，不再隐性跳变。
inline constexpr double KD_YAW       = 0.20;   // yaw_rate 的 D（口径修正后的真实值，等价于旧 0.50@dt=0.05）
inline constexpr double MAX_YAW_RATE = 1.80;   // yaw_rate 限幅 (rad/s)
inline constexpr double CARLIKE_KD_YAW = 0.50; // 车式纠偏使用实测角速度阻尼，不对跳动的目标角求导
inline constexpr double CARLIKE_MAX_YAW_RATE = 1.10; // 大角度纠偏的转向速度上限(rad/s)。0.8→1.1(2026-09-23，用户要求转弯更快；台架 --real 用时 -5%、反推减少，滞后×1.5 下持平)
inline constexpr double CARLIKE_MAX_ACCEL = 0.30; // 纠偏完成后的前进加速度上限(m/s²)
inline constexpr double CARLIKE_MAX_DECEL = 0.60; // 巡航前进指令减速斜坡上限(m/s²)；弯道/终点减速由前瞻速度剖面提前开始，避免一拍跌到近零
inline constexpr double CARLIKE_CURVATURE_FF = 1.00; // 弯道角速度前馈 v·κ 的增益(0关闭)；纯P跟踪在弯道必然滞后切外侧
inline constexpr double CARLIKE_CREEP_SPEED = 0.12; // 终点/尖角前线性停车律(v=V_MAX·d/ENDPOINT_SLOW_R)的最低爬行速度(m/s)，防止与曲率门叠加后趴窝(基线仿真曾以 0.02 m/s 爬 14 s)
inline constexpr double CARLIKE_HANDOFF_BRAKE_SPEED = 0.15; // 需要停车转向的换路(掉头/观察点换向)先沿旧路减到此速度以下再采纳(m/s)，防止惯性冲离新路线
inline constexpr double CARLIKE_HANDOFF_BRAKE_ANGLE_DEG = 45.0; // 巡航中换路，新路线前瞻方位误差超过此角即先预减速再采纳(度)；预演一拍常低估 10° 左右，故低于停车线 75°
inline constexpr double CARLIKE_MAX_YAW_ACCEL = 1.60; // 转向角加速度上限(rad/s²)。1.2→1.6 与 MAX_YAW_RATE 同步提高
inline constexpr double CARLIKE_YAW_FILTER_S = 0.12; // 无外部角速度测量时的航向差分低通(s)
inline constexpr double CARLIKE_PREDICTION_S = 0.20; // 根据惯性速度提前修正前瞻方位(s)
inline constexpr double CARLIKE_LATERAL_PREDICTION_S = 0.55; // 沿路线法向预测横向惯性(s)：适度提前收住越线，不放大偏航P增益
inline constexpr double CARLIKE_ALIGN_RESUME_DEG = 25.0; // 真正大角度停车纠偏后进入此角度即可继续转弯前进
inline constexpr double CARLIKE_ALIGN_STOP_SPEED = 0.06; // 兼容旧配置保留，不再阻塞转向/尖角/重规划
inline constexpr double CARLIKE_ALIGN_STOP_YAW_RATE = 0.12; // 转向惯性降至此角速度后才前进(rad/s)
inline constexpr double CARLIKE_ALIGN_SETTLE_S = 0.06; // 真正大角度恢复的短确认时间(s)
inline constexpr double CARLIKE_STOP_ALIGN_DEG = 75.0; // 超过此夹角才停车转向；普通弧线只减速
inline constexpr double CARLIKE_CORNER_STOP_DEG = 20.0; // 未能安全平滑的真实折线尖角仍按顶点保守通过
inline constexpr double CARLIKE_MAX_LATERAL_ACCEL = 0.35; // 连续弯道向心加速度上限(m/s²)

// ★先转再走·朝向门控★：防"规划出身后/大角度路径时,机头还没转过来飞机就带着机体平移甩出线外撞柱"。
//   机头到目标方向的偏差 |e_yaw| > HEADING_GATE_DEG → 前进+横向全清零,只转 yaw_rate(原地转身);
//   阈值内用 cos(e_yaw) 平滑门控(乘到前进和横向上)。正常巡航 e_yaw 很小≈不影响,只大角度才压。
//   ★前进和横向都乘这个门(关键)★:光压前进不压横向,飞机仍会沿线法向侧移甩出去。
inline constexpr double HEADING_GATE_DEG = 65.0; // 全向模式门限：机头偏离超此角度则原地转身(度)
inline constexpr double CARLIKE_HEADING_GATE_DEG = 18.0; // 普通纠偏的减速起点，停车阈值见CARLIKE_STOP_ALIGN_DEG
inline constexpr double CARLIKE_TURN_BLEND_M = 0.80; // 换线过渡长度(m)：当前方向到下一条线用 Bézier 丝滑衔接
inline constexpr double CARLIKE_TURN_BLEND_ANGLE_DEG = 15.0; // 转角小于此值不额外插入过渡段
inline constexpr double TURN_BLEND_MAX_ANGLE_DEG = 120.0; // 超此夹角不构造短Bezier过渡，用已校验原路径交tracker对齐
// ★锥内钩子判定(2026-09-23)★：机头锥内的 A* 解若在起步 CONE_HOOK_WINDOW_M 内就要偏离机头 ≥ CONE_HOOK_DEG 且执行曲率 ≥ CONE_HOOK_KAPPA，
//   说明是锥约束硬凑出的"先直走再直角急拐"(r01：观察点掉头时沿钩子以 0.07 m/s 爬 7 s)。视为锥内无解，
//   交给无锥路径(≤TURN_BLEND_MAX_ANGLE_DEG 由 tracker 停车转向)或原地转身找解，决策明确。0=关闭。
inline constexpr double CARLIKE_CONE_HOOK_DEG = 70.0;
inline constexpr double CARLIKE_CONE_HOOK_WINDOW_M = 1.20;
inline constexpr double CARLIKE_CONE_HOOK_KAPPA = 4.0; // 且执行曲线起步段最大曲率 ≥ 此值(1/m，半径≤0.25m)才算钩子；正常绕障弧 κ≈1~3，r01 钩子 κ=12
inline constexpr double TURN_ROUND_MIN_M = 0.25; // 大圆角因障碍逐步缩小时的切点距离下限(m)；短引导段可更小，由实际曲率/碰撞检查决定能否圆滑通过
inline constexpr double TURN_ROUND_MAX_CURVATURE = 5.0; // 圆角最大允许曲率(1/m)，不靠微小急弯假装平滑
inline constexpr int    CARLIKE_TURN_BLEND_SAMPLES = 12; // Bézier 过渡段采样点数

inline constexpr double KP_LAT       = 0.80;   // 横向偏差 → v_lat 的 P
inline constexpr double KD_LAT       = 0.04;   // v_lat 的 D（口径修正后的真实值，等价于旧 0.10@dt=0.05，见上方 KD_YAW 说明）
inline constexpr double MAX_V_LAT    = 0.20;   // 横向纠偏速度上限 (m/s)，压很低
inline constexpr double CARLIKE_DRIFT_KD = 0.60; // 普通前进时实测横漂速度的阻尼（不改旧全向KD_LAT）。0.8→0.6(2026-09-23 台架)：
                                                 //   0.4 时实机噪声下横向指令翻转最少，但滞后×2 时纠偏不足、误差比旧版大 5 cm；0.6 兼顾
// ★跟踪平稳性(2026-09-23，离线台架 ~/桌面/test/sim/bench + 仿真验证)★
//   实机"总有点晃"的机理：SLAM 厘米级位置噪声经"实测速度外推 + 前瞻几何"进入航向误差，每帧阶跃，
//   KP_YAW 放大后期望角速度每拍跳 0.1 rad/s，指令只受角加速度限幅 → 满幅锯齿；滞后 0.5 s 时直线上增幅振荡。
inline constexpr double CARLIKE_SMITH_S = 0.30;     // Smith 预估：用最近此时长【已发出】速度指令的积分预测位置(s)；0=回到实测速度外推
inline constexpr double CARLIKE_SMITH_YAW_S = 0.40; // Smith 预估：用最近此时长已发出角速度指令预测机头朝向(s)。0.2→0.4(2026-09-24 台架 V_MAX 0.8：仿真航向响应滞后约 0.38 s；--real 用时 -2%、滞后×1.5 前进反推 37→20 次)
inline constexpr double CARLIKE_COMMAND_FILTER_S = 0.20; // 巡航期望角速度/横漂补偿目标的一阶低通(s)；0=关闭
// 横向滑移减速 v *= 1/(1+GAIN·max(0,|横向速度|低通-DEADBAND))。旧版无死区无滤波：仿真 69% 时间由它限速，
//   实机横向速度估计噪声直接调制前进指令(前后点头)。台架实机模型：前进反复 -70%、jerk -24%、用时 -8%，贴线均值 +1 cm。
inline constexpr double CARLIKE_LATERAL_SLOW_GAIN = 3.0;
inline constexpr double CARLIKE_LATERAL_SLOW_DEADBAND = 0.02; // m/s。0.05 时滞后×2 下滑移减速反应迟、误差大；0.02 在×1 下贴线与旧版持平
inline constexpr double CARLIKE_LATERAL_SLOW_TAU = 0.30;      // s
// 停车转向/原地找解/观察点定点的锚点 = 预计滑停点：v²/(2·decel) + LATENCY·v，上限 MAX_M。
//   原 LATENCY 写死 0.10、MAX 0.20：定点从已发指令斜坡刹停后低估滑行，飞过锚点被回拉(r49 反向 -0.19 m/s、点头 -2.9°)。
inline constexpr double CARLIKE_STOP_ANCHOR_LATENCY_S = 0.35;
inline constexpr double CARLIKE_STOP_ANCHOR_MAX_M = 0.35;
// 航向角速度来源：雷达(MID360S)内置 IMU。实机不是 PX4 飞控，★严禁订阅 /mavros/imu/data★(话题存在但数据不正确，用户确认 2026-09-23)。
//   与实机 Point-LIO(mid360.yaml)同一话题。三轴角速度用 SLAM 姿态转到世界系取 z，雷达倾斜/倒装也正确；
//   与 SLAM 航向差分长期反号或幅值比偏离 [0.5,2] 时自动停用；断流超过 IMU_TIMEOUT_S 退回航向差分+EMA。空字符串=不订阅。
inline constexpr const char* IMU_TOPIC = "/livox/imu";
inline constexpr double IMU_TIMEOUT_S = 0.10;
inline constexpr double REQUIRED_PLAN_RESERVE_M = 0.08; // 必达点(归航/插点/视觉)规划先加此预留余量(m)，找不到再退回标准余量。
                                                    //   最短路折点紧贴禁入圈，平滑与圆角都会切进去→退回纯折线逐点停车爬行(r65 两柱间 16 s)；
                                                    //   离线复现：预留 ≥0.05 后平滑曲线即可通过标准余量验收。验收余量不变。
inline constexpr double REQUIRED_SLOW_RADIUS_M = 1.20; // 必达点线性减速半径下限(m)：覆盖 1.0m 末端批准直线+余量。
inline constexpr double REQUIRED_SLOW_V = 0.60; // 必达点减速半径处的速度(m/s)：半径内线性停车斜率=此值/半径，与 V_GOAL_MAX 解耦；半径外按减速度反推
inline constexpr double HOME_SLOW_RADIUS_M = 1.50; // 接近终点安全入口/绕障路点的减速范围(m)，不作为下降触发距离
inline constexpr double HOME_SLOW_V = 0.45;        // 接近安全入口时的速度上限(m/s)，与位置误差PD共同减速；关闭安全入口时仍用于旧归航跟踪器
inline constexpr double HOME_STRAIGHT_APPROACH_M = 1.00; // 终点安全入口圆半径(m)：同视觉目标按最近可达路程选点，到入口才锁实际yaw并直线斜降；已在圈内且直降安全则就地开始。0=关闭安全入口/同步降高，使用普通必达点避障
inline constexpr double HOME_APPROACH_V = 0.40;          // 终点直线接近的水平速度上限(m/s)，与视觉投放对准段一致；漂移纠偏不受机头夹角门控
inline constexpr double REQUIRED_DRIFT_KD = 0.80; // 必达点(归航/插点/视觉)跟踪的横漂阻尼，保持探索改 0.4 之前的值
inline constexpr double XY_BRAKE_ACCEL = 0.80; // 刹车/反向纠偏的指令变化上限(m/s²)，普通加速仍用CARLIKE_MAX_ACCEL

inline constexpr double V_EST_ALPHA  = 0.30;   // 位置差分估速度低通系数（同主控思路）

// ---------------------------------------------------------------------------
// 完成判定 + 归航刹停
//   覆盖率达标 → 进入"归航"：算法侧对到终点的位置误差跑 PD，平滑刹停，
//   到达有效终点位置容差即可交接，持续XY纠偏，不等待速度归零。
// ---------------------------------------------------------------------------
inline constexpr double DONE_COVERAGE   = 0.90;  // 完程度(已探索大格占比)达此值即"扫完"→归航
inline constexpr double SEARCH_COVERAGE = 0.95;  // 视觉投放模式下【投放未满 3 次】时的扫完阈值(用户定 95%)；投满 3 次或未开投放时按 DONE_COVERAGE(0.90，用户实测)；
                                                 //   投满 3 次立即归航；另有候选确认中/线索未确认/待投放目标时不收尾
inline constexpr double EXPLORE_BACKTRACK_AFTER_S = 5.0;  // 探索连续这么久(s)没有可执行路线(原地找解/A*无解) → 沿来时轨迹原路后退再换路(用户要求 2026-09-24，10→5 s)。0=关闭
inline constexpr double EXPLORE_BACKTRACK_M = 1.5;        // 每次原路后退的距离(m)，退完清空"够不到"黑名单重新规划
inline constexpr double GOAL_TOL_XY     = 0.10;  // 到终点位置容差 (m)，精确停所以收紧
inline constexpr double GOAL_STOP_V     = 0.05;  // 速度安全投影的噪声阈值(m/s)，不再作为到点交接条件

// 归航/POI末段速度制动：方向始终由安全轨迹给定，剩余弧长与实测速度仅调整速度大小
inline constexpr double KP_GOAL     = 1.20;   // 位置误差 → 速度的 P (1/s)
inline constexpr double KD_GOAL     = 0.80;   // 速度阻尼 D（越大刹得越稳、越不冲）
inline constexpr double V_GOAL_MAX  = 0.75;   // 必达段(归航/视觉接近/插点)速度上限 (m/s)。0.6→0.75(2026-09-24 仿真第 7 轮：平均全程 -10%)；末端刹停由 REQUIRED_SLOW_V 保持 0.6 时的手感
inline constexpr double REQUIRED_CONNECTOR_LENGTH_M = 1.00; // 归航/POI到真目标的显式末端直线最大长度(m)，不放松障碍余量
inline constexpr double REQUIRED_MOTION_INFLATE = 0.0;  // 仅速度刹停投影的内层余量(m)，限制到[0,global_margin]；规划保持原余量。0=只留拟合充气+机体半径(0.4m)
inline constexpr double REQUIRED_PATH_EXEC_INFLATE = 0.0;  // 【正在执行】的必达路径复核用的内层余量(m)，限制到[0,global_margin]。0=执行圈0.4m，比规划圈小0.1m
                                                          //   规划、新路验收仍用 GLOBAL_MARGIN；执行中转弯切外偏进规划圈但没进执行圈时
                                                          //   不再把整条路作废→不会 hold 零速+反复重规划卡在障碍旁；真进执行圈才按原规则处理
inline constexpr double REQUIRED_BLOCKED_REPLAN_S = 0.50; // 速度安全检查连续阻挡此时长后重搜(s)，漂移不重置；0关闭

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
inline constexpr double ROBOT_RADIUS = 0.4;   // 飞机半径 (m)，碰撞判定+自身回波滤除都用它。真机实测 30cm

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
inline constexpr double GLOBAL_MARGIN    = 0.10;  // 障碍额外安全余量(m)：禁入半径 = 障碍r + ROBOT_RADIUS + 此值。
inline constexpr double EXPLORE_GUARD_INFLATE = 0.03; // 探索段速度守卫/定点纠偏短投影的执行余量(m)：规划仍用 GLOBAL_MARGIN，执行时允许跟踪误差吃掉规划余量的一部分，物理半径不可侵入
                                                  //   飞机中心到障碍物外表面的规划最小距离
                                                  //   = OBS_R_INFLATE(0.10) + ROBOT_RADIUS(0.30) + GLOBAL_MARGIN(0.10) = 0.50m。
                                                  //   2026-09-22 由 0.30 降到 0.10(外圈 0.7→0.5)：目标物常距障碍约 1m，0.7 会把目标物
                                                  //   本身吃进禁入区；执行/刹停内层余量同时改 0(见 REQUIRED_*_INFLATE)，执行圈 0.4m。
                                                  //   ★越大越早远绕越不贴障碍,但太大会把"柱墙之间的缝"吃掉→墙边柱子绕不过去★。
                                                  //   真机杆间距若不足则宁可卡死报警也不贴撞；墙边柱风险靠 RETREAT_* + relax 兜底。
inline constexpr double GLOBAL_WALL_MARGIN = 0.60;// 离墙安全距离(m)：cell 中心距任一场地边界<此值即禁入。
                                                  //   2026-06-04 0.80→0.60：原 0.8 偏宽,墙边柱子时把"柱墙之间能过的缝"吃掉→A*判无解、飞机
                                                  //   转头面向墙卡死。0.6 物理离墙余量=0.6-ROBOT_RADIUS(0.30)=0.30m 仍绝不撞墙；且 <WALL_MARGIN(1.0)
                                                  //   覆盖目标(离墙≥1.0m)仍直接可达。调大更躲墙但墙边缝更窄/近墙目标够不到。
inline constexpr double COMMIT_TARGET_TOL = 0.60; // ★路径承诺★目标移动超此距离(m)才重算A*换边。
                                                  //   核心防"绕障左右翻边"：旧路径仍无碰撞就续用,不因左右绕代价 near-tie 每拍翻边→撞柱。
                                                  //   仅【目标大幅移动】或【旧路径被挡(会撞)】才重算。调大→更"咬死"旧路径(更稳但换区迟钝);调小→更易换路。
inline constexpr double GLOBAL_LOOKAHEAD = 0.60;  // 沿绕障轨迹取 carrot 的前瞻距离(m)：越大越平顺但贴角余量小
// 探索路线剩余不足此距离即接力下一观察点(原与 GLOBAL_LOOKAHEAD 共用 0.6)。1.2(2026-09-23 仿真 L1~L4)：
//   总用时 567.7→524.6 s(-7.6%)，真值均速 0.33→0.38，到点停车再选明显减少；L1 单项慢 7 s(路线稍长)。
inline constexpr double EXPLORE_HANDOFF_M = 1.20;
inline constexpr double EXPLORE_TARGET_MIN_DIST = 1.50; // 探索挑 A* 终点的最小距离(m)：取覆盖路径上第一个
                                                  //   距当前≥此值的点当 A* 目标(而非最远点)——只绕近处障碍,路径短不易过期,到了周期重规划接力。

// ★换路评估(路径迟滞)★(2026-06-10)：COMMIT_TARGET_TOL 只在【目标几乎没动】时咬死旧路；一旦目标
//   漂移触发重算，旧逻辑会无条件采纳新 A* 结果 → 仍可能"一会想走左边、一会想走右边"反复横跳。
//   这里再加一道闸：重算出新路后，先给【旧路】和【新路】各打分(score_path: 剩余弧长=时间, 最小障碍
//   边距=安全)，只有新路【更安全】且/或【更快】超过下列比例，才真的换路；否则续用旧路。
//   关键：左右横跳的两条路是 near-tie(几乎等长、等安全) → 两项都达不到阈值 → 不换 → 横跳根除。
//   只有旧路【已被挡(会撞)】或【已快走完(剩余很短)】时跳过本评估，直接采纳新路(安全/进度优先)。
inline constexpr double PATH_SWITCH_SAFETY_GAIN  = 0.50; // 新路"最小障碍边距"需比旧路高 ≥此比例(50%)才算【更安全】。调大→更不爱为安全换路(更稳)
inline constexpr double PATH_SWITCH_TIME_GAIN    = 0.20; // 新路"剩余弧长"需比旧路短 ≥此比例(20%)才算【更快】。调大→更不爱为抄近路换路(更稳/更咬死旧路)
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
inline constexpr int    TURN_SOLVE_RESEARCH_EVERY = 10;   // 50Hz控制下每10拍(0.2s)检查路径；转向控制仍逐拍执行
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
// 偏入障碍执行圈时，先水平回到原安全路线。仅此恢复阶段使用，不能覆盖普通避障限速。
inline constexpr double ROUTE_REJOIN_MIN_MPS = 0.20; // 圈内回线速度下限(m/s)，受加速斜坡和RETREAT_V_MAX限制；出圈后按惯性减速，到线5cm内结束恢复
inline constexpr double RETREAT_V_MAX      = 0.35; // 后退限速(m/s)：低速蹭出来,别猛退甩飞。
// ★后退超时(2026-08)★ ← 修"界面冻结、终端反复刷'后退脱困(已退 0.00m)'、飞机悬在原地"
//   两个 bug 叠加导致：
//   ① do_retreat 旧实现把后退点交给 pd_core，而后退点在飞机【身后】(e_yaw≈180°) →
//      朝向门控 head_gate=0 → 速度恒为 0，只发 yaw_rate 让飞机原地掉头 180°。
//      实测要转 72 拍(1.44s)速度才解禁；期间飞机没动 → traveled 恒为 0 →
//      "traveled < RETREAT_MAX_DIST"永远成立 → 无限重试。(已改为直接投影到机体系，不掉头)
//   ② A* 误判无解：3 个障碍各距 0.8m 环绕时，relax 突围只放行 ±0.60m 方框，
//      出框即恢复"障碍r+0.60"禁入 → 膨胀圆叠加封死出路 → probe 也无解 → 落到 retreat 分支。
//   与"退够距离"是【或】关系，先满足哪个都退出本轮后退，并阻止原地重新计时；
//   位置显著改变或有新的有效路线后才允许重试。没有安全路线时仍可能持续等待。
//   调大→给退不动的情形更多机会；调小→更快放弃转去别处。<=0 关闭超时(退回旧行为，不推荐)。
inline constexpr double RETREAT_TIMEOUT_S  = 2.0;  // 进入后退态超此秒数仍没退够 → 判"退不动"，交上层跳带/悬停

}  // namespace params
}  // namespace exploration
