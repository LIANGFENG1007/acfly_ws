// ============================================================================
//  params.hpp  ── 控制相关常量
// ============================================================================

#pragma once

#include <cmath>

namespace fly_mission {
namespace params {

// ---------------------------------------------------------------------------
// 速度上限（控制器输出 setpoint 的最大幅度）
// ---------------------------------------------------------------------------  
inline constexpr double MAX_SPEED_XY         = 0.8;    // 水平最大总速度 (m/s)      
inline constexpr double MAX_SPEED_Z          = 0.6;    // 起飞/升降/降落 垂直最大速度 (m/s)
inline constexpr double MAX_SPEED_Z_LEVEL    = 0.1;   // 平飞时垂直微调最大速度 (m/s)，限制上下抖动
inline constexpr double MAX_YAW_RATE         = 0.8;    // 最大转头角速度 (rad/s) 

// ---------------------------------------------------------------------------
// PD 位置控制增益
// 控制律：v = Kp * (target - current) - Kd * current_velocity
// ---------------------------------------------------------------------------
inline constexpr double KP_XY                = 0.8;    // 水平 P
inline constexpr double KD_XY                = 0.20;   // 水平 D
inline constexpr double KP_Z                 = 0.6;    // 垂直 P（调小，减少激进矫正）
inline constexpr double KD_Z                 = 0.20;   // 垂直 D（加大阻尼，抑制上下抖动）
inline constexpr double KP_YAW               = 0.7;    // yaw 误差 → yaw_rate

// 位置差分估速度低通滤波系数（0~1）：
//   v_est = α * inst_v + (1-α) * v_est
//   α 大响应快但噪声大，α 小平滑但有滞后
inline constexpr double V_EST_ALPHA          = 0.3;

// ---------------------------------------------------------------------------
// 环绕飞行（description_circle_right）专用参数
// 独立于上面的通用参数，单独调，互不影响
// ---------------------------------------------------------------------------
inline constexpr double CIRCLE_MAX_SPEED_XY  = 0.6;    // 环绕时水平速度上限 (m/s)
inline constexpr double CIRCLE_KP_YAW        = 0.8;    // 环绕时 机头朝向圆心 的 yaw P 增益
inline constexpr double CIRCLE_MAX_YAW_RATE  = 1.0;    // 环绕时 最大转头角速度 (rad/s)

// ---------------------------------------------------------------------------
// 到位判定（位置 / yaw 进入容差并持续稳定 N 秒）
// ---------------------------------------------------------------------------
inline constexpr double TOL_XY               = 0.15;                  // ★正常飞行★水平容差 (m)：
                                                                      //   起飞/走航点/悬停/绕杆等所有动作用它。放宽→到点判定更早、走线更流畅、
                                                                      //   不为最后几厘米磨蹭。找图不用它，见下面 FF_TOL_XY。
inline constexpr double TOL_Z                = 0.10;                  // 垂直容差 (m)
inline constexpr double TOL_YAW              = 8.0 * M_PI / 180.0;    // yaw 容差 (rad)
inline constexpr double SETTLE_DURATION      = 0.3;                   // xyz 稳定时长 (s)
inline constexpr double SETTLE_DURATION_YAW  = 0.3;                   // yaw 稳定时长 (s)

// ---------------------------------------------------------------------------
// 着陆判定
// ---------------------------------------------------------------------------
inline constexpr double LAND_DONE_REL_HEIGHT = 0.10;   // 高度低于 home_z + 此值 且已上锁 = 着陆完成

// ---------------------------------------------------------------------------
// 主循环周期
// ---------------------------------------------------------------------------
inline constexpr int    TIMER_PERIOD_MS      = 20;     // 50Hz

// ---------------------------------------------------------------------------
// ★★★ OFFBOARD setpoint 预热流 ★★★
//   false = ★当前★ 不预热：IDLE(任务未启动/已停止) 与 无位姿 时【什么都不发】。
//           雷达就绪 + 飞手手动切 OFFBOARD → BOOT_CHECK 当拍走到 takeoff()，
//           从那一拍起才开始发 setpoint（"直接发数据起飞"）。
//   true  = 预热：上述两种情况每拍发零速度 setpoint 占位(原行为)。
//
//   ★关掉的风险★：许多固件(PX4 系)要求【切 OFFBOARD 之前已有 setpoint 流】，
//   否则拒绝切入、或刚切进去就因"没收到 setpoint"退出 OFFBOARD。症状：
//       · 飞手切 OFFBOARD 切不进去 / 一进就跳回 POSCTL
//       · 或解锁后立刻失锁(主控 LOST_DEBOUNCE 判定飞手接管 → 任务中止)
//   ★真出现以上症状，把这里改回 true 重编即可恢复★。ACFly 是否有此要求需实机确认。
//
//   注：任务中止(stop()→IDLE)后也不再发 setpoint，飞控将按其失联保护动作处理。
// ---------------------------------------------------------------------------
inline constexpr bool   OFFBOARD_PREHEAT = true;

// ---------------------------------------------------------------------------
// ★★★ 起飞用位置环（"打点上去"）★★★
//   true  = 起飞段(TAKEOFF + WAIT_AFTER_TAKEOFF)把【目标位置】直接发给飞控，
//           由飞控内部位置环飞上去；起飞后悬停稳定 → 切走时自动恢复速度环 PD。
//           即：起飞"打点上去"，之后走航点/找图/绕杆/钻圈全部照旧用速度环 PD。
//   false = ★当前★ 全程速度环 PD(起飞也跑 PD)，与加这个开关之前的行为逐位一致。
//
//   ★为什么只有起飞用★：起飞是最需要"稳"的一段，交飞控位置环最省心；平飞走航点
//   用 PD 保留现有调参手感(KP_XY/KD_XY 等)与所有既有功能。
//
//   ★哪些参数在起飞段不生效★(它们只喂 PD)：MAX_SPEED_Z / KP_Z / KD_Z / V_EST_ALPHA。
//     起飞爬升快慢由【飞控自身位置环参数】决定，改这里的 MAX_SPEED_Z 无效。
//   ★仍然生效★：TOL_XY/TOL_Z + SETTLE_DURATION(到位判定只看实际位姿，与发的是
//     位置还是速度无关) → 状态机 takeoff()/is_reached() 语义完全不变。
//
//   ★前提★：飞控 local 位置估计与 SLAM/camera_init 同源。本机满足——acfly.launch.py
//   把 /mavros/odometry/out 重映射到 /aft_mapped_to_init，Point-LIO 位姿即飞控的
//   外部里程计，故可直接发 SLAM 系坐标。若这条链路变了，此开关必须重新评估。
// ---------------------------------------------------------------------------
inline constexpr bool   TAKEOFF_POSITION_MODE = false;

// ---------------------------------------------------------------------------
// ★ 视觉数据通道（shm 信箱，延迟优化第3步）★
//   true  = 视觉结果从共享内存信箱(/dev/shm/uav_cv_out)读，不再订阅 /cv/target_info——
//           本机内存直读(纳秒级)，去掉 DDS/rclpy 一段延迟。视觉端(finding_new1.2.1.py)
//           双写(shm+ROS)不用改，布局见 shm_mailbox.hpp。
//   false = 回退老路：订阅 /cv/target_info(JSON)。随时一键切回。
// ---------------------------------------------------------------------------
inline constexpr bool   USE_SHM_CV           = true;

// ---------------------------------------------------------------------------
// ★ 视觉找图（FindFigure）★  ← 找图调参都在这一段
//   起飞稳定后后台监听 /cv/target_info：同色+同形+坐标接近的检测连续累计满
//   FF_CONFIRM_FRAMES 帧 → 算术平均出图形中心 → 打断走线飞过去 → 悬停 → 拉黑 →
//   回被打断状态。中心落在已拉黑圆内则忽略；单次超时则放弃但仍拉黑。
//   均可运行期 -p ff_* 覆盖（见 find_figure.cpp 构造）。
// ---------------------------------------------------------------------------
inline constexpr int    FF_CONFIRM_FRAMES = 6;    // 连续确认帧数：越大越稳(误检难触发)但反应越慢
// ★同时累计的目标上限★：一画面可能有多个动物，每个各自独立累计确认(互不干扰)。
//   超过此数的新目标本帧丢弃(防误检爆内存)。设 >= 场地里可能同时看到的目标数。
inline constexpr int    FF_MAX_ACCUM      = 16;
// 类别编号 → 名字，★仅用于日志好读★(不参与任何判定)。须与视觉端 shm_class_ids 对应；
//   对不上只是日志名字不对，不影响功能(判定只用编号本身)。视觉端当前配置：
//   "monkey:1,peacock:2,tiger:3,wolf:4,elephant:5"
inline const char* ff_class_name(int id)
{
    switch (id) {
        case 1: return "monkey";
        case 2: return "peacock";
        case 3: return "tiger";
        case 4: return "wolf";
        case 5: return "elephant";
        default: return "";      // 未知编号：日志里直接显示编号
    }
}
// ★找图专用水平容差 (m)★：只在【飞向图形中心】那一段生效，比正常飞行 TOL_XY 收紧——
//   要拍/看清图形得停得更准；而正常走航点用宽的 TOL_XY 图流畅。两者互不影响：
//   实现方式是找图态改用 drone_.is_reached_tol(FF_TOL_XY)，其余状态照用 is_reached()。
//   ★必须 <= TOL_XY★(否则找图比正常飞还松，失去收紧的意义)；也别设太小(0.05 以下 SLAM
//   噪声会让 SETTLE_DURATION 一直被打断→永远判不到位→只能靠 FF_TIMEOUT_SEC 超时收场)。
inline constexpr double FF_TOL_XY         = 0.10;  // 找图到达容差 (m)：飞到图形中心多近算到
inline constexpr double FF_ASSOC_DIST     = 0.20;  // 跨帧同一目标的坐标关联阈值 (m)：越小越严(易断累计)，越大越易把相邻目标误当同一个
inline constexpr double FF_BLACK_RADIUS   = 0.20;  // 拉黑半径 (m)：以图形中心为圆，圆内的再确认目标一律忽略
inline constexpr double FF_HOVER_SEC      = 0.30;  // 到点悬停时长 (s)
inline constexpr double FF_TIMEOUT_SEC    = 3.00;  // 单次找图总超时 (s)：超过直接放弃(但仍拉黑)，回被打断状态
inline constexpr double FF_FRAME_GAP_SEC  = 0.20;  // 断帧阈值 (s)：两帧间隔超此认为目标暂时消失→累计清零(防残留计数)

// ---------------------------------------------------------------------------
// ★ 视觉寻线（LineFollower）★  ← 沿黑线飞的调参都在这一段
//   起飞后直接进入 FOLLOW_LINE：后台监听 /cv/target_info 的 line_x/line_y(机体系前视点,
//   前 dx=line_x / 左 dy=line_y，单位 m)。★方案：机头不转(yaw=0)★
//     · 前进 v_fwd = 纯 P: KP_X*line_x，钳到 [0, V_FWD_MAX]。前视点越远(直线,line_x≈0.5)
//       越快、越近(急弯,line_x≈0.05)越慢 → 急弯自动减速，给横向纠偏留时间(治"X冲太快Y跟不上")。
//     · 横向 v_lat = PD: KP_LAT*line_y + KD_LAT*d(line_y)/dt，把机身平移贴回线上。
//   line_x/line_y 都做低通+只在视觉新帧重算(滤抖、消 D 脉冲)。连续 LF_TIMEOUT_SEC 无有效线→降落。
//   均可 -p lf_* 覆盖。
// ---------------------------------------------------------------------------
inline constexpr double LF_KP_LAT         = 1.20;  // 前后+横向偏差→速度 的 P(★x/y共用★)：越大回线越快/前进越快(过大超调/左右晃)
inline constexpr double LF_KD_LAT         = 0.50;  // 偏差变化率→速度 的 D(★x/y共用★,阻尼)：压超调/振荡。晃就调大，迟钝就调小
inline constexpr double LF_Y_ALPHA        = 0.40;  // line_y/line_x 低通系数(0~1)：新值权重。越小越平滑(滤视觉抖，但滞后)，越大越跟手
inline constexpr double LF_MAX_V_LAT      = 0.70;  // 前后+横向 速度限幅 (m/s)(★x/y共用★)：前进"多快"的天花板，慢就调大
inline constexpr double LF_TIMEOUT_SEC    = 1.00;  // 连续无有效线超此时长 → 降落 (s)
inline constexpr double LF_MIN_VALID_M    = 0.02;  // line 向量模长 < 此视为"无有效线"(判 0)：视觉丢线发 0，容一点噪声

// ---------------------------------------------------------------------------
// ★ 钻圈（RingDriller / DRILL_RING）★  ← 钻圈调参都在这一段
//   进入 DRILL_RING：原地悬停订阅 /ring_detector/center，攒够 DRILL_COLLECT_FRAMES 帧
//   (每帧=检测端已多帧累积+圆拟合的一次结果) 对 xyz + 环面朝向取平均 → 算环心。
//   动作序列(均正对环面、目标高度=环心z+DRILL_Z_OFFSET_M[补雷达在机身上方]，MOVE_POSE 边飞边转升高)：
//     ① 飞到"环前 DRILL_STANDOFF_M 米"对准 → ② 悬停 DRILL_HOVER_SEC 秒
//     → ③ 穿过圈飞到"环后 DRILL_STANDOFF_M 米" → ④ 悬停 DRILL_HOVER_SEC 秒 → (状态机)降落。
//   采集超过 DRILL_COLLECT_TIMEOUT_S 秒：够 DRILL_MIN_FRAMES 帧则用现有帧继续，
//   否则判定没看到圆环、放弃(降落)。
//   注：到位判定复用 drone_ 的 MOVE_POSE 到位(TOL_XY/TOL_Z/TOL_YAW + SETTLE_DURATION)。
// ---------------------------------------------------------------------------
inline constexpr int    DRILL_COLLECT_FRAMES    = 30;    // 悬停采集帧数(平均去抖)：越大越稳越慢
inline constexpr double DRILL_STANDOFF_M        = 1.0;   // 环前/环后停靠点距环心多少米(沿环面法向)
inline constexpr double DRILL_Z_OFFSET_M        = 0.30;  // ★高度抬升★ 目标z=环心z+此值。SLAM位姿点(雷达)在机身中心
                                                         //   上方，直接对准环心会使机身偏低蹭下沿；抬高≈雷达到机身中心垂距，
                                                         //   让机身中心穿过环心。sim:lidar 在 base_link 上方0.12m。正=抬高。
inline constexpr double DRILL_HOVER_SEC         = 0.5;   // 每次到位后悬停时长 (s)：环前、环后各悬停一次
inline constexpr double DRILL_COLLECT_TIMEOUT_S = 5.0;   // 采集超时 (s)：超过仍不足下面帧数→放弃
inline constexpr int    DRILL_MIN_FRAMES        = 5;     // 超时后至少要有的帧数才敢用；否则放弃


// ---------------------------------------------------------------------------
// ★ 绕杆环绕（description_circle_right 无参版）★  ← 绕杆调参都在这一段
//   description_circle_right() 现在【不传参】，内部自带子状态机：
//     ① 原地悬停订阅 /pole_detector/center 攒够 POLE_CIRCLE_COLLECT_FRAMES 帧，
//        对 杆水平轴心(x,y) + 半径 取平均(半径>POLE_CIRCLE_MAX_RADIUS 判非杆)。
//     ② 飞到"距杆表面 POLE_CIRCLE_STANDOFF_M 米"处(=距杆心 杆半径+standoff)，
//        target_pose_slam 边转边动、机头朝杆、★只飞 xy，z 锁进入时高度★。
//     ③ 以杆心为圆心、②的接近半径为半径，绕杆环绕 POLE_CIRCLE_SWEEP_DEG 度
//        (复用圆周闭环，机头始终朝杆，z 不动)。
//   采集超 POLE_CIRCLE_COLLECT_TIMEOUT_S 秒仍不足 POLE_CIRCLE_MIN_FRAMES 帧 → 放弃(停)。
//   环绕线速度用 POLE_CIRCLE_SPEED，并受上面 CIRCLE_MAX_SPEED_XY 安全封顶。
// ---------------------------------------------------------------------------
inline constexpr int    POLE_CIRCLE_COLLECT_FRAMES    = 30;   // 悬停采集帧数(平均去抖)
inline constexpr double POLE_CIRCLE_STANDOFF_M        = 1.0;  // 环绕点距【杆表面】多少米(离杆心=杆半径+此值)
inline constexpr double POLE_CIRCLE_SPEED            = 0.5;  // 绕杆线速度 (m/s)
inline constexpr double POLE_CIRCLE_SWEEP_DEG        = 360.0;// 绕杆扫过角度 (度)，360=绕一圈
inline constexpr double POLE_CIRCLE_MAX_RADIUS       = 0.15; // 杆半径拟合上限(m)：超此判非细杆、放弃
inline constexpr double POLE_CIRCLE_COLLECT_TIMEOUT_S = 5.0; // 采集超时 (s)：超过仍不足下面帧数→放弃
inline constexpr int    POLE_CIRCLE_MIN_FRAMES       = 5;    // 超时后至少要有的帧数才敢用；否则放弃


// ---------------------------------------------------------------------------
// ★ 全局避障（Obstacle Avoidance）★  ← 避障调参都在这一段
//   位置类动作(HOLD/TAKEOFF/MOVE_XY/MOVE_Z/MOVE_POSE/TURN_YAW)统一在 PD 出口做避障：
//   订阅 /multi_pole_detector/center 把每根杆当圆形障碍(膨胀半径=杆半径+安全裕度)。
//   直线路径被挡→构造绕行【二次贝塞尔】、把曲线弧长前瞻点喂给 PD；不挡→正常直线。
//   ★target_x_/y_ 全程不改，只改 PD 追的"有效目标"★ → is_reached()/各原语语义零变。
//   EXTERNAL_VEL(寻线/探索)/IDLE/LAND 在 PD 之前就 return，天然不受影响；
//   CIRCLE 显式排除(被绕的杆自己就是障碍，避它会毁掉圈)。纯数学在 avoidance.hpp/.cpp。
// ---------------------------------------------------------------------------
inline constexpr bool   AVOID_ENABLE          = false;  // ★总开关★：false=行为与加避障前逐位一致
inline constexpr double AVOID_TRIGGER_DIST_M  = 1.1;   // ★何时开始绕★：飞机离挡路杆心>此→照直飞(远处不斜)；≤此才启贝塞尔绕行
inline constexpr double AVOID_SAFETY_MARGIN_M = 0.30;   // 膨胀量：inflate=杆半径+此(含机身半宽+缓冲)
inline constexpr double AVOID_CLEARANCE_M     = 0.40;   // 额外余量：贝塞尔顶点离杆心=inflate+此
inline constexpr double AVOID_REJOIN_AHEAD_M  = 0.50;   // ★绕完多快回航线★：贝塞尔终点=原航线上"过杆后 此米"处(不再斜奔远处航点)。越小越贴、回得越急
inline constexpr double AVOID_LOOKAHEAD_M     = 0.80;   // 沿曲线弧长前瞻(喂 PD 的点离当前多远)；影响绕行速度/贴合
inline constexpr double AVOID_OBSTACLE_TTL_S  = 0.50;   // 障碍时效(s)：超此没新帧→不避障(防绕看不见的幽灵杆)
inline constexpr double AVOID_MAX_RADIUS      = 1.00;   // 半径>此的拟合直接丢(坏拟合/把墙当大圆)；通用障碍防御
inline constexpr int    AVOID_BEZIER_SAMPLES  = 20;     // 贝塞尔弧长采样段数(前瞻点定位精度)
inline constexpr double AVOID_ARRIVE_SLACK_M  = 0.4;   // ★航点被杆占时的放宽到达★：航点在杆膨胀圈内，且飞机已到"杆边缘+此"内→算到达、推进下一点(越大越早判到达、离杆越远越安全)


// ---------------------------------------------------------------------------
// ★ 自主探索（EXPLORATION 状态）★
//   探索期间飞机速度【完全由 exploration_planner 节点给】(话题 /exploration/cmd_vel)，
//   主控只做转发。本参数 = "算法停发多久算它挂了"。
//   ★为什么需要它★：ext_cmd_valid_ 是一次性锁存标志(收到过就永远为真)，若不判数据
//   新鲜度，算法节点挂掉后主控会每拍拿【最后那条速度】继续转发，而转发动作本身会刷新
//   控制器内部看门狗的时间戳 → 那个 0.3s 看门狗永远不触发 → 飞机带着最后的速度一直
//   飞下去。加了本超时后：算法停发 → 主控停止转发 → 控制器看门狗超时 → 只保高悬停。
//   取值：算法 50Hz 发(20ms 一条)，0.5s 容得下 25 帧抖动，又能较快发现真停了。
// ---------------------------------------------------------------------------
inline constexpr double EXPLORE_CMD_TIMEOUT_S = 0.5;


// ---------------------------------------------------------------------------
// ★ 小车雷达追踪（TrackCar / TRACK_CAR 状态）★  ← 追踪调参都在这一段
//   场地里另有一台雷达装在地面小车上，跑自己【独立的一套 SLAM】，位姿发在
//   /aft_mapped_to_init2(nav_msgs/Odometry，与飞机的 /aft_mapped_to_init 同格式)。
//
//   ★为什么不能直接把收到的坐标当目标飞★：两套 SLAM 各自以【自己初始化的位置】
//   为原点，所以话题里报的是"小车雷达在【小车自己的】SLAM 系(记作 B 系)的坐标"，
//   而飞机的所有 target_* 原语吃的是"飞机 SLAM 系(记作 A 系)的坐标"。两者差一个
//   原点平移。两台雷达【摆放朝向一致】(用户保证) ⇒ 两系坐标轴平行、只差平移：
//
//       p_A = p_B + CAR_ORIGIN_IN_DRONE_SLAM
//
//   其中 CAR_ORIGIN_IN_DRONE_SLAM = 【小车雷达初始化位置】在飞机 A 系下的坐标，
//   也就是下面这两个数——这是本功能唯一需要现场标定的量。
//
//   ★怎么量这两个数★(在两套 SLAM 都启动后、小车还没动之前)：
//     方法1(推荐)：让小车停在起飞点旁边，量它相对飞机雷达的【前/左】距离。
//       A 系 +x = 飞机雷达初始化时的机头方向，+y = 左。
//       例：小车停在飞机正前方 2m、左 0.5m → CAR_ORIGIN_X=2.0, CAR_ORIGIN_Y=0.5
//     方法2：两套 SLAM 都起好、小车未动时，读一次两个话题的当前位置，相减：
//       ros2 topic echo /aft_mapped_to_init  --field pose.pose.position -n 1   # 得 pA0
//       ros2 topic echo /aft_mapped_to_init2 --field pose.pose.position -n 1   # 得 pB0(应≈0)
//       CAR_ORIGIN = pA0(飞机当时位置) + 量出的"小车相对飞机的偏移" - pB0
//     ★量错的后果★：飞机会稳定地跟在"真实小车 + 偏移误差"处——误差多少就偏多少，
//     且不会自己收敛(没有任何闭环能纠正它)。先小车不动、看飞机停的位置对不对再放跑。
//
//   ★偏航★：同样只差平移不差旋转，所以小车的 yaw 可【直接】当飞机的目标 yaw 用，
//   不需要任何换算(前提仍是两台雷达摆放朝向一致；若差一个固定角，改 CAR_YAW_OFFSET_DEG)。
// ---------------------------------------------------------------------------
// ★小车雷达初始化位置在【飞机 SLAM 系】下的坐标 (m)★ ← ★现场标定，改这里★
inline constexpr double CAR_ORIGIN_X = 0.82;   // A 系 x (前为正)
inline constexpr double CAR_ORIGIN_Y = -0.39;   // A 系 y (左为正)
// 两台雷达安装朝向的固定角差 (度)：小车 yaw + 此 = 飞机目标 yaw。
//   摆放一致就填 0。若发现飞机机头总比小车偏固定角度，把那个角填进来。
inline constexpr double CAR_YAW_OFFSET_DEG = 0.0;
// 数据超时 (s)：连续这么久没收到 /aft_mapped_to_init2 → 判"没有新数据"，
//   悬停在最后一个目标点等它回来(不拿旧坐标硬飞)。收到新数据自动恢复追踪。
inline constexpr double CAR_DATA_TIMEOUT_S = 1.0;
// 追踪时是否跟随小车的 z：false=★当前★锁住进入追踪时的高度(小车在地上跑，它的 z
//   对飞机没意义)；true=保持固定相对高差(小车上下坡时飞机跟着起伏)。
inline constexpr bool   CAR_TRACK_Z = false;

// ---------------------------------------------------------------------------
// ★★★ 小车位姿的数据来源：UDP 还是 ROS 话题 ★★★
//   true  = ★当前★ 走【UDP 单播】(小车机跑 tools/udp_pose_sender，见
//           tools/README_udp_pose.md)。CarTracker 不再订阅 ROS 话题。
//   false = 走 ROS 话题 /aft_mapped_to_init2 (旧行为，逐位一致)。
//
//   ★为什么换 UDP★(2026-07-29)：两台机器同网时 Fast DDS 靠【组播】做发现，且是
//   "全连接"——每个节点都要和对面每个节点建连+周期心跳，连接数是【乘积关系】。
//   本机 mavros 有 50+ 插件节点，小车机一开就把 WiFi 组播挤爆(WiFi 组播按最低速率
//   发且不重传)，DDS 线程还抢 CPU → ★mavros 串口失去实时性、setpoint 发不出去★。
//   实测现象就是"小车机一启动 mavros 就不传数据了"。UDP 单播无发现/无心跳/点对点，
//   彻底没有这个问题。★还必须给两台机器设不同 ROS_DOMAIN_ID★，否则 DDS 照旧互相发现。
//
//   坐标换算(CAR_ORIGIN_*/CAR_YAW_OFFSET_DEG)与超时逻辑两种来源【完全共用】，
//   换来源不影响标定值。
// ---------------------------------------------------------------------------
inline constexpr bool   CAR_USE_UDP  = true;
// UDP 监听端口：必须与小车机 udp_pose_sender 的 -p port:= 一致
inline constexpr int    CAR_UDP_PORT = 9870;

// ---------------------------------------------------------------------------
// ★★★ 启动指令(BEEP① 之后等的那个"1")的来源：UDP 还是 ROS 话题 ★★★
//   BOOT_CHECK 流程：飞控+雷达就绪 → BEEP① → ★等启动指令★ → BEEP② → 等 OFFBOARD → 解锁
//   true  = ★当前★ 走【UDP 单播】收命令(端口 CMD_UDP_PORT)，不订阅 /mission/start。
//   false = 走 ROS 话题 /mission/start (Int32 data==1)，旧行为逐位一致。
//
//   ★为什么也换 UDP★：只要还有一个 ROS 话题跨机，两台机器的 DDS 就仍要互相发现，
//   组播风暴照旧存在(见 CAR_USE_UDP 注释)。位姿换了 UDP 但命令还留在 ROS 话题上，
//   等于白改。所以命令一起走 UDP。
//
//   ★丢包怎么办★：UDP 不保证送达，而启动指令丢了就起飞不了。策略与原 ROS 版一致——
//   ★发送端持续重复发★(1s 1 次)，接收端【收到一次即锁定】、重复的忽略。
//   原来的用法也是 "ros2 topic pub -r 1 ..." 持续发，所以操作习惯不变。
//
//   ★端口必须与位姿通道不同★：位姿 9870 / 命令 9871。同端口不能被两个 socket 都 bind。
// ---------------------------------------------------------------------------
inline constexpr bool   CMD_USE_UDP   = true;
inline constexpr int    CMD_UDP_PORT  = 9871;


// ---------------------------------------------------------------------------
// ★★★ 追上判定 + 视觉锁定投掷（LOCK_DROP 状态机）★★★
//
//  完整流程(★全程保持一个高度，不下降★ —— 需求 2026-08)：
//    TRACK_CAR(雷达追踪，高度 = 起飞后悬停时锁定的追踪高度)
//      └─ 水平距离 ≤ CATCH_DIST 起累加时间；累计达 CATCH_HOLD_SEC → 切 LOCK_DROP
//         (切换那一刻把当时高度锁成 lock_z_，之后全程保持)
//    LOCK_DROP(视觉精确锁定，★高度不变★)
//      ├─ 位置：★用 shm 信箱的 dx/dy(机体系)★ 换算成 SLAM 目标点(收帧时冻结)
//      ├─ 偏航：★仍用小车雷达(UDP)的 yaw★(shm 里只有 dx/dy，没有 yaw)
//      ├─ 高度：★保持 lock_z_ 不变★(不下降)
//      └─ hypot(dx,dy) ≤ DROP_DIST → 串口发 "DIANCI"(非阻塞)
//         → 再多锁定 DROP_HOLD_SEC → 以【当前高度】飞回 (0,0) → 降落
//
//  ★"追上计时"的口径★：距离超出 CATCH_DIST 时【暂停累加但不清零】(按用户选择)，
//    即允许断断续续凑满 CATCH_HOLD_SEC。要改成"必须连续保持"就在状态机里把
//    catch_timer_ 清零(代码里已标注位置)。
// ---------------------------------------------------------------------------
// 追上判定距离 (m)：飞机与小车的【水平】距离(不含高度差)小于此，开始累加追上时间
inline constexpr double CATCH_DIST        = 0.50;
// 追上确认时长 (s)：累计在 CATCH_DIST 内达此时长 → 认为"真的追上了"，切视觉锁定
inline constexpr double CATCH_HOLD_SEC    = 2.0;
 
// ───────────────────────────────────────────────────────────────────────────
//  ★★★ 以下三个"下降"参数当前【已停用】★★★(需求 2026-08：全程保持一个高度)
//    锁定/投掷阶段的高度 = 判定追上那一刻的实际高度，锁死不变，不再降到某个目标值。
//    所以【改这三个值不会有任何效果】。代码里对应的 TRACK_DESCEND 状态已删除。
//    要恢复"边追边降"：把这三个参数改回启用，并重新加回 TRACK_DESCEND 状态
//    (git 历史里有完整实现，含下降积分、到位判据、下降段超时兜底)。
//    ★飞行高度改哪里★：改起飞高度 takeoff(x)(fly_mission_node 的 TAKEOFF case)，
//    追踪/锁定全程都用起飞后悬停时锁定的那个高度。
// ───────────────────────────────────────────────────────────────────────────
// [已停用] 原"锁定阶段目标高度"(相对起飞点，m)
inline constexpr double LOCK_TARGET_Z_REL = 0.80; 
// [已停用] 原"下降速度"(m/s)
inline constexpr double LOCK_DESCEND_SPD  = 0.50;
// [已停用] 原"下降段总超时"(s)
inline constexpr double DESCEND_TIMEOUT_SEC = 15.0;

// 投掷判定距离 (m)：视觉偏差 ≤ 此值 → 发 "DIANCI" 给 Arduino 投掷
//   ★应 < CATCH_DIST★(锁定比追踪更精确)，否则一进锁定就立刻投
inline constexpr double DROP_DIST         = 0.10;

// ★★★ 投掷落点前置补偿 (m) ★★★
//   ★为什么需要★：投掷物离机时带着飞机的前向速度，落地前会继续往前飞
//   (高 1.5m 时下落约 0.55s，若飞机有 0.5m/s 前向速度，落点就偏前 ~0.28m)；
//   投掷机构本身若有前抛角、或相机光轴与机体轴不平行，也会造成固定的前偏。
//   这些都是【系统性偏差】——每次都偏同一个方向同一个量，所以可以直接补偿。
//
//   ★怎么用★：把判定用的 dx 减去本值，即"目标还在前方这么多米时就提前投"。
//       判定距离 = hypot(dx - DROP_LEAD_X, dy)
//   实测落点偏目标【前方】15cm → 本值填 0.15。
//
//   ★符号约定★：dx 向前为正(视觉端 raw_dx=(cy-py)*h/fy，注释明确"前为正")。
//     · 落点偏【前】(过头了) → 本值填【正数】(提前投)  ← 当前情况
//     · 落点偏【后】(没到)   → 本值填【负数】(晚点投)
//     · 落点准             → 填 0
//   ★只影响投掷时机判定，不影响飞行目标点★——飞机仍然是往 dx/dy=0 (正上方)飞，
//   只是"什么时候按下投掷按钮"提前了一点。所以不会让飞机停在偏前的位置。
//
//   ★调参方法★：让小车静止、飞机悬停对准后投一次，量落点偏差：
//     · 静止时也偏前 → 是机构前抛角/相机光轴不正，本值就是主要修正手段
//     · 静止时准、移动时才偏 → 是速度效应，本值按常用追踪速度标定
//       (速度变化大时补偿不可能对所有速度都准，取常用工况即可)
inline constexpr double DROP_LEAD_X       = 0.30;

// ★★★ 投掷落点【横向】补偿 (m) ★★★ —— 与上面 DROP_LEAD_X 完全同一套机制，
//   只是作用在 dy(左右)而不是 dx(前后)。判定距离变成：
//       hypot(dx - DROP_LEAD_X, dy - DROP_LEAD_Y)
//
//   ★符号约定★：dy ★向左为正★(视觉端 shm_dy_sign=1.0，注释明确"左为正")。
//   ★符号与"落点偏哪边"【相同】★(与直觉相反，下面有推导)：
//     · 落点偏【左】 → 本值填【正数】   ← ★当前情况：实测偏左 20cm → 填 +0.20★
//     · 落点偏【右】 → 本值填【负数】
//     · 落点准       → 填 0
//
//   ★推导(别凭直觉，容易反)★：设投掷瞬间的视觉横偏是 dy，落点相对目标的横向偏差
//   记作 E(左正)。已知"正对目标(dy=0)投时 E=+0.20(偏左)"，而投掷物会跟着飞机
//   相对目标的位置走，所以 E = -dy + 0.20。
//   要 E=0 ⇒ dy = +0.20 ⇒ ★必须在"目标位于飞机左侧 20cm"时触发投掷★。
//   判定式 hypot(dx-LEAD_X, dy-LEAD_Y) 的中心在 dy=LEAD_Y，
//   所以 LEAD_Y = +0.20。
//   直观理解：飞机继续往目标飞的过程中，投掷物本来会落在目标左边；那就干脆
//   ★早一点、在飞机还没越过目标(目标仍在左前方)时就投★，让那个左偏刚好把
//   投掷物送到目标上。
//
//   ★横向偏差的来源★(与前向不同)：相机横向安装偏移、光轴左右不正、
//   投掷机构侧向偏置、或飞机带侧滑速度。同样是系统性偏差，可以固定补偿。
//
//   ★只影响投掷时机判定，不影响飞行目标点★(同 DROP_LEAD_X)——飞机仍然往
//   dx/dy=0 (目标正上方)飞，不会停在偏心的位置。
//
//   ★实测校验法★(比推符号可靠)：改完飞一次，看落点是否居中。
//     · 若偏得更厉害了(20cm → 40cm) → 符号反了，取相反数
//     · 若变成偏右 → 补过头，减小绝对值
inline constexpr double DROP_LEAD_Y       = 0.0;   // 实测落点偏左 20cm → 填 +0.20

// 投掷后额外锁定时长 (s)：发完 DIANCI 再原地锁定这么久(等投掷物真的离机)，然后返航
inline constexpr double DROP_HOLD_SEC     = 2.0;
// ★★★ 锁定后"必投"倒计时 (s) ★★★
//   ★语义(需求 2026-08)★：从【切入 LOCK_DROP 那一刻】起算，到点【无条件投掷】——
//     · 视觉能不能看见   —— 不管
//     · 视觉进程死没死   —— 不管
//     · 雷达有没有接管   —— 不管
//     · 有没有对准到 DROP_DIST —— 不管
//   即这是一个纯挂钟倒计时，不依赖任何数据源。到点就发 DIANCI，然后走与正常投掷
//   完全相同的收尾：追加锁定 DROP_HOLD_SEC → 返航 (0,0) → 降落。
//   当然若在倒计时内就对准了 DROP_DIST，会【提前】正常投掷(哪个先到算哪个)。
//
//   ★为什么要它★：LOCK_DROP 若只以"对准成功"为出口，视觉标定偏/小车太快/镜头脏/
//   视觉进程挂了 都会让飞机永久悬停在小车上方直到电池耗尽。宁可投得不够准，
//   也必须把投掷这个动作做完并安全返航。
//   ★代价★：到点强投时飞机不一定正对目标，落点精度取决于当时的偏差。
//   ★调参提示★：给视觉留出对准时间即可。太短 → 还没对准就强投(落点偏)；
//   太长 → 视觉真死时白等。4s 是"够视觉试几十帧、又不至于久等"的折中。
//   设 <=0 = 关闭必投(不建议：会退回"可能永久悬停"的老问题)。
inline constexpr double LOCK_TIMEOUT_SEC  = 8.0;

// 锁定阶段视觉数据超时 (s)：shm 里 dx/dy 超过这么久没更新 → 视觉数据判为不可用
//   (不拿旧 dx/dy 硬飞)。超时后不投掷，改按下面的回退策略继续跟。
inline constexpr double LOCK_CV_TIMEOUT_S = 0.5;
// ★视觉丢失回退到雷达定位的时长 (s)★：视觉连续这么久没数据 → 改用小车雷达(UDP)
//   的位置继续锁定(精度不如视觉，但比原地干等强——小车可能已经开走了)。
//   ★视觉数据分三级(高度全程不变，三级都一样)★：
//     0 ~ LOCK_CV_TIMEOUT_S        视觉正常，用 dx/dy 精确锁定 + 可投掷
//     LOCK_CV_TIMEOUT_S ~ 本值     短暂丢帧：原地锁住等它回来(不投)
//     > 本值                       长时间丢：★回退雷达定位★继续跟着小车飞
//                                   (仍不投——雷达精度不够，投了大概率偏)
//   视觉一旦恢复立即切回精确锁定。要关掉回退(丢了就一直等)把本值设成很大即可。
//
//   ★★★ 与必投倒计时 LOCK_TIMEOUT_SEC 的时序关系(重要) ★★★
//   当前配置：CV_TIMEOUT=0.5s / CV_FALLBACK=3.0s / ★必投=4.0s★
//   若视觉从进锁定就一直死：
//       0.5s  判视觉不可用 → 原地锁住等
//       3.0s  回退雷达定位，跟着小车飞
//       4.0s  ★必投★ → 投掷 → 收尾返航
//   ⇒ ★雷达回退实际只工作 1 秒★就被必投打断。这是"必投优先"的预期结果，不是 bug。
//     若你希望视觉死时让雷达多跟一会儿再投，把 LOCK_TIMEOUT_SEC 调大(或把
//     LOCK_CV_FALLBACK_S 调小让雷达更早接管)。
inline constexpr double LOCK_CV_FALLBACK_S = 3.0;
// 投掷用的 Arduino 指令字符串(经串口发给 Arduino，会自动补 \n)
inline constexpr const char* DROP_CMD     = "DIANCI";


// ═══════════════════════════════════════════════════════════════════════════
//  ★★★ 纯雷达投掷模式(不用视觉) ★★★
//
//  true  = ★纯雷达投掷★：完全不看 shm 视觉的 dx/dy，只用小车雷达(UDP)定位。
//          流程(接在 TRACK_CAR 判定追上之后)：
//            RADAR_DESCEND  边追边降：以 RD_DESCEND_SPD 降到 RD_DROP_H_REL 高度；
//                           水平目标 = 小车位置 + 【车身后方 RD_OFS_X】(随车头旋转)；
//                           y 与 yaw 正常跟随小车(不加偏移)
//            RADAR_DROP     到高度后开始判稳：水平误差 ≤RD_STABLE_TOL 且高度到位，
//                           连续 RD_STABLE_SEC 秒 → ★直接投掷★(发 DIANCI)
//                           → 锁定 DROP_HOLD_SEC → 爬回 RD_RETURN_H_REL
//                           → 飞回起点 (0,0) → 降落
//  false = 原来的视觉锁定投掷(LOCK_DROP，用 shm 的 dx/dy 精确对准)。
//
//  ★两条链完全独立★：纯雷达链不碰 lock_* 那套视觉变量，视觉链也不受影响，
//  改这个开关不会互相污染。
// ═══════════════════════════════════════════════════════════════════════════
inline constexpr bool   RADAR_DROP_MODE   = true;

// 下降速率 (m/s)：目标高度按这个速率匀速往下走。
//   ★注意★：MOVE_POSE 默认被平飞档 MAX_SPEED_Z_LEVEL(0.1m/s) 限死，所以本状态
//   会调 set_plat_descend_mode(true) 把垂直限速放开到本值(与落平台同一机制)。
inline constexpr double RD_DESCEND_SPD    = 0.4;
// 投掷高度 (m，★相对起飞点★)：降到这个高度就停止下降、开始判稳准备投掷。
inline constexpr double RD_DROP_H_REL     = 1.0;
// ★水平目标相对小车雷达的偏移 (m，小车机体系，车头为 X 正 / 左为 Y 正)★
//   需求：投掷点在目标【后方 50cm】→ RD_OFS_X = -0.50。
//   ★随小车 yaw 旋转★(见 radar_drop_target()，偏移定义在小车机体系)，
//   所以小车转弯时投掷点始终保持在车身后方，不会跑到侧面。
//   y 方向不加偏移(需求：y 正常矫正) → RD_OFS_Y = 0。
inline constexpr double RD_OFS_X          = -0.25;
inline constexpr double RD_OFS_Y          =  0.0;
// 判稳的水平容差 (m)：飞机与【偏移后的投掷点】水平距离小于此值才算"稳住了"。
//   比 DROP_DIST(视觉用的 0.10) 放宽——雷达定位精度不如视觉，要求太严会一直凑不满。
inline constexpr double RD_STABLE_TOL     = 0.20;
// 判稳持续时长 (s)：水平进容差 + 高度到位，连续这么久 → 投掷。
inline constexpr double RD_STABLE_SEC     = 2.0;
// 高度到位容差 (m)：|实际高度 - RD_DROP_H_REL| 小于此值算高度到位。
inline constexpr double RD_H_TOL          = 0.15;
// 投掷后爬回的高度 (m，相对起飞点)：投完先爬到这个高度再返航(避免低空长距离飞行)。
inline constexpr double RD_RETURN_H_REL   = 1.5;
// ★下降段超时 (s)★：这么久还没降到投掷高度 → 不再等，直接进判稳阶段
//   (高度可能因载重/风降不到位，但不该无限期悬着)。
inline constexpr double RD_DESCEND_TIMEOUT_S = 4.0;
// ★判稳总超时 (s)★：进入判稳后这么久还没凑满 RD_STABLE_SEC → ★无条件投掷★。
//   与视觉链的 LOCK_TIMEOUT_SEC 同思路：小车乱跑/雷达标定偏时也要能把动作做完。
inline constexpr double RD_DROP_TIMEOUT_S = 6.0;


// ---------------------------------------------------------------------------
// ★★★ 遥测上报(UDP 单播，飞机 → 监控端) ★★★
//   持续上报：飞机坐标(飞机 SLAM 系) + 小车原始坐标(小车 B 系，★未加标定平移★)
//             + 飞行状态(1=起飞 2=追踪 3=投掷 4=降落 0=其它)
//   协议/收发端见 udp_telemetry.hpp(64 字节定长包，magic='TLM1')。
//
//   ★只发不收★：UDP 无连接，对方没开机/掉线/重启都不影响本机飞行——发送失败
//   只按 TLM_WARN_PERIOD_S 节流打一条★红色★告警(RCLCPP_ERROR)，绝不阻塞主循环。
//   ★正常时终端完全不打印★(需求)：只在连续发送失败时才提示。
//
//   端口分工(三条通道端口都不同，互不干扰)：
//       9870 收小车位姿 / 9871 收启动指令 / ★9872 发遥测★
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ★★★ 第二段任务：降落到移动平台 → 再起飞 → 回起点 ★★★
//
//  完整流程(接在第一段"投掷后返航降落"之后)：
//    LAND(第一段降落完成，已上锁)
//      └─ 转 WAIT_TRIGGER：地面待机，等 UDP 命令 ★2★(端口与命令 1 相同 = 9871)
//    REARM        自己切 OFFBOARD + 解锁
//    TAKEOFF_AGAIN 爬到 1.5m(★以最初起飞点为基准，不重记 home★)
//    TRACK_CAR2   ★直接进追踪，不悬停★；水平距离 ≤CATCH_DIST 累计
//                 达 ★CATCH_HOLD_SEC_2★(与第一段不同的确认时长) → 转下降
//    TRACK_LAND   ★边追踪边匀速下降★：水平跟小车，目标高度按 PLAT_DESCEND_SPD 持续下压；
//                 ★靠接触检测判落地★(实际高度降不动了 → 已被平台托住) → 主动上锁
//    PLAT_WAIT    平台上等 PLAT_WAIT_SEC 秒(不需要任何指令)
//    PLAT_TAKEOFF 自己切 OFFBOARD + 解锁 + 爬到 1.5m
//    GO_HOME      飞回 SLAM 原点 (0,0)
//    LAND         降落，任务结束
// ---------------------------------------------------------------------------
// 第二段任务总开关：false = 第一段降落完【直接结束】(进 FINISHED)。
//   ★true 时 LAND 不会结束任务★：会 drone_.stop() 后进 WAIT_TRIGGER 地面待机，
//   一直等第二段的启动命令(UDP 命令 2 / /mission/takeoff_again)。
//   ⇒ 只想"起飞→任务→降落→结束"的流程，这里必须是 false，否则飞机落地后
//     程序不退出、日志一直刷"地面待机中，等命令 2"。
inline constexpr bool   MISSION2_ENABLE   = false;
// 第二段任务的"追上确认时长"(s)：与第一段 CATCH_HOLD_SEC 分开，互不影响。
//   累计在 CATCH_DIST 内达此时长 → 开始边追边降。
inline constexpr double CATCH_HOLD_SEC_2  = 8.0;

// ═══════════════════════════════════════════════════════════════════════════
//  ★★★ 落平台用【接触检测】而不是高度阈值 ★★★
//
//  ★为什么不用高度阈值★：那需要"上锁高度 = home_z + 平台高度 + 间隙"。
//    · home_z 是 BOOT_CHECK 时(几分钟前)记的，而落平台发生在
//      起飞→追踪21s→投掷→返航降落→等命令→再起飞→追踪21s→下降 之后；
//      这几分钟里 SLAM 的 z 会漂移，home_z 是旧基准 → 上锁高度跟着偏。
//    · 平台高度还得人工量准。量错/漂移任何一个超过间隙(十几厘米)，
//      就会 "从高处摔" 或 "撞进平台"。两个误差源和安全间隙同一量级，不可接受。
//
//  ★接触检测怎么做★：目标高度持续匀速往下压，同时看【实际高度还降不降】：
//      · 还在降  → 说明悬在空中，继续压
//      · ★目标在降，实际却不降了★ → 起落架已经顶在平台上(被平台托住)
//        → 连续满足 PLAT_TOUCH_HOLD_S 秒 → 判定已接触 → 主动上锁
//  ★好处★：完全不依赖平台高度、不依赖 home_z、不受 SLAM 漂移影响 ——
//    它测的是"物理上还能不能继续下降"，这个判据本身就是绝对的。
//  ★代价★：上锁前飞机会用推力顶着平台约 PLAT_TOUCH_HOLD_S 秒(轻微压着)。
// ═══════════════════════════════════════════════════════════════════════════
// ★★★ 上锁高度闸门 (m，★相对起飞点★) ★★★
//   ★硬性安全线★：飞机相对起飞点的高度【高于此值时，无论速度判定怎么说都绝不上锁】。
//   只有降到这条线以下，才启用下面的 Z 速度判定(PLAT_TOUCH_VZ)去判断有没有接触平台。
//
//   ★为什么必须有★：光靠"Z 速度降不动"判接触是不够的——飞机刚进入下降段时还在
//   悬停(垂直速度≈0)、或者下降途中被风顶一下速度短暂归零，都会被误判成"已接触"
//   → ★在高空锁桨 = 摔机★。加这道高度闸门后，即使速度判定误判，只要飞机还在
//   高处就绝不会上锁，把最坏后果从"摔机"降到"降不下去(走超时返航)"。
//
//   ★这个值是死的，但会自动跟随雷达启动偏移★：实际比较的是
//       (当前 SLAM z − home_z) ≤ 本值
//   home_z 是 BOOT_CHECK 时飞机停在地面记下的 SLAM z，所以雷达每次启动时
//   原点在哪都无所谓 —— 相减后就是"离起飞地面多高"，物理含义固定。
//
//   ★★★ 怎么取值：必须【明显大于】平台台面高度，不能只是"大于" ★★★
//     飞机停在台面上时，h_rel ≈ 台面高度 + 起落架高度。若闸门 ≈ 台面高度，
//     读数就正好卡在闸门边界上：噪声让它偶尔超过闸门 → below_gate 变 false →
//     ★接触计时被清零★，而 PLAT_TOUCH_HOLD_S(0.5s) 要求【连续 25 拍】都在闸门内，
//     在边界上几乎不可能满足 → ★落到平台了却永远不锁桨★
//     (2026-08 实测踩过：台面 20cm、闸门也填 0.20，就是这个症状)。
//   ⇒ ★经验取值：台面高度 + 0.25~0.35m★
//     例：台面离地 20cm → 本值取 0.50。飞机停上去 h_rel≈0.22，距闸门 0.28m 余量，
//        噪声再大也不会跨过去；同时"从 50cm 降到 22cm"这 28cm 就是速度判定的作用区间。
//   ★取大了★：闸门形同不存在，回到"可能高空误判"的风险
//     (但仍有 plat_descending_ + PLAT_TOUCH_HOLD_S 两道兜底，不会立刻摔)。
//   ★取小了★(≤台面高度)：飞机永远进不了判定区 → 一直压着平台直到超时返航。
//     ⇒ ★宁可偏大，绝不可接近或小于台面高度★。
inline constexpr double PLAT_DISARM_MAX_H_REL = 0.50;   // 台面 20cm + 30cm 余量

// 判定"实际高度不再下降"的速率阈值 (m/s)：实际下降速率低于此值就算"降不动了"。
//   ★只在高度已低于 PLAT_DISARM_MAX_H_REL 时才参与判定★(见上)。
//   降速取自控制器的 vz_est()(位置差分 + V_EST_ALPHA 低通)，不是单拍裸差分 ——
//   单拍差分下 ±1cm 的 SLAM 噪声就等于 0.01/0.02 = 0.5m/s 假速度，是本阈值的 10 倍，
//   会把接触计时反复清零 → ★永远判不出接触、永不锁桨★(2026-08 实测踩过)。
//   ★要大于高度估计的噪声抖动、小于正常下降速率★：
//     太小 → 噪声就能超过它 → 接触计时被反复清零 → 判不出接触
//     太大 → 正常下降途中被误判成已接触(有高度闸门兜底，不会真在高空锁桨)
//   0.05 = 每秒降不到 5cm 就认为被托住了。
inline constexpr double PLAT_TOUCH_VZ      = 0.05;

// ★"已确认真的降起来了"的最小降速 (m/s)★：实际降速超过此值才启用接触检测。
//   ★为什么要它★：刚进边追边降时飞机还在悬停(降速≈0)，若不设这道门，会立刻满足
//   "降不动"→ 在高空锁桨。有了它，必须先看到飞机确实在往下走。
//
//   ★★★ 取值铁律：必须【明显小于】PLAT_DESCEND_SPD ★★★
//     进入本状态会调 set_plat_descend_mode(true) 把垂直限速放开到 PLAT_DESCEND_SPD，
//     那就是实际降速的【硬上限】。本值若接近或超过它 → 这道门永远过不去 →
//     ★接触检测永不启用 → 永不锁桨 → 走到超时返航★。
//     ⇒ 取 PLAT_DESCEND_SPD 的 1/3 ~ 1/2，同时留出 >PLAT_TOUCH_VZ 的余量
//       (两者相等会在"启用检测"和"判定接触"之间来回跳)。
//     当前：PLAT_DESCEND_SPD=0.20 → 本值 0.08(=40%)，且 0.08 > PLAT_TOUCH_VZ(0.05)。
//   ★这个死锁踩过两次★：一次是用 PLAT_DESCEND_SPD*0.5 推算却忘了放开垂直限速
//     (平飞档 MAX_SPEED_Z_LEVEL=0.1 = 门槛，严格大于永不成立)；一次是改成独立参数
//     但填了 0.25 > PLAT_DESCEND_SPD(0.2)。★改这两个值必须一起看★。
inline constexpr double PLAT_DESCEND_MIN_VZ = 0.08;
// 接触判定持续时长 (s)：连续这么久都"降不动"才判定真接触(防瞬时噪声误判)。
//   ★别太短★：0.1s 级别容易被一次高度跳变骗到 → 提前上锁(有高度闸门兜底，
//     不会真在高空锁桨，但仍不该发生)。
//   ★别太长★：这段时间飞机在用推力压着平台，平台在动时可能被拖行。0.5s 是折中。
inline constexpr double PLAT_TOUCH_HOLD_S  = 0.5;
// 目标高度相对"接触时高度"最多再往下压多少 (m)：给 PD 制造持续向下的动力。
//   目标压到 (进入下降时高度 - 一直减) 但不会低于 (接触判定时的实际高度 - 此值)，
//   避免无限往下压导致推力饱和、把平台压坏或把飞机顶翻。
inline constexpr double PLAT_PUSH_LIMIT    = 0.40;
// 边追边降的下降速率 (m/s)：目标高度按这个速率匀速往下走。
//   ★同时是垂直限速★：进本状态会 set_plat_descend_mode(true) 把 MOVE_POSE 的垂直
//   限速从平飞档 MAX_SPEED_Z_LEVEL 放开到本值，所以本值 = 实际降速的硬上限。
//   ★别太快★：平台在动，降太快来不及水平跟上就会落偏。
inline constexpr double PLAT_DESCEND_SPD   = 0.2;

// ★★★ 编译期护栏：把上面那条"取值铁律"钉死 ★★★
//   这两个 static_assert 拦的是【实测踩过两次】的死锁：接触检测的启用门槛一旦够到
//   实际降速的硬上限，门就永远过不去 → 落到平台了却永不锁桨 → 超时返航。
//   谁再把这几个值改回互相咬死的组合，★编译就会失败★，而不是等到实机上才发现。
static_assert(PLAT_DESCEND_MIN_VZ < PLAT_DESCEND_SPD * 0.75,
              "PLAT_DESCEND_MIN_VZ 必须明显小于 PLAT_DESCEND_SPD(实际降速的硬上限)，"
              "否则接触检测永不启用 → 永不锁桨。建议取 PLAT_DESCEND_SPD 的 1/3~1/2。");
static_assert(PLAT_DESCEND_MIN_VZ > PLAT_TOUCH_VZ,
              "PLAT_DESCEND_MIN_VZ 必须大于 PLAT_TOUCH_VZ，否则降速在阈值附近抖动时"
              "会在'启用检测'与'判定接触'之间来回跳。");
// 下降段总超时 (s)：这么久还没降到上锁高度 → 放弃降落，直接飞回起点降落。
//   ★为什么要★：小车一直乱跑/水平总追不上时，不能无限期悬着降不下去。
inline constexpr double PLAT_DESCEND_TIMEOUT_S = 40.0;
// 落到平台后的等待时长 (s)：等满就自己重新起飞(不需要任何指令)。
inline constexpr double PLAT_WAIT_SEC      = 5.0;


// ---------------------------------------------------------------------------
inline constexpr bool   TLM_ENABLE        = true;             // 总开关
inline constexpr const char* TLM_DEST_IP  = "192.168.12.187"; // ★监控端 IP★
inline constexpr int    TLM_DEST_PORT     = 9872;             // 与监控端一致
// 上报频率 (Hz)：状态机是 50Hz，这里限频以省带宽。10Hz 对监控/画轨迹足够。
inline constexpr double TLM_RATE_HZ       = 10.0;
// 发送失败告警节流 (s)：失败时每这么久打一条红色 ERROR(不刷屏)。
inline constexpr double TLM_WARN_PERIOD_S = 5.0;


// ---------------------------------------------------------------------------
// ★ Arduino 串口指令（arduino_send）★
//   状态机里 arduino_send("LED ON") 经串口(115200,8N1)发 ASCII "LED ON\n"，
//   默认重发 ARDUINO_SEND_TIMES 次(防单次丢包，Arduino 端重复收同指令应幂等)。
//   没插 Arduino/设备名不对 → 只打告警，任务照飞。实现在 arduino_serial.hpp。
// ---------------------------------------------------------------------------
inline constexpr const char* ARDUINO_DEV     = "/dev/tty_module_A";  // 真机 udev 固定别名(→ttyCH341USB0)。仿真原为 /dev/ttyUSB0。ls -l /dev/tty_module_* 确认
inline constexpr int    ARDUINO_SEND_TIMES   = 5;     // 默认每条指令重发次数
inline constexpr int    ARDUINO_SEND_GAP_MS  = 20;    // 重发间隔(ms)：防 Arduino 64B 串口缓冲一次涌爆
inline constexpr double ARDUINO_BOOT_WAIT_S  = 2.0;   // 首次打开串口后等待(s)：打开动作会复位 Arduino，等它重启进 loop 再发


}  // namespace params
}  // namespace fly_mission
