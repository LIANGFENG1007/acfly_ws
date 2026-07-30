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
inline constexpr double KP_YAW               = 0.6;    // yaw 误差 → yaw_rate

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
