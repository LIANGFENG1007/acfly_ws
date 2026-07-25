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
inline constexpr double MAX_SPEED_Z_LEVEL    = 0.05;   // 平飞时垂直微调最大速度 (m/s)，限制上下抖动
inline constexpr double MAX_YAW_RATE         = 0.8;    // 最大转头角速度 (rad/s) 

// ---------------------------------------------------------------------------
// PD 位置控制增益
// 控制律：v = Kp * (target - current) - Kd * current_velocity
// ---------------------------------------------------------------------------
inline constexpr double KP_XY                = 0.8;    // 水平 P
inline constexpr double KD_XY                = 0.25;   // 水平 D
inline constexpr double KP_Z                 = 0.6;    // 垂直 P（调小，减少激进矫正）
inline constexpr double KD_Z                 = 0.25;   // 垂直 D（加大阻尼，抑制上下抖动）
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
inline constexpr double TOL_XY               = 0.10;                  // 水平容差 (m)
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
//   注：到达图形上方的水平容差【不在此设】——复用上面的 TOL_XY（含稳定计时）。
// ---------------------------------------------------------------------------
inline constexpr int    FF_CONFIRM_FRAMES = 6;    // 连续确认帧数：越大越稳(误检难触发)但反应越慢
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
