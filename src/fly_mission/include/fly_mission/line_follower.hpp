#pragma once

// ============================================================================
//  line_follower.hpp  ── 视觉寻线：沿黑线飞，机头始终朝前进方向
//
//  角色：主控 fly_mission 的 FOLLOW_LINE 状态用的外挂模块。起飞后由主控直接进入寻线，
//        本类后台监听机载视觉给的"下一个前视点"，算出机体系速度命令交主控执行；
//        并判定"线走完/丢线"(连续一段时间没有有效线)让主控降落。
//        本类【不订阅位姿、不发指令】——飞行动作由主控用 set_velocity_body 执行。
//
//  输入：/cv/target_info (std_msgs/String, JSON)，取其中 line_x / line_y：
//        视觉端(blackline*.py)算出的"线上距画面中心(≈机体正下方)10cm 的前视点"的
//        【机体系偏移】——line_x=前后(前为正)、line_y=左右(左为正)，未做 yaw 旋转。
//        丢线时视觉发 line_x=line_y=0(仍在发)，故靠"向量模长≈0"判无效。
//        (同话题也可能来找图格式 {"targets":..}；本类只读 line_x/line_y，读不到即无效，不崩。)
//
//  控制：前进方向偏角 = atan2(line_y, line_x)。yaw_rate 把机头转向它(偏角纠到 0 →
//        机头始终朝前进方向)；v_fwd 前进(机头偏太多乘 cos 门控压低，防画龙)；v_lat=0。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <mutex>

namespace fly_mission {

class LineFollower
{
public:
    explicit LineFollower(rclcpp::Node* node);

    // 喂入一帧【已解析好的】视觉 JSON(主控订阅 /cv/target_info、只 parse 一次后分发到这里)。
    //   本类不再自己订阅/解析(避免与 FindFigure 重复 parse 拖慢节点)；只从 j 取 line_x/line_y。
    //   线程：由主控的视觉回调(Reentrant 后台线程)调用；内部自带锁。
    void ingest(const nlohmann::json& j);

    // 进入 FOLLOW_LINE 时调一次：清超时计时(避免用上次残留)，从此刻起算无线时长。
    void begin();

    // 本状态每拍调用：算出机体系速度命令(前进/横向/yaw_rate)写入出参。
    //   返回 true = 有有效线、命令有效(主控 set_velocity_body 执行)；
    //   返回 false = 当前无有效线(命令给 0，主控悬停等)；是否降落看 timed_out()。
    bool compute(double& v_fwd, double& v_lat, double& yaw_rate);

    // 是否已连续 LF_TIMEOUT_SEC 没有有效线(线走完/丢线) → 主控据此降落。
    bool timed_out() const;

private:
    rclcpp::Node* node_;


    // ---- 可调参数：默认值集中在 params.hpp ★视觉寻线★ 段，构造里 declare_parameter 读入 ----
    // ★前进(x)与横向(y)同款 PD，共用 kp_lat_/kd_lat_/max_v_lat_★(x/y 一套增益，见 compute)。
    double kp_lat_         = 1.00;   // 前后+横向偏差 → 速度的 P(★x/y 共用★)：越大回线越快(过大超调/晃)
    double kd_lat_         = 0.30;   // 偏差变化率 → 速度的 D(★x/y 共用★)：阻尼，压超调/振荡
    double y_alpha_        = 0.40;   // line_x/line_y 低通系数(0~1)：新值权重，越小越平滑
    double max_v_lat_      = 0.40;   // 前后+横向 速度限幅 (m/s)(★x/y 共用★)
    double timeout_sec_    = 1.00;   // 连续无有效线超此 → 降落 (s)
    double min_valid_m_    = 0.02;   // line 模长 < 此视为无有效线(判 0)

    // ---- 最新一帧线数据(后台写/主线程读，mtx_ 保护) ----
    mutable std::mutex mtx_;
    double       line_x_ = 0.0, line_y_ = 0.0;   // 最新前视点(机体系)：前 x / 左 y
    bool         have_valid_ = false;            // 最新一帧是否有有效线
    rclcpp::Time last_valid_time_;               // 上次收到有效线的时刻(超时判定)
    bool         last_valid_valid_ = false;      // last_valid_time_ 是否已被赋过值

    // ---- 低通 + 横向 D 项状态(主线程 compute 里用) ----
    double       x_filt_ = 0.0;          // 低通后的 line_x(前进用)
    double       y_filt_ = 0.0;          // 低通后的 line_y(横向用)
    bool         filt_init_ = false;     // 低通是否已初始化(首帧直接用，不做差分)
    rclcpp::Time prev_frame_time_;       // 上一【视觉帧】时刻(算 D 项 dt，用帧率而非主循环率)

    // ---- 消除 D 脉冲：控制在【视觉真更新那一帧】才重算，主循环其余拍复用缓存命令 ----
    uint64_t     frame_seq_ = 0;         // 视觉有效帧序号(回调 +1)
    uint64_t     computed_seq_ = 0;      // compute 已据以重算过的帧序号
    double       cmd_vfwd_ = 0.0;        // 缓存的前进命令
    double       cmd_vlat_ = 0.0;        // 缓存的横向纠偏命令
};

}  // namespace fly_mission
