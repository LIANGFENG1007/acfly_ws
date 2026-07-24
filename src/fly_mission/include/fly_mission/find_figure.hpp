#pragma once

// ============================================================================
//  find_figure.hpp  ── 机载视觉找图：后台监听 → 30 帧稳定确认 → 打断走线飞过去
//
//  角色：主控 fly_mission 的一个【外挂模块】，与写死的巡航走线(FWD_*)解耦。
//        起飞稳定后由主控在后台启用；本类订阅机载视觉话题、把"稳定看到的图形"
//        排队交给主控去飞，并维护"已访问(拉黑)区域"避免重复飞同一个图形。
//        本类【不订阅位姿、不发 setpoint、不碰探索算法】——飞行动作全部由主控用
//        现成的 target_xy_slam(PD) 执行，本类只负责"看到没/该飞哪/飞完没"。
//
//  输入：/cv/target_info (std_msgs/String, JSON)。视觉端已把图形位置解算成
//        【SLAM 全局坐标】，格式：
//          {"targets":[{"id":1,"color":"red","shape":"triangle","x":..,"y":..}, ...]}
//        一帧可含多个目标；坐标已是全局系(主控无需再转)；color/shape 为英文。
//
//  确认逻辑(防误触发)：只有【同颜色+同形状 且 世界坐标与当前累计中心接近】的检测
//        才算"同一个物理目标"跨帧累计；坐标跳开或断帧则计数清零重来——防止 A 处
//        图形攒 15 帧、B 处图形又攒 15 帧凑成 30 帧、算出一个两者之间不存在的假中心。
//        连续累计满 confirm_frames 帧 → 用这些帧世界坐标的【算术平均】作图形中心。
//
//  拉黑：确认出的中心若落在任一"已访问圆"(半径 black_radius)内 → 直接丢弃、连队都
//        不入(不再飞过去)。飞完一个目标(或超时放弃)后把其中心加入已访问列表。
//        ★仅本模块内存态，不持久化、不喂给探索算法★。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace fly_mission {

class FindFigure
{
public:
    // 一拍执行结果：主控据此决定继续飞还是回到被打断的状态。
    enum class Step {
        FLYING,   // 仍在飞向当前图形(主控应继续 target_xy_slam(out_x,out_y))
        DONE      // 本次找图结束(到点悬停完 / 超时放弃 / 队空)——主控应回被打断状态
    };

    explicit FindFigure(rclcpp::Node* node);

    // 喂入一帧【已解析好的】视觉 JSON(主控订阅 /cv/target_info、只 parse 一次后分发到这里)。
    //   本类不再自己订阅/解析(避免与 LineFollower 重复 parse 拖慢节点)；只从 j 取 targets。
    //   线程：由主控的视觉回调(Reentrant 后台线程)调用；内部自带锁。
    void ingest(const nlohmann::json& j);

    // ★启用找图★：主控在【起飞稳定完全结束(WAIT_AFTER_TAKEOFF 到位)】那一刻调一次。
    //   启用前：喂入的帧一律丢弃(不累计/不入队)，has_pending() 恒 false。
    //   目的：起飞爬升途中相机高度未稳，视觉解算的世界坐标不准；且不该把起飞点附近的
    //         图形攒进队列——否则起飞一稳就被打断、把起飞点错误拉黑。故从源头堵住。
    void enable();

    // 队列里是否有待处理(且未被拉黑)的图形 → 主控据此打断当前走线进 FINDFIGURE。
    bool has_pending() const;

    // 进入 FINDFIGURE 时调一次：起找图总计时、清悬停态(为队首目标开一段新的处理过程)。
    void begin();

    // FINDFIGURE 每拍调用。reached_current_target = 主控 target_xy_slam 的到位判定
    //   (drone_.is_reached())——本类不自己判位置，复用主控现成的 MOVE_XY 到位(含稳定计时)。
    //   出参 out_x/out_y = 本拍应飞向的图形中心(SLAM 系，实时刷新)。
    //   返回 FLYING → 主控 target_xy_slam(out_x,out_y)；DONE → 主控回被打断状态。
    Step tick(bool reached_current_target, double& out_x, double& out_y);

private:
    // 视觉发来的一个图形(本帧一条)。坐标已是 SLAM 全局系。
    struct Target {
        std::string color;
        std::string shape;
        double      x = 0.0;
        double      y = 0.0;
    };
    // 2D 点(拉黑圆心 / 队列目标中心)。不引 exploration::Vec2(跨包)，本模块自带。
    struct P { double x = 0.0; double y = 0.0; };

    // 从【已解析的 JSON】取出本帧目标列表(容错：字段缺失/类型不对跳过，绝不抛)。
    std::vector<Target> parse_targets(const nlohmann::json& j) const;

    // 点 p 是否落在任一已访问(拉黑)圆内。需在持有 mtx_ 时调用。
    bool in_blacklist(const P& p) const;

    // ---- ROS ----
    rclcpp::Node* node_;

    // ---- 可调参数：★默认值集中在 fly_mission/params.hpp 的 ★视觉找图★ 段★，
    //      构造里 declare_parameter 从那里读入，运行期 -p ff_* 可覆盖。
    //      下面初值仅构造前占位(会被 params 覆盖)——改默认值请去 params.hpp。
    int    confirm_frames_ = 30;    // 连续确认帧数
    double assoc_dist_     = 0.30;  // 跨帧同一目标的坐标关联阈值(m)
    double black_radius_   = 0.50;  // 拉黑半径(m)
    // 注：到达图形上方的水平容差不在此设——复用主控 target_xy_slam 的到位判定
    //     (params::TOL_XY，含稳定计时)，由主控把 drone_.is_reached() 传进 tick()。
    double hover_sec_      = 0.50;  // 到点悬停时长(s)
    double timeout_sec_    = 5.00;  // 单次找图总超时(s)：超过直接放弃(但仍拉黑)，回被打断状态
    double frame_gap_sec_  = 0.50;  // 断帧阈值(s)：两帧间隔超此认为目标暂时消失→累计清零(防残留计数)

    // ---- 累计候选(后台写/主线程读，mtx_ 保护) ----
    mutable std::mutex mtx_;
    bool         enabled_ = false;      // ★未启用前丢弃所有帧★(起飞稳定后主控 enable() 才开始)
    bool         acc_active_ = false;   // 是否正在累计某个目标
    std::string  acc_color_;
    std::string  acc_shape_;
    double       acc_cx_ = 0.0, acc_cy_ = 0.0;  // 已收帧世界坐标的算术平均(=当前估计中心)
    int          acc_n_  = 0;                    // 已累计帧数
    rclcpp::Time acc_last_time_;                 // 上次命中累计的时刻(断帧判定)
    bool         acc_time_valid_ = false;

    // ---- FIFO 待处理队列 + 拉黑列表(mtx_ 保护) ----
    std::deque<Target>  queue_;      // 已确认待飞的图形(当前优先，放行后弹下一个)
    std::vector<P>      visited_;    // 已访问(拉黑)图形中心，圆半径 = black_radius_

    // ---- 找图执行态(主线程用) ----
    bool         hover_active_ = false;   // 到点后进入悬停计时
    rclcpp::Time hover_until_;            // 悬停结束时刻
    rclcpp::Time find_start_time_;        // 本次找图开始时刻(总超时用)
    bool         find_time_valid_ = false;

    // 累计重置(需在持有 mtx_ 时调用)
    void reset_accum();
};

}  // namespace fly_mission
