// ============================================================================
//  line_follower.cpp  ── 视觉寻线 实现（详见同名 .hpp）
// ============================================================================

#include "fly_mission/line_follower.hpp"
#include "fly_mission/params.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>

namespace fly_mission {

LineFollower::LineFollower(rclcpp::Node* node)
    : node_(node)
{
    // 参数：默认值集中在 params.hpp ★视觉寻线★ 段，运行期 -p lf_* 可覆盖。
    auto dd = [this](const std::string& n, double v) { return node_->declare_parameter<double>(n, v); };
    kp_lat_       = dd("lf_kp_lat",       params::LF_KP_LAT);
    kd_lat_       = dd("lf_kd_lat",       params::LF_KD_LAT);
    y_alpha_      = dd("lf_y_alpha",      params::LF_Y_ALPHA);
    max_v_lat_    = dd("lf_max_v_lat",    params::LF_MAX_V_LAT);
    timeout_sec_  = dd("lf_timeout_sec",  params::LF_TIMEOUT_SEC);
    min_valid_m_  = dd("lf_min_valid_m",  params::LF_MIN_VALID_M);

    // 不再自己订阅 /cv/target_info：改由主控订阅一次、只 parse 一次后 ingest() 喂进来
    //   (原来 FindFigure/LineFollower 各订各 parse 同一条消息，重复解析拖慢节点→起飞/走线卡顿)。
    RCLCPP_INFO(node_->get_logger(),
        "[寻线] LineFollower 已启动 (x/y 同款PD KP=%.2f KD=%.2f 限速%.2fm/s, 无线超时 %.1fs)",
        kp_lat_, kd_lat_, max_v_lat_, timeout_sec_);
}

void LineFollower::begin()
{
    std::lock_guard<std::mutex> lk(mtx_);
    // 进入寻线：把"上次有效线时刻"设为现在，从此刻起算无线时长(不用历史残留)。
    last_valid_time_  = node_->now();
    last_valid_valid_ = true;
    have_valid_       = false;
    // 复位低通/D 项状态：下次 compute 从干净开始，不用上次残留。
    filt_init_    = false;
    x_filt_       = 0.0;
    y_filt_       = 0.0;
    computed_seq_ = frame_seq_;   // 对齐帧号：进入后等下一帧新数据才算(不吃旧帧)
    cmd_vfwd_ = 0.0; cmd_vlat_ = 0.0;
}

void LineFollower::ingest(const nlohmann::json& j)
{
    double lx = 0.0, ly = 0.0;
    bool valid = false;
    try {
        // 只取 line_x/line_y；没有这俩字段(如找图格式)→ 当无有效线，不崩。
        if (j.contains("line_x") && j.contains("line_y") &&
            j["line_x"].is_number() && j["line_y"].is_number()) {
            lx = j["line_x"].get<double>();
            ly = j["line_y"].get<double>();
            if (std::isfinite(lx) && std::isfinite(ly) &&
                std::hypot(lx, ly) >= min_valid_m_) {     // 模长≈0 = 视觉丢线发的 0 → 无效
                valid = true;
            }
        }
    } catch (const std::exception& ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[寻线] 取 line_x/line_y 出错: %s", ex.what());
    }

    std::lock_guard<std::mutex> lk(mtx_);
    have_valid_ = valid;
    if (valid) {
        line_x_ = lx;
        line_y_ = ly;
        last_valid_time_  = node_->now();
        last_valid_valid_ = true;
        ++frame_seq_;          // 标记"来了一帧新的有效线"→ compute 据此只在新帧重算(消 D 脉冲)
    }
}

bool LineFollower::compute(double& v_fwd, double& v_lat, double& yaw_rate)
{
    std::lock_guard<std::mutex> lk(mtx_);
    v_fwd = 0.0; v_lat = 0.0; yaw_rate = 0.0;

    if (!have_valid_) {            // 无有效线→命令0；断了重置滤波，下次有线重新起
        filt_init_ = false;
        cmd_vfwd_ = 0.0; cmd_vlat_ = 0.0;
        return false;
    }

    // ★只在【视觉真来了新的一帧】才重算 P/PD/滤波★——主循环 20Hz 比视觉快，若每拍都算，
    //   帧未更新时 D 项时零时爆，会给横向打脉冲加剧振荡。新帧才算 → D 项用真实帧率 dt，干净。
    if (frame_seq_ != computed_seq_) {
        const double x_raw = line_x_;             // 前视点前后距离(m)，直线≈0.5 急弯≈0.05
        const double y_raw = line_y_;             // 横向偏差(左为正)，含视觉抖动
        const rclcpp::Time now = node_->now();

        double d_x = 0.0, d_y = 0.0;
        if (!filt_init_) {
            x_filt_ = x_raw;                      // 首帧直接用，不做差分(避免 D 跳变)
            y_filt_ = y_raw;
            filt_init_ = true;
        } else {
            const double prev_x = x_filt_;
            const double prev_y = y_filt_;
            // ① 低通(EMA)：滤视觉逐帧抖动
            x_filt_ = y_alpha_ * x_raw + (1.0 - y_alpha_) * x_filt_;
            y_filt_ = y_alpha_ * y_raw + (1.0 - y_alpha_) * y_filt_;
            const double dt = (now - prev_frame_time_).seconds();
            if (dt > 1e-3 && dt < 1.0) {
                d_x = (x_filt_ - prev_x) / dt;    // 前后 D，用视觉帧率 dt
                d_y = (y_filt_ - prev_y) / dt;    // 横向 D，用视觉帧率 dt
            }
        }
        prev_frame_time_ = now;
        computed_seq_    = frame_seq_;

        // ② 前进 PD(★与横向完全同款★：同增益 kp_lat_/kd_lat_、同限幅 max_v_lat_)——
        //    横向线也能像纵向一样稳定跟踪。line_x>0 前进、line_x<0 后退(支持倒着寻线)。
        cmd_vfwd_ = std::clamp(kp_lat_ * x_filt_ + kd_lat_ * d_x, -max_v_lat_, max_v_lat_);

        // ③ 横向 PD：line_y>0(线在左) → v_lat>0(往左移)去够线；D 阻尼压超调/振荡。
        //    ★符号★：若实测往反方向纠(越纠越偏)，把这里 kp/kd 前加负号(或改视觉 line_y 符号)。
        cmd_vlat_ = std::clamp(kp_lat_ * y_filt_ + kd_lat_ * d_y, -max_v_lat_, max_v_lat_);

        // 诊断日志(节流0.5s)：看前后/横向偏差与命令，判前进快慢 + 纠偏方向对不对。
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
            "[寻线诊断] line_x=%.3f→v_fwd=%.2f | line_y=%.3f(左正)→v_lat=%.2f(>0左移) | yaw=0",
            x_filt_, cmd_vfwd_, y_filt_, cmd_vlat_);
    }

    // 每拍输出缓存命令(新帧刚更新；无新帧复用上一帧，平稳不打脉冲)，机头不转。
    v_fwd    = cmd_vfwd_;
    v_lat    = cmd_vlat_;
    yaw_rate = 0.0;               // ★去 yaw★：机头不转，只靠横向平移纠偏
    return true;
}

bool LineFollower::timed_out() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!last_valid_valid_) return false;   // 还没起算(未 begin)——不判超时
    return (node_->now() - last_valid_time_).seconds() >= timeout_sec_;
}

}  // namespace fly_mission
