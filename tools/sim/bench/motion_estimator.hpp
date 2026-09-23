// ============================================================================
//  motion_estimator.hpp  ── 由 SLAM 位姿(可选 IMU 陀螺)估计机体速度与航向角速度
//
//  旧做法：相邻两帧位置/航向差分，再 EMA(α=0.3) 低通。差分把 SLAM 的厘米级噪声按 1/dt
//  放大(10~20 Hz 下 1 cm 噪声 ≈ 0.14~0.28 m/s)，再经航向阻尼、横漂补偿、前进限速直接进入
//  指令 —— 离线台架在 0.5 cm/0.6° 噪声下直线巡航就出现每拍满幅的角速度来回跳，是实机"晃"的主因。
//
//  现做法：每轴一个常速度卡尔曼滤波(状态=位置,速度；过程噪声=加速度谱密度)。稳态下等效于
//  带最优增益的 α-β 滤波：同样噪声下速度噪声约为差分+EMA 的 1/3~1/5，相位滞后相当。
//  航向角速度优先用 IMU 陀螺(物理测量、几乎无噪声)，陀螺过期才退回航向滤波。
//  只输出速度/角速度；位置仍用 SLAM 原值，不给跟踪增加额外滞后。
//
//  ★2026-09-23 结论：节点未采用★(仅离线台架 tracker_bench --kf 对照用)。实机级噪声下速度误差只比
//  差分+EMA 低 20%，无噪声时滞后是其 2.6 倍；仿真归航末段因速度方向滞后触发刹停投影反复急刹。
//  抖动的真正来源是"实测速度外推"，已由 tracker 的 Smith 预估解决。
// ============================================================================
#pragma once
#include <algorithm>
#include <cmath>

namespace exploration {

class ConstantVelocityFilter {
public:
    // accel_sigma：过程噪声(加速度标准差, 单位/s²)；meas_sigma：测量噪声标准差。
    ConstantVelocityFilter(double accel_sigma = 1.5, double meas_sigma = 0.01)
        : q_(accel_sigma * accel_sigma), r_(meas_sigma * meas_sigma) {}
    void reset() { valid_ = false; v_ = 0.0; }
    // angular=true 时把新测量相对预测值的残差折回 (-π, π]。
    void update(double z, double dt, bool angular = false)
    {
        if (valid_ && !(dt > 0.0)) return;  // 重复/倒序帧不含新信息
        if (!valid_ || dt > 0.5) {          // 首帧或长时间断流：从静止重新开始
            x_ = z; v_ = 0.0;
            p00_ = r_; p01_ = 0.0; p11_ = 1.0;
            valid_ = true;
            return;
        }
        // 预测
        x_ += v_ * dt;
        const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
        p00_ += dt * (2.0 * p01_ + dt * p11_) + q_ * dt4 / 4.0;
        p01_ += dt * p11_ + q_ * dt3 / 2.0;
        p11_ += q_ * dt2;
        // 更新
        double innovation = z - x_;
        if (angular) innovation = std::atan2(std::sin(innovation), std::cos(innovation));
        const double s = p00_ + r_;
        const double k0 = p00_ / s, k1 = p01_ / s;
        x_ += k0 * innovation;
        v_ += k1 * innovation;
        const double p00 = p00_, p01 = p01_;
        p00_ -= k0 * p00;
        p01_ -= k0 * p01;
        p11_ -= k1 * p01;
        if (angular) x_ = std::atan2(std::sin(x_), std::cos(x_));
    }
    double rate() const { return v_; }
    double value() const { return x_; }
    bool valid() const { return valid_; }

private:
    double q_, r_;
    bool valid_ = false;
    double x_ = 0.0, v_ = 0.0, p00_ = 0.0, p01_ = 0.0, p11_ = 0.0;
};

struct MotionEstimatorConfig {
    double accel_sigma = 1.5;      // 水平加速度过程噪声 (m/s²)
    double pos_sigma = 0.01;       // SLAM 位置噪声 (m)
    double yaw_accel_sigma = 3.0;  // 航向角加速度过程噪声 (rad/s²)
    double yaw_sigma = 0.006;      // SLAM 航向噪声 (rad)
    double gyro_timeout = 0.10;    // 陀螺超过此时长未更新则退回航向滤波 (s)
};

class MotionEstimator {
public:
    explicit MotionEstimator(const MotionEstimatorConfig& cfg = {})
        : cfg_(cfg), fx_(cfg.accel_sigma, cfg.pos_sigma), fy_(cfg.accel_sigma, cfg.pos_sigma),
          fyaw_(cfg.yaw_accel_sigma, cfg.yaw_sigma) {}
    void reset() { fx_.reset(); fy_.reset(); fyaw_.reset(); gyro_at_ = -1e9; }
    // dt = 相邻两帧 SLAM 源时间戳间隔 (s)；received_at = 本机收到该帧的单调时钟 (s)。
    void observe_pose(double x, double y, double yaw, double dt, double received_at = 0.0)
    {
        fx_.update(x, dt); fy_.update(y, dt); fyaw_.update(yaw, dt, true);
        raw_x_ = x; raw_y_ = y; yaw_ = yaw; pose_at_ = received_at; yaw_integrated_ = 0.0;
        gyro_integrated_at_ = received_at;
    }
    // wz：机体系 z 轴角速度(rad/s，逆时针为正)；now：本机单调时钟 (s)。
    void observe_gyro(double wz, double now)
    {
        // 陀螺在两帧 SLAM 之间积分出的航向增量，供 predict_yaw 外推
        if (gyro_fresh(now) && now > gyro_integrated_at_)
            yaw_integrated_ += gyro_ * (now - std::max(gyro_at_, gyro_integrated_at_));
        gyro_ = wz; gyro_at_ = now;
        gyro_integrated_at_ = std::max(gyro_integrated_at_, now);
    }
    // ★位姿外推★：SLAM 10~20 Hz、控制 50 Hz。两帧之间若一直用旧位姿，航向/横向误差每帧阶跃一次，
    //   期望角速度随之跳变，指令只能顶着角加速度限幅走三角波(台架：直线上增幅振荡)。
    //   用估计速度与陀螺把最近一帧推到 now + lead(lead 补偿 SLAM 自身延迟)，外推时长有上限。
    double predict_x(double now, double lead, double max_age) const
    { return raw_x_ + fx_.rate() * horizon(now, lead, max_age); }
    double predict_y(double now, double lead, double max_age) const
    { return raw_y_ + fy_.rate() * horizon(now, lead, max_age); }
    double predict_yaw(double now, double lead, double max_age) const
    {
        const double h = horizon(now, lead, max_age);
        const double gyro_part = gyro_fresh(now)
            ? yaw_integrated_ + gyro_ * std::max(0.0, std::min(now, pose_at_ + max_age) - std::max(gyro_at_, gyro_integrated_at_))
            : fyaw_.rate() * (h - lead);
        const double y = yaw_ + gyro_part + yaw_rate(now) * lead;
        return std::atan2(std::sin(y), std::cos(y));
    }
    // 滤波后的位置/航向(常速度模型下无稳态滞后)。
    double x() const { return fx_.value(); }
    double y() const { return fy_.value(); }
    double yaw() const { return fyaw_.value(); }
    double world_vx() const { return fx_.rate(); }
    double world_vy() const { return fy_.rate(); }
    double v_fwd() const { return std::cos(yaw_) * fx_.rate() + std::sin(yaw_) * fy_.rate(); }
    double v_lat() const { return -std::sin(yaw_) * fx_.rate() + std::cos(yaw_) * fy_.rate(); }
    double yaw_rate(double now) const
    {
        return gyro_fresh(now) ? gyro_ : fyaw_.rate();
    }
    bool gyro_fresh(double now) const { return now - gyro_at_ >= 0.0 && now - gyro_at_ <= cfg_.gyro_timeout; }

private:
    MotionEstimatorConfig cfg_;
    ConstantVelocityFilter fx_, fy_, fyaw_;
    double horizon(double now, double lead, double max_age) const
    { return std::clamp(now - pose_at_, 0.0, max_age) + lead; }
    double yaw_ = 0.0, gyro_ = 0.0, gyro_at_ = -1e9;
    double raw_x_ = 0.0, raw_y_ = 0.0, pose_at_ = 0.0;
    double yaw_integrated_ = 0.0, gyro_integrated_at_ = -1e9;
};

}  // namespace exploration
