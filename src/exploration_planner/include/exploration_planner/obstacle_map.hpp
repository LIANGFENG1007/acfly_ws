// ============================================================================
//  obstacle_map.hpp  ── 雷达点云 → 障碍物圆（聚类拟合）
//
//  输入：每帧 /cloud_registered 的世界系 2D 点（节点已滤掉地面 z 与自身回波）。
//  做法：把点累计进一张"占据栅格"(分辨率 OBS_CELL)；命中达阈值的格子视为占据；
//        对占据格做连通域(8邻)聚类；每簇拟合成圆(圆心=质心, 半径=覆盖簇内点的最大距离)。
//  输出：std::vector<Obstacle>（圆心+半径，SLAM 系）。
//
//  设计：持久累计(静态世界)，所以"雷达一开就慢慢把已见障碍固定下来"。
//        内部自带互斥锁：cloud 回调线程写、主循环/可视化线程读，互不打架。
// ============================================================================

#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include "exploration_planner/types.hpp"
                                                              
namespace exploration {

struct ObstacleConfig {
    double min_x, min_y, max_x, max_y;   // 场地范围 (SLAM 系, m)
    double cell;                         // 占据栅格分辨率 (m)
    double edge_ignore;                  // 距墙此范围内的点丢弃 (m)
    int    hit_thresh;                   // 占据格判定阈值(滑动累计值达此算占据)
    int    hit_inc;                      // 本帧命中 +此值
    int    hit_max;                      // 累计封顶
    int    hit_decay;                    // 本帧未命中 -此值(衰减，离开的障碍格慢慢消失)
    int    min_cells;                    // 一簇最少占据格数
    double min_r, max_r;                 // 拟合圆半径裁剪 (m)
    double r_inflate;                    // 半径额外充气 (m)
    // ★时序平滑(障碍跟踪)★：逐帧重新聚类拟合的圆会闪动/瞬移。把本帧测量圆关联到上一帧的
    //   "同一个障碍"，对圆心/半径做指数滑动平均(EMA，偏重最近、老数据淡出)，压住跳变。
    double track_alpha;                  // EMA 系数(0~1)：越大越跟手(响应快但抖)，越小越平滑(稳但滞后)
    double track_assoc_dist;             // 关联门限 (m)：测量圆中心距某 track < 此值才算"同一个障碍"
    int    track_max_misses;             // 连续多少帧没再看到就删该障碍(最近一段时间没出现=已离开视野)
    // ★latch 智能回收(防幽灵)★：latch 格若【当前在视野内】却【连续 ghost_clear_frames 帧无点命中】→
    //   判为幽灵(早已不存在/曾是噪声)，撤销 latch。看不到的 latch 格冻结不回收(不误伤真障碍)。
    double fov_deg;                      // 视野总开角 (度)：判定 latch 格是否在当前视野内
    double fov_range;                    // 视野半径 (m)
    int    ghost_clear_frames;           // 视野内连续无命中达此帧数 → 撤销 latch(<=0 关闭回收=纯永久 latch)
};

class ObstacleMap
{
public:
    explicit ObstacleMap(const ObstacleConfig& cfg);

    // 喂入一帧世界系 2D 点（已滤地面/自身回波）：累计进占据栅格并重建障碍圆列表。
    //   px,py,yaw/pose_valid：本帧观测位姿(SLAM系)，用于 latch 智能回收判定"格是否在当前视野内"。
    //   pose_valid=false 时跳过回收(不知视野→不敢撤 latch)。
    void integrate(const std::vector<Vec2>& pts,
                   double px = 0.0, double py = 0.0, double yaw = 0.0, bool pose_valid = false);

    // 取当前障碍圆列表的拷贝（线程安全）。
    Obstacles snapshot() const;

    // 取当前【占据栅格格中心点】的拷贝(SLAM/世界系 2D)。即算法真正当障碍的点(已滤地面/自身/
    //   范围外/贴墙,且经滑动累计稳定下来)——发布到 rviz 看"处理后的点"用。线程安全。
    std::vector<Vec2> occupied_points() const;

private:
    ObstacleConfig cfg_;
    int nx_, ny_;                        // 占据栅格尺寸
    std::vector<uint16_t> hit_;          // 命中累计(饱和)，行优先 idx = ix*ny_+iy
    // ★latch 永久累积★：某格 hit_ 达到过 hit_thresh 即永久置位，此后恒为占据——即使 hit_ 之后
    //   衰减到 0 也不撤销。用途：静态障碍(杆/凳不动)"一旦识别就永久记住、绝不消失"。半径不受
    //   影响：仍每帧按【当前占据格(含 latched)】重新聚类拟合，所以障碍大小照常随观测变化。
    std::vector<uint8_t>  latched_;      // 1=该格曾达占据阈值→永久占据
    std::vector<uint16_t> ghost_miss_;   // latch 智能回收：该 latch 格【在视野内却无命中】的连续帧数，超阈值撤销 latch
    Obstacles obstacles_;                // 最近一次"时序平滑后"对外输出的障碍圆
    std::vector<Vec2> occ_points_;       // 最近一次重建时的占据格中心点(rviz 可视化用)

    // 时序平滑跟踪：每个障碍维持一个平滑圆，逐帧把新测量 EMA 进来，避免闪动/瞬移。
    struct Track {
        double cx, cy, r;   // 平滑后的圆心+半径(对外输出的就是它)
        int    misses;      // 连续未关联到测量的帧数(超 track_max_misses 删除)
        bool   matched;     // 本帧是否关联到测量(内部临时用)
    };
    std::vector<Track> tracks_;          // 当前存活的障碍跟踪

    mutable std::mutex mtx_;             // 保护 hit_ / obstacles_ / tracks_

    inline int idx(int ix, int iy) const { return ix * ny_ + iy; }
    bool in_grid(int ix, int iy) const { return ix >= 0 && ix < nx_ && iy >= 0 && iy < ny_; }

    // 在 mtx_ 已持有时调用：扫描占据格做连通域聚类拟合圆，得到本帧"测量圆"，再交时序平滑。
    void rebuild_locked();

    // 在 mtx_ 已持有时调用：把本帧测量圆 meas 关联到已有 track 并 EMA 平滑，更新 obstacles_。
    void track_and_smooth(const Obstacles& meas);
};

}  // namespace exploration
