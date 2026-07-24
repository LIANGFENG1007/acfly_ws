// ============================================================================
//  visualizer.hpp  ── OpenCV 弹窗：栅格填充 + 规划路线 + 飞机位置/朝向/视野
//
//  render() 在节点主线程被周期调用，返回画好的 cv::Mat 由调用方 imshow。
//  画面内容：
//    - 已扫小格按填充程度着色，达 80% 的大格整格高亮（"不断填充"效果）
//    - 大格网格线
//    - 规划好的贝塞尔参考轨迹（线）
//    - 飞机位置(marker) + 朝向箭头 + 150° 扇形视野轮廓
//    - 终点(星标)
//    - 覆盖率 % 文本
// ============================================================================

#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "exploration_planner/types.hpp"
#include "exploration_planner/bezier.hpp"
#include "exploration_planner/grid_map.hpp"

namespace exploration {

class Visualizer
{
public:
    // canvas_px：画布边长(像素)。窗口按场地等比缩放。
    Visualizer(const GridConfig& grid_cfg, int canvas_px = 640);

    // 渲染一帧。pose_valid=false 时不画飞机。
    //   pois：收到的插点(目的地)位置列表，画蓝色圆点标记。
    //   obstacles：雷达拟合的障碍圆，画绿色实心圆(半径按比例缩放到正确位置)。
    //   turning/turn_dir：飞机正在【原地转身找解】时画旋转标志(turn_dir +1=逆时针/-1=顺时针)。
    //   unreachable_valid/unreachable_pos：目标【真被围死(转一圈仍无解)】时在该位置画红色叉。
    cv::Mat render(const GridMap& grid,
                   const Trajectory& traj,
                   const Vec2& goal, bool goal_valid,
                   double px, double py, double yaw, bool pose_valid,
                   const Vec2& lookahead, bool look_valid,
                   const std::vector<Vec2>& pois,
                   const Obstacles& obstacles,
                   bool turning, int turn_dir,
                   bool unreachable_valid, const Vec2& unreachable_pos);

private:
    GridConfig cfg_;
    int   W_, H_;
    double scale_;          // 米 → 像素
    double field_w_, field_h_;

    // 世界(SLAM) → 像素。y 轴翻转（图像 y 向下）。
    cv::Point to_px(double x, double y) const;
};

}  // namespace exploration
