#include "exploration_node.hpp"
namespace exploration {
ExplorationNode::ExplorationNode() : Node("exploration_planner_node") {
  configure();
  defaults_.check(*this);
  connect_interfaces();
  last_plan_time_ = last_global_plan_time_ = retreat_start_ = now();
  timer_ = create_wall_timer(std::chrono::milliseconds(timer_period_ms_), [this] { on_timer(); });
  RCLCPP_INFO(get_logger(), "区域测绘就绪：场地 [%.2f,%.2f] × [%.2f,%.2f]，覆盖目标 %.1f%%",
              gcfg_.min_x, gcfg_.max_x, gcfg_.min_y, gcfg_.max_y, 100 * search_coverage_);
}
} // namespace exploration
