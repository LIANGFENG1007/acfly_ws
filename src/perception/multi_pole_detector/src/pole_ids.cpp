#include "pole_ids.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace multi_pole_detector {
PoleIds::PoleIds(double distance, double weight) : assoc_dist_(distance), track_ema_(weight) {
  if (!(weight >= 0 && weight <= 1)) throw std::invalid_argument("track_ema must be in [0, 1]");
}
void PoleIds::assign(std::vector<Pole>& poles) {
  const int np = static_cast<int>(poles.size());
  const int nt = static_cast<int>(tracks_.size());

  // 收集所有 (杆 i, track t, 距离) 且距离<assoc_dist_ 的候选，按距离升序贪心匹配
  struct Pair {
    double d;
    int i;
    int t;
  };
  std::vector<Pair> pairs;
  for (int i = 0; i < np; ++i)
    for (int t = 0; t < nt; ++t) {
      const double ddx = poles[i].x - tracks_[t].x, ddy = poles[i].y - tracks_[t].y;
      const double d = std::hypot(ddx, ddy);
      if (d < assoc_dist_) pairs.push_back({d, i, t});
    }
  std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d < b.d; });

  std::vector<char> pole_used(np, 0), track_used(nt, 0);
  for (const auto& pr : pairs) {
    if (pole_used[pr.i] || track_used[pr.t]) continue;
    poles[pr.i].id = tracks_[pr.t].id;
    // 低通更新该编号位置(跟随杆的最新观测，平滑抖动)
    tracks_[pr.t].x = track_ema_ * poles[pr.i].x + (1.0 - track_ema_) * tracks_[pr.t].x;
    tracks_[pr.t].y = track_ema_ * poles[pr.i].y + (1.0 - track_ema_) * tracks_[pr.t].y;
    pole_used[pr.i] = 1;
    track_used[pr.t] = 1;
  }
  // 没认领到已知编号的杆 → 新建编号(位置不属于任何已知 track)
  for (int i = 0; i < np; ++i) {
    if (pole_used[i]) continue;
    Track tk;
    tk.id = next_id_++;
    tk.x = poles[i].x;
    tk.y = poles[i].y;
    poles[i].id = tk.id;
    tracks_.push_back(tk);
  }
}

}  // namespace multi_pole_detector
