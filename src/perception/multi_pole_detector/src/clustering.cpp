#include "clustering.hpp"
namespace multi_pole_detector {
std::vector<std::vector<int>> cluster_xy(const Points& pts, double tolerance) {
  const int n = static_cast<int>(pts.size());
  std::vector<std::vector<int>> clusters;
  std::vector<char> visited(n, 0);
  const double tol2 = tolerance * tolerance;
  std::vector<int> stack;
  for (int i = 0; i < n; ++i) {
    if (visited[i]) continue;
    // 从 i 起 BFS/DFS 收一个连通分量
    std::vector<int> comp;
    stack.clear();
    stack.push_back(i);
    visited[i] = 1;
    while (!stack.empty()) {
      const int c = stack.back();
      stack.pop_back();
      comp.push_back(c);
      const double cx = pts[c].x(), cy = pts[c].y();
      for (int j = 0; j < n; ++j) {
        if (visited[j]) continue;
        const double ddx = pts[j].x() - cx, ddy = pts[j].y() - cy;
        if (ddx * ddx + ddy * ddy <= tol2) {
          visited[j] = 1;
          stack.push_back(j);
        }
      }
    }
    clusters.push_back(std::move(comp));
  }
  return clusters;
}

}  // namespace multi_pole_detector
