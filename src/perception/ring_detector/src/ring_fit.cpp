#include "ring_fit.hpp"

#include <Eigen/Dense>
namespace ring_detector {
bool fit_circle(const acfly::perception::Points& pts, Eigen::Vector3d& center, double& radius,
                Eigen::Vector3d& e1, Eigen::Vector3d& e2) {
  const size_t n = pts.size();
  if (n < 3) return false;

  // 均值(作 PCA 原点 + 投影基准)
  Eigen::Vector3d mean(0, 0, 0);
  for (const auto& p : pts) mean += p.cast<double>();
  mean /= static_cast<double>(n);

  // PCA：协方差最大两特征向量 = 环平面内两正交轴(e1,e2)
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& p : pts) {
    const Eigen::Vector3d d = p.cast<double>() - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(n);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);  // 特征值升序
  if (es.info() != Eigen::Success) return false;
  e1 = es.eigenvectors().col(2).normalized();  // 最大方向
  e2 = es.eigenvectors().col(1).normalized();  // 次大方向(与 e1 正交，同在环平面)

  // 各点投影到 (e1,e2) 平面得 2D 坐标 (u,v)，相对 mean
  // Kåsa 代数圆拟合：解 [2u 2v 1][a b cc]^T = u²+v²，圆心(a,b)、半径=sqrt(cc+a²+b²)
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d bvec = Eigen::Vector3d::Zero();
  for (const auto& p : pts) {
    const Eigen::Vector3d d = p.cast<double>() - mean;
    const double u = d.dot(e1), v = d.dot(e2);
    const double uu = u * u + v * v;
    Eigen::Vector3d row(2.0 * u, 2.0 * v, 1.0);
    A += row * row.transpose();
    bvec += row * uu;
  }
  // 解 3x3(用 LDLT，A 对称半正定)
  Eigen::Vector3d sol = A.ldlt().solve(bvec);
  const double a = sol(0), b = sol(1), cc = sol(2);
  const double r2 = cc + a * a + b * b;
  if (!(r2 > 0) || !sol.allFinite()) return false;
  radius = std::sqrt(r2);

  // 2D 圆心 (a,b) 变回 3D：mean + a*e1 + b*e2
  center = mean + a * e1 + b * e2;
  return true;
}

}  // namespace ring_detector
