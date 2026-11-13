#pragma once
#include "acfly_perception_common/pole_fit.hpp"

namespace multi_pole_detector {
using acfly::perception::Pole;
// IDs denote static landmarks in one map for the lifetime of the process.
class PoleIds {
 public:
  PoleIds(double distance, double weight);
  void assign(std::vector<Pole>& poles);

 private:
  struct Track {
    int id;
    double x, y;
  };
  double assoc_dist_;
  double track_ema_;
  std::vector<Track> tracks_;
  int next_id_{};
};
}  // namespace multi_pole_detector
