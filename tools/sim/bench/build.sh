#!/bin/bash
# 编译离线跟踪台架(只依赖 tracker/bezier 源码，不需要 ROS)
set -e
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
SRC=$ROOT/src/exploration_planner
g++ -O2 -std=c++17 -I$SRC/include "$ROOT/tools/sim/bench/tracker_bench.cpp" \
    $SRC/src/trajectory_tracker.cpp $SRC/src/bezier.cpp -o /tmp/tracker_bench
g++ -O2 -std=c++17 -I$SRC/include "$ROOT/tools/sim/bench/explore_bench.cpp" \
    $SRC/src/coverage_planner.cpp $SRC/src/global_planner.cpp $SRC/src/grid_map.cpp -o /tmp/explore_bench
