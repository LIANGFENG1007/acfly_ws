#!/usr/bin/env bash
# Build and test a copy of src in a clean ROS environment; leave the logs for review.
set -e
repo=$(cd "$(dirname "$0")/.." && pwd)
work=$(mktemp -d "${TMPDIR:-/tmp}/acfly-survey-test-XXXXXXXX")
echo "隔离验证目录：$work"
cp -a "$repo/src" "$work/src"
/usr/bin/env -i HOME="$HOME" USER="${USER:-$(id -un)}" LANG=C.UTF-8 PATH=/usr/bin:/bin \
  ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$work/ros-log" CMAKE_BUILD_PARALLEL_LEVEL=2 \
  /bin/bash -ec '
    cd "$1"
    source /opt/ros/humble/setup.bash
    colcon build --base-paths src --executor sequential --cmake-args -DBUILD_TESTING=ON
    source install/setup.bash
    colcon test --executor sequential --return-code-on-test-failure || status=$?
    colcon test-result --verbose || exit 1
    if [[ "${status:-0}" != 0 ]]; then exit "$status"; fi
    /usr/bin/python3 src/perception/tests/test_detectors.py -v
  ' bash "$work"
