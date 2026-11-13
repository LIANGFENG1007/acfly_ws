"""Exercise installed perception nodes on synthetic clouds in a localhost ROS domain."""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Float64MultiArray, Header


def circle(cx, cy, radius, vertical=False):
    for i in range(24):
        angle = 2 * math.pi * i / 24
        a, b = radius * math.cos(angle), radius * math.sin(angle)
        yield (cx, cy + a, 1 + b) if vertical else (cx + a, cy + b, 1.0)


class InstalledDetectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ.setdefault("ROS_DOMAIN_ID", "179")
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def check_detector(self, package, points, expected):
        directory = Path(get_package_share_directory(package))
        self.assertTrue((directory / "rviz" / f"{package}.rviz").is_file())
        executable = Path(get_package_prefix(package)) / "lib" / package / f"{package}_node"
        prefix = f"/perception_regression/{package}"
        with tempfile.TemporaryDirectory() as tmp:
            # All samples are outside the default ROI. The file and CLI must both take effect.
            overlay = Path(tmp) / "profile.yaml"
            overlay.write_text(
                f"/**:\n  ros__parameters:\n    cloud_topic: {prefix}/input\n"
                f"    center_topic: {prefix}/measurement\n"
                "    box_min_x: 0.0\n"
            )
            with subprocess.Popen([
                str(executable), "--ros-args", "--params-file", str(overlay),
                "-p", "box_max_x:=10", "-p", "accum_frames:=1.0",
                "-r", f"__node:={package}_under_test",
            ]) as child:
                node = rclpy.create_node(f"{package}_probe")
                try:
                    # Defaults are double for the ROI and integer for the frame count.
                    # Inspect both effective values after overriding in the opposite numeric type.
                    client = node.create_client(GetParameters, f"/{package}_under_test/get_parameters")
                    self.assertTrue(client.wait_for_service(timeout_sec=4))
                    future = client.call_async(GetParameters.Request(names=["box_max_x", "accum_frames"]))
                    rclpy.spin_until_future_complete(node, future, timeout_sec=4)
                    self.assertTrue(future.done())
                    xmax, frames = future.result().values
                    self.assertEqual((xmax.type, xmax.integer_value), (ParameterType.PARAMETER_INTEGER, 10))
                    self.assertEqual((frames.type, frames.double_value), (ParameterType.PARAMETER_DOUBLE, 1.0))
                    results = []
                    node.create_subscription(
                        Float64MultiArray, prefix + "/measurement",
                        lambda msg: results.append(list(msg.data)), 10,
                    )
                    pub = node.create_publisher(PointCloud2, prefix + "/input", 10)
                    msg = create_cloud_xyz32(Header(frame_id="camera_init"), points)
                    end = time.monotonic() + 10
                    while not results and time.monotonic() < end:
                        self.assertIsNone(child.poll(), f"{package} exited before publishing")
                        msg.header.stamp = node.get_clock().now().to_msg()
                        pub.publish(msg)
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertTrue(results, f"{package} published no center")
                    self.assertEqual(len(results[0]), len(expected))
                    for actual, want in zip(results[0], expected):
                        self.assertAlmostEqual(actual, want, places=4)
                finally:
                    node.destroy_node()
                    if child.poll() is None:
                        child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        child.kill()

    def test_reject_invalid_parameters(self):
        cases = [
            ("pole_detector", "min_pointz:=10", "min_pointz"),
            ("pole_detector", "min_points:=false", "min_points"),
            ("ring_detector", "box_min_x:=12", "each box_min must be < box_max"),
            ("multi_pole_detector", "track_ema:=1.4", "track_ema must be in [0, 1]"),
        ]
        for package, argument, error in cases:
            with self.subTest(parameter=argument):
                binary = Path(get_package_prefix(package)) / "lib" / package / f"{package}_node"
                result = subprocess.run(
                    [str(binary), "--ros-args", "-p", argument], check=False,
                    capture_output=True, text=True, timeout=4,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(error, result.stderr)

    def test_single_pole(self):
        self.check_detector("pole_detector", list(circle(8, 0, 0.03)),
                            [8, 0, 1, 0, 0.03])

    def test_multiple_poles(self):
        samples = list(circle(8, 0, 0.03)) + list(circle(9, 0, 0.04))
        self.check_detector("multi_pole_detector", samples,
                            [0, 8, 0, 1, 0, 0.03, 1, 9, 0, 1, 0, 0.04])

    def test_ring(self):
        self.check_detector("ring_detector", list(circle(8, 0, 0.5, vertical=True)),
                            [8, 0, 1, 0, 0])


if __name__ == "__main__":
    unittest.main()
