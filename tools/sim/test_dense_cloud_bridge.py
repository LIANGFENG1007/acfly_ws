"""Synthetic ROS-message tests; no ROS graph or simulator is started."""

import math
import unittest

import numpy as np
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

from dense_cloud_bridge import CloudSynchronizer, cloud_xyz, pose_sample, transform_cloud


def stamp(seconds):
    value = round(seconds * 1e9)
    return Time(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def odom(seconds, position=(1.0, 2.0, 3.0), quaternion=(0.0, 0.0, 0.0, 1.0)):
    result = Odometry()
    result.header.stamp = stamp(seconds)
    result.header.frame_id, result.child_frame_id = "camera_init", "body"
    p, q = result.pose.pose.position, result.pose.pose.orientation
    p.x, p.y, p.z = map(float, position)
    q.x, q.y, q.z, q.w = map(float, quaternion)
    return result


def cloud(seconds, points=((1.0, 0.0, 0.0),), big_endian=False, organized=False):
    result = PointCloud2()
    result.header.frame_id = "body"
    result.header.stamp = stamp(seconds)
    result.height = 2 if organized else 1
    result.width = len(points) // result.height
    result.point_step = 20
    result.row_step = result.width * result.point_step + 8
    result.fields = [PointField(name=name, offset=4 + index * 4, datatype=PointField.FLOAT32, count=1)
                     for index, name in enumerate(("x", "y", "z"))]
    result.is_bigendian = big_endian
    data = bytearray(result.height * result.row_step)
    for index, point in enumerate(points):
        offset = (index // result.width) * result.row_step + (index % result.width) * result.point_step + 4
        data[offset:offset + 12] = np.asarray(point, dtype=">f4" if big_endian else "<f4").tobytes()
    result.data = bytes(data)
    return result


class DenseCloudTest(unittest.TestCase):
    def test_full_rotation_and_original_stamp(self):
        message = cloud(42.0, ((1, 0, 0), (0, 1, 0), (0, 0, 1), (float("nan"), 0, 0)), True, True)
        # A 120-degree rotation around (1,1,1) cycles x->y->z->x.
        pose = pose_sample(odom(42.0, quaternion=(0.5, 0.5, 0.5, 0.5)))
        result = transform_cloud(message, pose)
        np.testing.assert_allclose(cloud_xyz(result), [[1, 3, 3], [1, 2, 4], [2, 2, 3]])
        self.assertEqual(result.header.stamp, message.header.stamp)
        self.assertEqual(result.header.frame_id, "camera_init")
        self.assertEqual(result.point_step, 12)
        self.assertEqual(result.width, 3)
        self.assertTrue(result.is_dense)

    def test_waits_for_matching_pose(self):
        sync = CloudSynchronizer()
        self.assertEqual(sync.add_pose(odom(9.9), 1000.0), [])
        self.assertEqual(sync.add_cloud(cloud(10.0), 1000.01), [])
        result = sync.add_pose(odom(10.0), 1000.02)
        self.assertEqual(len(result), 1)
        np.testing.assert_allclose(cloud_xyz(result[0]), [[2, 2, 3]])
        self.assertEqual(sync.max_match_delta_s, 0.0)

    def test_nearest_tolerance_and_timeout(self):
        warnings = []
        sync = CloudSynchronizer(warn=warnings.append)
        sync.add_pose(odom(20.0), 1.0)
        self.assertEqual(len(sync.add_cloud(cloud(20.03), 1.01)), 1)
        self.assertEqual(sync.add_cloud(cloud(20.061), 1.02), [])
        self.assertEqual(sync.flush(1.2), [])
        self.assertEqual(sync.dropped, 1)
        self.assertTrue(any("timed out" in warning for warning in warnings))

    def test_epoch_mismatch_and_reset(self):
        sync = CloudSynchronizer()
        sync.add_pose(odom(1_788_000_000.0), 200.0)
        self.assertEqual(sync.add_cloud(cloud(12.0), 200.01), [])
        sync.flush(200.2)
        self.assertEqual(sync.published, 0)
        sync.add_pose(odom(12.05), 200.21)
        self.assertEqual(len(sync.add_cloud(cloud(12.05), 200.22)), 1)
        sync.add_cloud(cloud(12.2), 200.23)
        sync.add_pose(odom(0.1), 200.24)
        self.assertEqual(len(sync.clouds), 0)
        self.assertEqual(len(sync.poses), 1)
        self.assertEqual(len(sync.add_cloud(cloud(0.1), 200.25)), 1)

    def test_queues_bounded_and_duplicate_scan_rejected(self):
        sync = CloudSynchronizer(pose_limit=3, cloud_limit=2)
        for index in range(6):
            sync.add_pose(odom(index * 0.01), 1.0)
            sync.add_cloud(cloud(10.0 + index * 0.01), 1.0)
        self.assertEqual(len(sync.poses), 3)
        self.assertEqual(len(sync.clouds), 2)
        self.assertEqual(sync.dropped, 4)
        sync.add_cloud(cloud(10.05), 1.01)
        self.assertEqual(sync.dropped, 5)

    def test_invalid_frames_pose_and_cloud(self):
        sync = CloudSynchronizer()
        sync.add_pose(odom(1.0, quaternion=(0, 0, 0, 0)), 1.0)
        self.assertEqual(len(sync.poses), 0)
        wrong = odom(1.0)
        wrong.child_frame_id = "base_link"
        sync.add_pose(wrong, 1.0)
        self.assertEqual(len(sync.poses), 0)
        sync.add_pose(odom(1.0), 1.0)
        message = cloud(1.0)
        message.header.frame_id = "camera_init"
        self.assertEqual(sync.add_cloud(message, 1.0), [])
        message = cloud(1.0)
        message.data = message.data[:-10]
        self.assertEqual(sync.add_cloud(message, 1.0), [])
        self.assertEqual(sync.dropped, 2)


if __name__ == "__main__":
    unittest.main()
