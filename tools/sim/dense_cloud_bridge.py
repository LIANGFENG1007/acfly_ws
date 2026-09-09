#!/usr/bin/env python3
"""Transform Point-LIO body scans with source-time-matched odometry for doors.

Point-LIO publish_frame_body applies the lidar-to-IMU extrinsic to the full
feats_undistort cloud. Its body frame and lidar_end_time stamp match the pose
published on /aft_mapped_to_init; no additional sensor extrinsic is needed.
"""

import argparse
from collections import deque
from dataclasses import dataclass
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid odometry quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def cloud_xyz(message):
    fields = {field.name: field for field in message.fields}
    formats, offsets = [], []
    endian = ">" if message.is_bigendian else "<"
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError("cloud requires scalar FLOAT32/FLOAT64 xyz fields")
        size = 4 if field.datatype == PointField.FLOAT32 else 8
        if field.offset + size > message.point_step:
            raise ValueError("xyz field extends beyond point_step")
        formats.append(endian + ("f4" if size == 4 else "f8"))
        offsets.append(field.offset)
    if message.row_step < message.width * message.point_step:
        raise ValueError("cloud row_step is smaller than its points")
    if len(message.data) < message.height * message.row_step:
        raise ValueError("cloud data buffer is truncated")
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": formats,
                      "offsets": offsets, "itemsize": message.point_step})
    points = np.ndarray((message.height, message.width), dtype=dtype,
                        buffer=message.data, strides=(message.row_step, message.point_step))
    xyz = np.column_stack([points[name].reshape(-1) for name in ("x", "y", "z")])
    return xyz[np.all(np.isfinite(xyz), axis=1)]


@dataclass
class PoseSample:
    stamp: int
    position: np.ndarray
    rotation: np.ndarray


def pose_sample(message):
    if message.header.frame_id != "camera_init" or message.child_frame_id != "body":
        raise ValueError("odometry must describe camera_init -> body")
    p, q = message.pose.pose.position, message.pose.pose.orientation
    position = np.asarray([p.x, p.y, p.z], dtype=np.float64)
    if not np.all(np.isfinite(position)):
        raise ValueError("invalid odometry position")
    return PoseSample(stamp_ns(message.header.stamp), position,
                      rotation_matrix([q.x, q.y, q.z, q.w]))


def transform_cloud(message, pose):
    if message.header.frame_id != "body":
        raise ValueError("input cloud must use Point-LIO body frame")
    xyz = cloud_xyz(message)
    if not len(xyz):
        raise ValueError("cloud has no finite points")
    world = np.asarray(xyz @ pose.rotation.T + pose.position, dtype="<f4")
    if not np.all(np.isfinite(world)):
        raise ValueError("transformed cloud contains non-finite coordinates")
    output = PointCloud2()
    output.header.stamp = message.header.stamp
    output.header.frame_id = "camera_init"
    output.height, output.width = 1, len(world)
    output.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                     for index, name in enumerate(("x", "y", "z"))]
    output.is_bigendian = False
    output.point_step, output.row_step = 12, 12 * len(world)
    output.is_dense = True
    output.data = world.tobytes()
    return output


class CloudSynchronizer:
    def __init__(self, tolerance_s=0.03, timeout_s=0.15, pose_limit=120, cloud_limit=4, warn=None):
        if not 0 <= tolerance_s <= 0.03 or timeout_s <= 0 or pose_limit < 1 or cloud_limit < 1:
            raise ValueError("require 0 <= tolerance <= 0.03s, positive timeout and queue sizes")
        self.tolerance_ns = round(tolerance_s * 1e9)
        self.timeout_s = timeout_s
        self.poses = deque(maxlen=pose_limit)
        self.clouds = deque()
        self.cloud_limit = cloud_limit
        self.latest = {}
        self.warn = warn or (lambda message: None)
        self.published = 0
        self.dropped = 0
        self.max_match_delta_s = 0.0

    def source_time(self, stream, stamp):
        previous = self.latest.get(stream)
        if previous is not None and stamp < previous - 500_000_000:
            self.dropped += len(self.clouds)
            self.clouds.clear()
            self.poses.clear()
            self.latest.clear()
            self.warn("source clock moved backwards; cleared pose/cloud queues")
        if stream == "cloud" and previous is not None and stamp <= previous and stream in self.latest:
            self.dropped += 1
            self.warn("duplicate or out-of-order cloud dropped")
            return False
        self.latest[stream] = max(stamp, self.latest.get(stream, stamp))
        return True

    def add_pose(self, message, received_at):
        try:
            pose = pose_sample(message)
        except ValueError as error:
            self.warn(str(error))
            return self.flush(received_at)
        self.source_time("pose", pose.stamp)
        self.poses.append(pose)
        return self.flush(received_at)

    def add_cloud(self, message, received_at):
        if message.header.frame_id != "body":
            self.dropped += 1
            self.warn("input cloud must use Point-LIO body frame")
            return self.flush(received_at)
        if self.source_time("cloud", stamp_ns(message.header.stamp)):
            if len(self.clouds) >= self.cloud_limit:
                self.clouds.popleft()
                self.dropped += 1
                self.warn("cloud queue full; dropped oldest unmatched scan")
            self.clouds.append((received_at, message))
        return self.flush(received_at)

    def flush(self, now):
        output = []
        while self.clouds:
            received_at, cloud = self.clouds[0]
            stamp = stamp_ns(cloud.header.stamp)
            # Wall time bounds waiting only. Matching compares source stamps.
            if now - received_at > self.timeout_s:
                self.clouds.popleft()
                self.dropped += 1
                self.warn("unmatched cloud timed out; no odometry within source-time tolerance")
                continue
            pose = min(self.poses, key=lambda item: (abs(item.stamp - stamp), item.stamp), default=None)
            delta = abs(pose.stamp - stamp) if pose is not None else self.tolerance_ns + 1
            if delta > self.tolerance_ns:
                break
            self.clouds.popleft()
            try:
                output.append(transform_cloud(cloud, pose))
                self.max_match_delta_s = max(self.max_match_delta_s, delta * 1e-9)
                self.published += 1
            except (ValueError, TypeError, BufferError) as error:
                self.dropped += 1
                self.warn(str(error))
        return output


class DenseCloudBridge(Node):
    def __init__(self, args):
        super().__init__("corridor_dense_cloud_bridge")
        self.last_warning = {}
        self.sync = CloudSynchronizer(args.match_tolerance_s, args.wait_timeout_s,
                                      args.pose_queue_size, args.cloud_queue_size, self.warning)
        qos = QoSProfile(depth=args.cloud_queue_size, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(PointCloud2, args.output_topic, qos)
        self.create_subscription(PointCloud2, args.input_topic,
                                 lambda msg: self.publish(self.sync.add_cloud(msg, time.monotonic())), qos)
        self.create_subscription(Odometry, args.odom_topic,
                                 lambda msg: self.publish(self.sync.add_pose(msg, time.monotonic())),
                                 QoSProfile(depth=args.pose_queue_size, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.02, lambda: self.publish(self.sync.flush(time.monotonic())))
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"{args.input_topic} + {args.odom_topic} -> {args.output_topic}; "
                               f"source-time tolerance {args.match_tolerance_s:.3f}s")

    def warning(self, message):
        now = time.monotonic()
        if now - self.last_warning.get(message, -float("inf")) >= 2.0:
            self.get_logger().warning(message)
            self.last_warning[message] = now

    def publish(self, messages):
        for message in messages:
            self.publisher.publish(message)

    def report(self):
        self.get_logger().info(f"dense scans={self.sync.published} dropped={self.sync.dropped} "
                               f"pending={len(self.sync.clouds)} "
                               f"max_match_delta={self.sync.max_match_delta_s:.6f}s")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/cloud_registered_body")
    parser.add_argument("--odom-topic", default="/aft_mapped_to_init")
    parser.add_argument("--output-topic", default="/corridor/cloud_registered_dense")
    parser.add_argument("--match-tolerance-s", type=float, default=0.03)
    parser.add_argument("--wait-timeout-s", type=float, default=0.15)
    parser.add_argument("--pose-queue-size", type=int, default=120)
    parser.add_argument("--cloud-queue-size", type=int, default=4)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = DenseCloudBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
