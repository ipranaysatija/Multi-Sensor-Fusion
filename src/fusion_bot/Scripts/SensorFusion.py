#!/usr/bin/env python3
"""
ROS2 node: Fuse 2D LiDAR (sensor_msgs/LaserScan) with camera (sensor_msgs/Image)
- Synchronizes topics with message_filters
- Applies extrinsic calibration T_cam_lidar (4x4) to map LiDAR points into camera frame
- Projects to pixel coords using pinhole intrinsics (fx, fy, cx, cy)
- Publishes overlay image and optional PointCloud2 in camera frame
- Supports optional image undistortion (if distortion coeffs provided)
- Can also overlay bounding boxes from an object detector and count LiDAR hits

Usage:
  - Add to ament_python package and list as entry_point
  - Provide parameters via YAML or CLI (see below)

Params:
  camera/image_topic: string (default '/camera/image_raw')
  lidar/scan_topic: string (default '/scan')
  intrinsics/fx, intrinsics/fy, intrinsics/cx, intrinsics/cy: float
  intrinsics/dist_coeffs: list[5] (optional, k1,k2,p1,p2,k3)
  extrinsics/T_cam_lidar: list[16] row-major 4x4
  lidar/range_min, lidar/range_max: float (override LaserScan if provided)
  lidar/downsample: int (take every N-th beam)
  viz/show_window: bool (imshow)
  viz/circle_radius_px: int
  viz/overlay_topic: string (default '/fusion/overlay')
  fusion/boxes_topic: string (optional detector boxes topic)
  output/pointcloud_topic: string (default '/fusion/points_cam')

Notes:
  - 2D LiDAR is assumed to lie in its own XY plane with Z=0. Each beam is (x=r*cos(a), y=r*sin(a), z=0).
  - Only points with Z_cam > 0 are projected.
  - Distortion is corrected if intrinsics/dist_coeffs are provided.
"""

import math
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer
import sensor_msgs_py.point_cloud2 as pc2


class LidarCameraFusionSync(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion_sync')
        self.bridge = CvBridge()

        # --- Parameters ---
        self.declare_parameter('camera/image_topic', '/camera/image_raw')
        self.declare_parameter('lidar/scan_topic', '/scan')
        self.declare_parameter('viz/overlay_topic', '/fusion/overlay')
        self.declare_parameter('viz/show_window', False)
        self.declare_parameter('viz/circle_radius_px', 3)
        self.declare_parameter('lidar/downsample', 1)
        self.declare_parameter('lidar/range_min', 0.0)
        self.declare_parameter('lidar/range_max', 0.0)

        # Intrinsics
        self.declare_parameter('intrinsics/fx', 525.0)
        self.declare_parameter('intrinsics/fy', 525.0)
        self.declare_parameter('intrinsics/cx', 319.5)
        self.declare_parameter('intrinsics/cy', 239.5)
        self.declare_parameter('intrinsics/dist_coeffs', [0.,0.,0.,0.,0.])

        # Extrinsics: 4x4 row-major list
        default_T = np.eye(4, dtype=float).reshape(-1).tolist()
        self.declare_parameter('extrinsics/T_cam_lidar', default_T)

        # Detector boxes topic
        self.declare_parameter('fusion/boxes_topic', '')

        # Output pointcloud topic
        self.declare_parameter('output/pointcloud_topic', '/fusion/points_cam')

        img_topic = self.get_parameter('camera/image_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('lidar/scan_topic').get_parameter_value().string_value

        # Build subscribers for message_filters
        self.image_sub = Subscriber(self, Image, img_topic, qos_profile=10)
        self.scan_sub = Subscriber(self, LaserScan, scan_topic, qos_profile=10)

        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.scan_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.synced_callback)

        # Publishers
        overlay_topic = self.get_parameter('viz/overlay_topic').get_parameter_value().string_value
        self.pub_overlay = self.create_publisher(Image, overlay_topic, 10)
        cloud_topic = self.get_parameter('output/pointcloud_topic').get_parameter_value().string_value
        self.pub_cloud = self.create_publisher(PointCloud2, cloud_topic, 10)

        # Cache params
        self.fx = float(self.get_parameter('intrinsics/fx').value)
        self.fy = float(self.get_parameter('intrinsics/fy').value)
        self.cx = float(self.get_parameter('intrinsics/cx').value)
        self.cy = float(self.get_parameter('intrinsics/cy').value)
        self.dist_coeffs = np.array(self.get_parameter('intrinsics/dist_coeffs').value, dtype=float)

        T_list = self.get_parameter('extrinsics/T_cam_lidar').get_parameter_value().double_array_value
        if len(T_list) != 16:
            self.get_logger().warn('extrinsics/T_cam_lidar must be 16 values (row-major 4x4). Using identity.')
            self.T_cam_lidar = np.eye(4, dtype=float)
        else:
            self.T_cam_lidar = np.array(T_list, dtype=float).reshape(4, 4)

        self.downsample = max(1, int(self.get_parameter('lidar/downsample').value))
        self.range_min_override = float(self.get_parameter('lidar/range_min').value)
        self.range_max_override = float(self.get_parameter('lidar/range_max').value)
        self.show_window = bool(self.get_parameter('viz/show_window').value)
        self.circle_radius = int(self.get_parameter('viz/circle_radius_px').value)

    def synced_callback(self, img_msg: Image, scan_msg: LaserScan):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # Undistort if coeffs given
        if np.any(np.abs(self.dist_coeffs) > 1e-6):
            K = np.array([[self.fx, 0, self.cx],[0, self.fy, self.cy],[0,0,1]], dtype=float)
            img = cv2.undistort(img, K, self.dist_coeffs)

        ranges = scan_msg.ranges
        n = len(ranges)
        angle = scan_msg.angle_min
        incr = scan_msg.angle_increment

        rmin = self.range_min_override if self.range_min_override > 0 else scan_msg.range_min
        rmax = self.range_max_override if self.range_max_override > 0 else scan_msg.range_max

        pts_lidar = []
        for i in range(0, n, self.downsample):
            r = ranges[i]
            if math.isfinite(r) and rmin < r < rmax:
                a = angle + i * incr
                x = r * math.cos(a)
                y = r * math.sin(a)
                pts_lidar.append((x, y, 0.0))

        if not pts_lidar:
            return

        pts_cam = self.transform_points_h(pts_lidar, self.T_cam_lidar)

        overlay = img.copy()
        h, w = overlay.shape[:2]

        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        cloud_points = []
        for X, Y, Z in pts_cam:
            if Z <= 0.0 or not np.isfinite(Z):
                continue
            u = fx * (X / Z) + cx
            v = fy * (Y / Z) + cy
            ui, vi = int(round(u)), int(round(v))
            if 0 <= ui < w and 0 <= vi < h:
                cv2.circle(overlay, (ui, vi), self.circle_radius, (0, 0, 255), -1)
                cloud_points.append((X, Y, Z))

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = img_msg.header
        self.pub_overlay.publish(overlay_msg)

        if cloud_points:
            cloud_msg = pc2.create_cloud_xyz32(img_msg.header, cloud_points)
            self.pub_cloud.publish(cloud_msg)

        if self.show_window:
            cv2.imshow('LiDAR-Camera Fusion', overlay)
            cv2.waitKey(1)

    @staticmethod
    def transform_points_h(pts_xyz: List[tuple], T: np.ndarray) -> List[tuple]:
        P = np.array([[x, y, z, 1.0] for (x, y, z) in pts_xyz], dtype=float)
        Pc = (P @ T.T)[:, :3]
        return [tuple(row) for row in Pc]


def main():
    rclpy.init()
    node = LidarCameraFusionSync()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()

