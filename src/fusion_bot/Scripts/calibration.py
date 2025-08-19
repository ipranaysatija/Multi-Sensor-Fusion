#!/usr/bin/env python3
"""
Extended ROS2 node features:
- tf2 lookup of T_cam_lidar using frames (camera_optical_frame, lidar_frame)
- Optional image undistortion via (k1,k2,p1,p2,k3) and (fx,fy,cx,cy)
- Publish PointCloud2 of LiDAR points transformed into camera frame
- Optional 2D detection fusion: subscribe to vision_msgs/Detection2DArray and count projected LiDAR hits per bbox
"""

import math
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer

import tf2_ros
from builtin_interfaces.msg import Time as RosTime

try:
    from vision_msgs.msg import Detection2DArray
    HAVE_VISION_MSGS = True
except Exception:
    HAVE_VISION_MSGS = False


class LidarCameraFusionPro(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion_pro')
        self.bridge = CvBridge()

        # --- Parameters ---
        # Topics
        self.declare_parameter('camera/image_topic', '/camera/image_raw')
        self.declare_parameter('lidar/scan_topic', '/scan')
        self.declare_parameter('viz/overlay_topic', '/fusion/overlay')
        self.declare_parameter('cloud/topic', '/fusion/points_cam')
        self.declare_parameter('detections/topic', '/detections')

        # Frames for tf2
        self.declare_parameter('frames/camera_optical', 'camera_optical_frame')
        self.declare_parameter('frames/lidar', 'laser')
        self.declare_parameter('tf2/use', True)
        self.declare_parameter('tf2/timeout_sec', 0.05)

        # Intrinsics
        self.declare_parameter('intrinsics/fx', 525.0)
        self.declare_parameter('intrinsics/fy', 525.0)
        self.declare_parameter('intrinsics/cx', 319.5)
        self.declare_parameter('intrinsics/cy', 239.5)

        # Distortion (rad-tan)
        self.declare_parameter('distortion/enable', False)
        self.declare_parameter('distortion/k1', 0.0)
        self.declare_parameter('distortion/k2', 0.0)
        self.declare_parameter('distortion/p1', 0.0)
        self.declare_parameter('distortion/p2', 0.0)
        self.declare_parameter('distortion/k3', 0.0)

        # Manual extrinsics fallback
        default_T = np.eye(4, dtype=float).reshape(-1).tolist()
        self.declare_parameter('extrinsics/T_cam_lidar', default_T)

        # Misc
        self.declare_parameter('viz/show_window', False)
        self.declare_parameter('viz/circle_radius_px', 3)
        self.declare_parameter('lidar/downsample', 1)
        self.declare_parameter('lidar/range_min', 0.0)
        self.declare_parameter('lidar/range_max', 0.0)

        # Fetch params
        self.fx = float(self.get_parameter('intrinsics/fx').value)
        self.fy = float(self.get_parameter('intrinsics/fy').value)
        self.cx = float(self.get_parameter('intrinsics/cx').value)
        self.cy = float(self.get_parameter('intrinsics/cy').value)

        self.enable_dist = bool(self.get_parameter('distortion/enable').value)
        self.k1 = float(self.get_parameter('distortion/k1').value)
        self.k2 = float(self.get_parameter('distortion/k2').value)
        self.p1 = float(self.get_parameter('distortion/p1').value)
        self.p2 = float(self.get_parameter('distortion/p2').value)
        self.k3 = float(self.get_parameter('distortion/k3').value)

        self.use_tf2 = bool(self.get_parameter('tf2/use').value)
        self.tf_timeout = float(self.get_parameter('tf2/timeout_sec').value)
        self.cam_frame = self.get_parameter('frames/camera_optical').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('frames/lidar').get_parameter_value().string_value

        T_list = self.get_parameter('extrinsics/T_cam_lidar').get_parameter_value().double_array_value
        if len(T_list) == 16:
            self.T_cam_lidar_param = np.array(T_list, dtype=float).reshape(4,4)
        else:
            self.T_cam_lidar_param = np.eye(4)

        img_topic = self.get_parameter('camera/image_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('lidar/scan_topic').get_parameter_value().string_value
        overlay_topic = self.get_parameter('viz/overlay_topic').get_parameter_value().string_value
        cloud_topic = self.get_parameter('cloud/topic').get_parameter_value().string_value

        self.downsample = max(1, int(self.get_parameter('lidar/downsample').value))
        self.range_min_override = float(self.get_parameter('lidar/range_min').value)
        self.range_max_override = float(self.get_parameter('lidar/range_max').value)
        self.show_window = bool(self.get_parameter('viz/show_window').value)
        self.circle_radius = int(self.get_parameter('viz/circle_radius_px').value)

        # Subscribers via message_filters (image + scan)
        self.image_sub = Subscriber(self, Image, img_topic, qos_profile=10)
        self.scan_sub = Subscriber(self, LaserScan, scan_topic, qos_profile=10)
        self.sync = ApproximateTimeSynchronizer([self.image_sub, self.scan_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.synced_callback)

        # Optional detection subscriber (not synchronized; use latest)
        self.latest_dets = None
        det_topic = self.get_parameter('detections/topic').get_parameter_value().string_value
        if HAVE_VISION_MSGS:
            self.create_subscription(Detection2DArray, det_topic, self.dets_cb, 10)
        else:
            self.get_logger().warn('vision_msgs not found; bbox fusion disabled.')

        # Publishers
        self.pub_overlay = self.create_publisher(Image, overlay_topic, 10)
        self.pub_cloud = self.create_publisher(PointCloud2, cloud_topic, 10)

        # tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Precompute matrices for undistort
        self.K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0,0,1]], dtype=np.float32)
        self.D = np.array([self.k1, self.k2, self.p1, self.p2, self.k3], dtype=np.float32)

        self.get_logger().info(f"FusionPro: image='{img_topic}', scan='{scan_topic}', overlay='{overlay_topic}', cloud='{cloud_topic}'")

    def dets_cb(self, msg):
        self.latest_dets = msg

    def get_T_cam_lidar(self, stamp) -> np.ndarray:
        if not self.use_tf2:
            return self.T_cam_lidar_param
        try:
            ts = rclpy.time.Time.from_msg(stamp)
            trans = self.tf_buffer.lookup_transform(self.cam_frame, self.lidar_frame, ts, rclpy.duration.Duration(seconds=self.tf_timeout))
            T = self.transform_to_matrix(trans)
            return T
        except Exception as e:
            self.get_logger().warn(f'tf2 lookup failed ({self.cam_frame}<-{self.lidar_frame}): {e}; using param extrinsics')
            return self.T_cam_lidar_param

    @staticmethod
    def transform_to_matrix(t):
        # geometry_msgs/TransformStamped -> 4x4
        q = t.transform.rotation
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        # quaternion to rotation matrix
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1-2*(qy*qy+qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx*qx+qy*qy)]
        ], dtype=float)
        T = np.eye(4)
        T[:3,:3] = R
        T[:3, 3] = [x,y,z]
        return T

    def synced_callback(self, img_msg: Image, scan_msg: LaserScan):
        # Convert image
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # Optional undistortion
        if self.enable_dist:
            img = cv2.undistort(img, self.K, self.D[:5])

        # Build LiDAR points (XY, Z=0)
        ranges = scan_msg.ranges
        n = len(ranges)
        angle0 = scan_msg.angle_min
        incr = scan_msg.angle_increment
        rmin = self.range_min_override if self.range_min_override > 0 else scan_msg.range_min
        rmax = self.range_max_override if self.range_max_override > 0 else scan_msg.range_max

        pts_lidar = []  # (x,y,0)
        for i in range(0, n, self.downsample):
            r = ranges[i]
            if math.isfinite(r) and rmin < r < rmax:
                a = angle0 + i * incr
                pts_lidar.append((r*math.cos(a), r*math.sin(a), 0.0))
        if not pts_lidar:
            return

        # Transform via tf2 or param
        T = self.get_T_cam_lidar(scan_msg.header.stamp)
        P = np.c_[np.array(pts_lidar), np.ones(len(pts_lidar))]  # (N,4)
        Pc = (P @ T.T)[:, :3]  # (N,3) in camera frame

        # Publish PointCloud2
        cloud = self.points_to_cloud(Pc, frame_id=self.cam_frame, stamp=img_msg.header.stamp)
        self.pub_cloud.publish(cloud)

        # Project to pixels & overlay
        overlay = img.copy()
        h, w = overlay.shape[:2]
        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        pixels = []
        for X, Y, Z in Pc:
            if Z <= 0 or not np.isfinite(Z):
                pixels.append(None)
                continue
            u = fx * (X / Z) + cx
            v = fy * (Y / Z) + cy
            if 0 <= u < w and 0 <= v < h:
                ui, vi = int(round(u)), int(round(v))
                pixels.append((ui, vi))
                cv2.circle(overlay, (ui, vi), self.circle_radius, (0, 0, 255), -1)
            else:
                pixels.append(None)

        # Optional bbox fusion
        if HAVE_VISION_MSGS and self.latest_dets is not None:
            counts = self.count_points_in_bboxes(pixels, self.latest_dets, w, h)
            # draw boxes with counts
            for det, cnt in counts:
                bb = det.bbox
                cx_det, cy_det = bb.center.x, bb.center.y
                w_det, h_det = bb.size_x, bb.size_y
                x1 = int(cx_det - w_det/2)
                y1 = int(cy_det - h_det/2)
                x2 = int(cx_det + w_det/2)
                y2 = int(cy_det + h_det/2)
                x1 = max(0, min(w-1, x1)); y1 = max(0, min(h-1, y1))
                x2 = max(0, min(w-1, x2)); y2 = max(0, min(h-1, y2))
                cv2.rectangle(overlay, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(overlay, f"{cnt}", (x1, max(0,y1-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # Publish overlay
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = img_msg.header
        overlay_msg.header.frame_id = self.cam_frame
        self.pub_overlay.publish(overlay_msg)

        if self.show_window:
            cv2.imshow('FusionPro', overlay)
            cv2.waitKey(1)

    def count_points_in_bboxes(self, pixels, det_array, img_w, img_h):
        results = []
        for det in det_array.detections:
            bb = det.bbox
            cx, cy = bb.center.x, bb.center.y
            w, h = bb.size_x, bb.size_y
            x1 = max(0, int(cx - w/2)); y1 = max(0, int(cy - h/2))
            x2 = min(img_w-1, int(cx + w/2)); y2 = min(img_h-1, int(cy + h/2))
            cnt = 0
            for p in pixels:
                if p is None: continue
                u,v = p
                if x1 <= u <= x2 and y1 <= v <= y2:
                    cnt += 1
            results.append((det, cnt))
        return results

    @staticmethod
    def points_to_cloud(points_xyz: np.ndarray, frame_id: str, stamp) -> PointCloud2:
        # points_xyz: (N,3)
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        # pack as float32
        pts32 = points_xyz.astype(np.float32)
        data = pts32.tobytes()
        cloud = PointCloud2(
            header=header,
            height=1,
            width=pts32.shape[0],
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=12*pts32.shape[0],
            data=data,
            is_dense=True
        )
        return cloud


def main():
    rclpy.init()
    node = LidarCameraFusionPro()
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

