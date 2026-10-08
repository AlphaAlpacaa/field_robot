#!/usr/bin/env python3
"""
tomato_detector.py

Finds ripe tomatoes in the RGB image, reads their distance from the depth
image, and publishes their 3D positions in the robot's frame.

Pipeline
--------
1. RGB -> HSV, threshold the two red hue bands (red wraps around 0/180 in HSV)
2. Morphological open/close to drop speckle
3. Contours -> keep blobs that are big enough and roughly circular
4. For each blob centre, take the median depth over a small patch (a single
   pixel is too noisy, and may land on a gap between leaves)
5. Back-project the pixel through the camera intrinsics to get a 3D point in
   the optical frame
6. Transform that point into the target frame (base_footprint by default)

Outputs
-------
/tomatoes/poses    geometry_msgs/PoseArray      for the picking node
/tomatoes/markers  visualization_msgs/MarkerArray  for RViz
/tomatoes/debug_image  sensor_msgs/Image        mask overlay, for tuning

Parameters
----------
camera        camera namespace, e.g. front_camera or wrist_camera
target_frame  frame the poses are published in
min_area      minimum blob area in pixels
min_circularity  0-1, how round a blob must be
max_range     ignore detections further away than this (m)

Usage
-----
    ros2 run field_robot tomato_detector.py
    ros2 run field_robot tomato_detector.py --ros-args -p camera:=wrist_camera
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class TomatoDetector(Node):

    def __init__(self):
        super().__init__('tomato_detector')

        self.declare_parameter('camera', 'front_camera')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('min_area', 60)
        self.declare_parameter('min_circularity', 0.6)
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('publish_debug_image', True)

        cam = self.get_parameter('camera').value
        self.target_frame = self.get_parameter('target_frame').value
        self.min_area = self.get_parameter('min_area').value
        self.min_circ = self.get_parameter('min_circularity').value
        self.max_range = self.get_parameter('max_range').value
        self.debug = self.get_parameter('publish_debug_image').value

        self.bridge = CvBridge()
        self.K = None          # camera intrinsics, filled from camera_info

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sensor data is best-effort; using the default reliable QoS here would
        # silently receive nothing.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        self.create_subscription(
            CameraInfo, f'/{cam}/camera_info', self.on_camera_info, sensor_qos)

        rgb_sub = message_filters.Subscriber(
            self, Image, f'/{cam}/image', qos_profile=sensor_qos)
        depth_sub = message_filters.Subscriber(
            self, Image, f'/{cam}/depth_image', qos_profile=sensor_qos)

        # RGB and depth are published separately and never share a timestamp
        # exactly, so pair them approximately.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.on_images)

        self.pose_pub = self.create_publisher(PoseArray, '/tomatoes/poses', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/tomatoes/markers', 10)
        self.debug_pub = self.create_publisher(
            Image, '/tomatoes/debug_image', 10)

        self.get_logger().info(
            f'tomato_detector running on /{cam}, publishing in '
            f'{self.target_frame}')

    # ------------------------------------------------------------------
    def on_camera_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.optical_frame = msg.header.frame_id
            self.get_logger().info(
                f'intrinsics received: fx={self.K[0,0]:.1f} '
                f'fy={self.K[1,1]:.1f} frame={self.optical_frame}')

    # ------------------------------------------------------------------
    def find_red_blobs(self, bgr):
        """Return (centre_x, centre_y, radius) for each ripe tomato, plus mask."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Red occupies both ends of the hue circle, so two ranges are needed.
        # Saturation/value lower bounds reject brown soil and dark shadow.
        lower1 = cv2.inRange(hsv, np.array([0, 120, 70]),
                             np.array([10, 255, 255]))
        lower2 = cv2.inRange(hsv, np.array([170, 120, 70]),
                             np.array([180, 255, 255]))
        mask = cv2.bitwise_or(lower1, lower2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        results = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            # 1.0 is a perfect circle; partially hidden fruit scores lower
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < self.min_circ:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(c)
            results.append((cx, cy, radius))

        return results, mask

    # ------------------------------------------------------------------
    def depth_at(self, depth, u, v, patch=3):
        """Median depth in a small window, ignoring invalid readings."""
        h, w = depth.shape[:2]
        u0, u1 = max(0, u - patch), min(w, u + patch + 1)
        v0, v1 = max(0, v - patch), min(h, v + patch + 1)
        window = depth[v0:v1, u0:u1].astype(np.float32).ravel()
        valid = window[np.isfinite(window) & (window > 0.01)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    # ------------------------------------------------------------------
    def on_images(self, rgb_msg, depth_msg):
        if self.K is None:
            return

        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        blobs, mask = self.find_red_blobs(bgr)

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx0, cy0 = self.K[0, 2], self.K[1, 2]

        poses = PoseArray()
        poses.header.frame_id = self.target_frame
        poses.header.stamp = rgb_msg.header.stamp

        markers = MarkerArray()
        debug = bgr.copy() if self.debug else None

        for i, (u, v, radius) in enumerate(blobs):
            z = self.depth_at(depth, int(u), int(v))
            if z is None or z > self.max_range:
                continue

            # pinhole back-projection, optical convention: x right, y down, z fwd
            point = PointStamped()
            point.header.frame_id = self.optical_frame
            point.header.stamp = rgb_msg.header.stamp
            point.point.x = (u - cx0) * z / fx
            point.point.y = (v - cy0) * z / fy
            point.point.z = z

            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, self.optical_frame,
                    rclpy.time.Time())        # latest available
                p = do_transform_point(point, tf)
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f'tf failed: {e}', throttle_duration_sec=5)
                continue

            pose = Pose()
            pose.position = p.point
            pose.orientation.w = 1.0
            poses.poses.append(pose)

            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = rgb_msg.header.stamp
            m.ns = 'tomatoes'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = pose
            m.scale.x = m.scale.y = m.scale.z = 0.07
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.1, 0.9
            m.lifetime.sec = 1
            markers.markers.append(m)

            if debug is not None:
                cv2.circle(debug, (int(u), int(v)), int(radius), (0, 255, 0), 2)
                cv2.putText(debug, f'{z:.2f}m', (int(u) - 20, int(v) - int(radius) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        self.pose_pub.publish(poses)
        self.marker_pub.publish(markers)

        if debug is not None:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            msg.header = rgb_msg.header
            self.debug_pub.publish(msg)


def main():
    rclpy.init()
    node = TomatoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
