#!/usr/bin/env python3
"""ROS 2 (rclpy) YOLOv3-based human detection node.

Migration of the ROS 1 ``yolo_script_v1.py``. The most important behavioral
change is that detections are now *published* on ``/detected_humans`` as
``geometry_msgs/PointStamped`` messages (one per detection, world-frame x/y)
instead of only being printed to stdout, closing the gap between detection
and any downstream search-and-rescue coordination consumer.
"""
import math

import numpy as np
import rclpy
import tensorflow as tf
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import tf_transformations

from fire_fighting_drone import ml_helper as ml

CLASS_NAMES = ml.class_names


class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0
        self.curr_yaw = 0.0

        self.bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        self.model = ml.YoloV3()
        weights_path = self.declare_parameter(
            'weights_path', 'checkpoints/yolov3.tf').value
        self.model.load_weights(weights_path)

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.get_pose, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/r200/rgb/image_raw', self.get_rgb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/r200/depth/image_raw', self.get_depth, qos_profile_sensor_data)

        self.detection_pub = self.create_publisher(PointStamped, '/detected_humans', 10)
        self.detection_size = self.declare_parameter('image_size', 416).value

        timer_period = self.declare_parameter('detection_period_sec', 0.2).value
        self.create_timer(timer_period, self.detect_once)

        self.get_logger().info('Object detection node initialized')

    def get_pose(self, location_data):
        self.curr_x = location_data.pose.position.x
        self.curr_y = location_data.pose.position.y
        self.curr_z = location_data.pose.position.z
        rot_q = location_data.pose.orientation
        (_, _, self.curr_yaw) = tf_transformations.euler_from_quaternion(
            [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def get_rgb(self, rgb_data):
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_data, 'bgr8').copy()

    def get_depth(self, depth_data):
        self.depth_image = self.depth_bridge.imgmsg_to_cv2(depth_data, '32FC1').copy()

    def detect_once(self):
        if self.rgb_image is None or self.depth_image is None:
            return

        img_raw = self.rgb_image
        depth = self.depth_image
        tx, ty, tyaw = self.curr_x, self.curr_y, self.curr_yaw

        img = tf.expand_dims(img_raw, 0)
        img = ml.preprocess_image(img, self.detection_size)
        outputs = self.model.predict(img)

        num_detections = int(outputs[3][0])
        if num_detections <= 0:
            return

        positions = self.localize_detections(img_raw, depth, outputs, tx, ty, tyaw)
        classes = outputs[2][0]
        stamp = self.get_clock().now().to_msg()
        for i in range(positions.shape[1]):
            x, y = positions[0, i], positions[1, i]
            if math.isnan(x) or math.isnan(y):
                continue
            label = CLASS_NAMES[int(classes[i])]
            if label != 'person':
                continue
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'world'
            msg.point.x = float(x)
            msg.point.y = float(y)
            msg.point.z = 0.0
            self.detection_pub.publish(msg)
            self.get_logger().info(f'Detected {label} at ({x:.2f}, {y:.2f})')

    def localize_detections(self, rgb, depth, outputs, tx, ty, tyaw):
        rgb = np.array(rgb)
        depth = np.array(depth)
        centers = center_finder(rgb, outputs)
        dz = dist_z_finder(centers, depth)
        dx = dist_x_finder(dz, rgb.shape, centers)
        return final_pos(dz, dx, tx, ty, tyaw)


def center_finder(img, outputs):
    centers = []
    wh = np.flip(img.shape[0:2])
    boxes, _score, _classes, nums = outputs
    boxes = boxes[0]
    for i in range(nums[0]):
        x1y1 = np.array((np.array(boxes[i][0:2]) * wh).astype(np.int32))
        x2y2 = np.array((np.array(boxes[i][2:4]) * wh).astype(np.int32))
        xy = ((x1y1 + x2y2) // 2).astype(np.int32)
        centers.append(xy)
    return np.array(centers, dtype=np.int32)


def dist_z_finder(centers, depth):
    j = 3
    dist = []
    for i in range(centers.shape[0]):
        box = depth[centers[i, 1] - j:centers[i, 1] + j, centers[i, 0] - j:centers[i, 0] + j]
        dist.append(np.sum(box) / box.size)
    return np.array(dist)


def dist_x_finder(z, shape_rgb, centers, h_pov=1.3962634):
    p_fac = h_pov / shape_rgb[1]
    c_x = centers[:, 0]
    px = shape_rgb[1] / 2
    dr = c_x - px
    angle = np.array(dr) * p_fac
    return z * np.tan(angle)


def final_pos(z, x, d_x, d_y, yaw):
    out_x = d_x + z * math.cos(yaw) + x * math.sin(yaw)
    out_y = d_y + z * math.sin(yaw) - x * math.cos(yaw)
    return np.array([out_x, out_y])


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
