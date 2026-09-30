#!/usr/bin/env python3
"""ROS 2 (rclpy) flight controller node for the PX4/MAVROS SITL drone.

This merges and migrates the two ROS 1 controllers that used to live side by
side (``controler.py`` - position setpoints, and ``controler_v1.py`` -
velocity setpoints) into a single node exposing both control modes, and adds
autonomous handling of ``/next_goal`` messages published by the RRT-NBVP
exploration planner (``fire_fighting_drone_cpp``), closing the loop that was
previously missing between exploration and navigation.

Because blocking helper methods such as :meth:`FlightController.move_to`
poll node state that is only updated by subscription callbacks, this node
must be spun by an executor running in a background thread (see
:func:`main`) while the main thread drives a mission sequence.
"""
import math
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix
import tf_transformations


class FlightController(Node):

    def __init__(self):
        super().__init__('flight_controller')

        # Data.
        self.gps_lat = 0.0
        self.gps_long = 0.0
        self.gps_alt = 0.0
        self.gps_alt_correction = 535.298913472
        self.img_count = 0

        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0
        self.curr_roll = 0.0
        self.curr_pitch = 0.0
        self.curr_yaw = 0.0
        self.current_position = [0.0, 0.0, 0.0]

        self.set_x = 0.0
        self.set_y = 0.0
        self.set_z = 0.0
        self.set_yaw = 0.0
        self.set_ori_x = 0.0
        self.set_ori_y = 0.0
        self.set_ori_z = 0.707
        self.set_ori_w = 0.707

        self.set_linear_velocity = [0.0, 0.0, 0.0]
        self.set_angular_velocity = [0.0, 0.0, 0.0]

        self.bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        self.delta = self.declare_parameter('position_tolerance', 0.1).value
        self.delta_yaw = self.declare_parameter('yaw_tolerance', 0.01).value
        self.waypoint_number = 0
        self.follow_nbvp_goals = self.declare_parameter('follow_nbvp_goals', True).value
        self.nbvp_altitude = self.declare_parameter('nbvp_altitude_override', 0.0).value

        # Subscribers.
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.get_pose, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/r200/rgb/image_raw', self.get_rgb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/r200/depth/image_raw', self.get_depth, qos_profile_sensor_data)
        self.create_subscription(
            PointStamped, '/next_goal', self.next_goal_callback, 10)

        # Publishers.
        self.publish_pose = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # Services.
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.last_vel_time = self.get_clock().now()

        # Single dedicated worker thread consumes NBVP goals one at a time so
        # concurrent /next_goal messages cannot race on the shared
        # set_x/set_y/set_z/set_yaw setpoint state used by move_to()/set_pose().
        self._goal_lock = threading.Lock()
        self._pending_goal = None
        self._goal_event = threading.Event()
        self._goal_worker = threading.Thread(target=self._goal_worker_loop, daemon=True)
        self._goal_worker.start()

        self.get_logger().info('Flight controller initialized')

    # ------------------------------------------------------------------
    # Mode setup
    # ------------------------------------------------------------------

    def _call_service(self, client, request, timeout_sec=5.0):
        """Calls a service without blocking the executor thread.

        This node is spun by a ``MultiThreadedExecutor`` running in a
        background thread (see :func:`main`), so we must not call
        ``rclpy.spin_until_future_complete`` here: that would spin this same
        node from a second thread concurrently with the executor and is not
        safe. Instead we simply poll the future while the executor thread
        processes its completion in the background.
        """
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return None
        future = client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout_sec:
            time.sleep(0.05)
        return future.result()

    def toggle_arm(self, arm_bool):
        request = CommandBool.Request()
        request.value = arm_bool
        self._call_service(self.arm_client, request)

    def takeoff(self, t_alt):
        time.sleep(2)
        t_lat = self.gps_lat
        t_long = self.gps_long
        if self.gps_alt > 1.0:
            self.get_logger().info('Drone is in air')
            if self.gps_alt > t_alt:
                return
            if self.gps_alt < t_alt:
                t_alt -= self.gps_alt

        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = t_lat
        request.longitude = t_long
        request.altitude = t_alt
        self._call_service(self.takeoff_client, request)
        time.sleep(5)

    def land(self, t_alt):
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = self.gps_lat
        request.longitude = self.gps_long
        request.altitude = t_alt
        self._call_service(self.land_client, request)
        self.get_logger().info('LANDING')

    def set_offboard_mode(self):
        request = SetMode.Request()
        request.custom_mode = 'OFFBOARD'
        self._call_service(self.set_mode_client, request)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def gps_callback(self, data):
        self.gps_lat = data.latitude
        self.gps_long = data.longitude
        self.gps_alt = data.altitude
        if self.gps_alt >= 530:
            self.gps_alt -= self.gps_alt_correction

    def get_pose(self, location_data):
        self.curr_x = location_data.pose.position.x
        self.curr_y = location_data.pose.position.y
        self.curr_z = location_data.pose.position.z
        self.current_position = [self.curr_x, self.curr_y, self.curr_z]
        rot_q = location_data.pose.orientation
        (self.curr_roll, self.curr_pitch, self.curr_yaw) = tf_transformations.euler_from_quaternion(
            [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def get_rgb(self, rgb_data):
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_data, 'bgr8').copy()

    def get_depth(self, depth_data):
        self.depth_image = self.depth_bridge.imgmsg_to_cv2(depth_data, '32FC1').copy()

    def next_goal_callback(self, msg):
        """Fly to the next exploration viewpoint published by the NBVP planner.

        This closes the previously-missing integration gap between the
        exploration planner (``/next_goal``) and the flight controller: the
        ROS 1 planner published viewpoints that nothing ever consumed.

        The goal is handed off to a single dedicated worker thread (see
        :meth:`_goal_worker_loop`) rather than spawned as a new thread per
        message, so that concurrent goals cannot race on the shared
        setpoint state used by :meth:`move_to`/:meth:`set_pose`. If a newer
        goal arrives while an older one is still in flight, only the newest
        goal is flown to once the current motion completes.
        """
        if not self.follow_nbvp_goals:
            return
        z = msg.point.z if self.nbvp_altitude == 0.0 else self.nbvp_altitude
        self.get_logger().info(
            f'Received NBVP goal: ({msg.point.x:.2f}, {msg.point.y:.2f}, {z:.2f})')
        with self._goal_lock:
            self._pending_goal = (msg.point.x, msg.point.y, z)
        self._goal_event.set()

    def _goal_worker_loop(self):
        """Serially consumes NBVP goals, always flying to the latest one."""
        while rclpy.ok():
            self._goal_event.wait()
            with self._goal_lock:
                goal = self._pending_goal
                self._pending_goal = None
                self._goal_event.clear()
            if goal is not None:
                self.move_to(*goal)

    # ------------------------------------------------------------------
    # Position-setpoint control
    # ------------------------------------------------------------------

    def set_waypoint(self, x, y, z):
        self.set_x = x
        self.set_y = y
        self.set_z = z

    def set_orientation(self, roll, pitch, yaw):
        self.set_yaw = yaw
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        self.set_ori_x, self.set_ori_y, self.set_ori_z, self.set_ori_w = q

    def set_pose(self, rate_hz=20.0, timeout_sec=30.0):
        period = 1.0 / rate_hz
        deadline = time.time() + timeout_sec

        def distance():
            return math.sqrt(
                (self.set_x - self.curr_x) ** 2 +
                (self.set_y - self.curr_y) ** 2 +
                (self.set_z - self.curr_z) ** 2)

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.set_x
        pose_msg.pose.position.y = self.set_y
        pose_msg.pose.position.z = self.set_z
        pose_msg.pose.orientation.x = self.set_ori_x
        pose_msg.pose.orientation.y = self.set_ori_y
        pose_msg.pose.orientation.z = self.set_ori_z
        pose_msg.pose.orientation.w = self.set_ori_w

        while distance() > self.delta and time.time() < deadline:
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.publish_pose.publish(pose_msg)
            time.sleep(period)

        while (self.curr_yaw - self.set_yaw) ** 2 > self.delta_yaw and time.time() < deadline:
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.publish_pose.publish(pose_msg)
            time.sleep(period)

        self.waypoint_number += 1
        self.get_logger().info(f'Waypoint reached: {self.waypoint_number}')

    def move_to(self, x, y, z):
        self.set_waypoint(x, y, z)
        self.set_pose()

    # ------------------------------------------------------------------
    # Velocity-setpoint control
    # ------------------------------------------------------------------

    def set_velocity(self):
        val_msg = TwistStamped()
        val_msg.header.stamp = self.get_clock().now().to_msg()
        val_msg.twist.linear.x = self.set_linear_velocity[0]
        val_msg.twist.linear.y = self.set_linear_velocity[1]
        val_msg.twist.linear.z = self.set_linear_velocity[2]
        val_msg.twist.angular.x = self.set_angular_velocity[0]
        val_msg.twist.angular.y = self.set_angular_velocity[1]
        val_msg.twist.angular.z = self.set_angular_velocity[2]
        self.vel_pub.publish(val_msg)
        self.last_vel_time = self.get_clock().now()

    def update_target_vel(self, linear, angular):
        self.set_linear_velocity = list(linear)
        self.set_angular_velocity = list(angular)
        elapsed = (self.get_clock().now() - self.last_vel_time).nanoseconds * 1e-9
        if elapsed < (1.0 / 20.0):
            time.sleep(max(0.0, 0.05 - elapsed))
        self.set_velocity()

    # ------------------------------------------------------------------
    # Image capture helpers (data collection)
    # ------------------------------------------------------------------

    def save_rgb(self):
        if self.rgb_image is None:
            return
        rgb_filename = f'drone_img/rgb/rgb_camera_image{self.img_count}.jpeg'
        cv2.imwrite(rgb_filename, self.rgb_image)
        self.img_count += 1

    def save_depth(self):
        if self.depth_image is None:
            return
        depth_filename = f'drone_img/depth/depth_camera_image{self.img_count}.png'
        cv2.imwrite(depth_filename, self.depth_image)
        self.img_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = FlightController()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.toggle_arm(True)
        node.takeoff(3.0)
        node.set_offboard_mode()
        node.get_logger().info('Ready: waiting for NBVP goals on /next_goal')
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
