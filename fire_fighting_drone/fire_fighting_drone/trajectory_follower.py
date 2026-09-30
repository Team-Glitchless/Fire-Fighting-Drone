#!/usr/bin/env python3
"""ROS 2 (rclpy) trajectory follower.

Migration of the ROS 1 ``vc1.1.py`` node: flies a minimum-snap trajectory
(:mod:`fire_fighting_drone.trajectory_generator`) using velocity setpoints
corrected by a PID controller (:mod:`fire_fighting_drone.pid`, bug-fixed -
see that module's docstring).
"""
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from fire_fighting_drone import pid as pid_module
from fire_fighting_drone import trajectory_generator as traj
from fire_fighting_drone.flight_controller import FlightController


def avg_dist(points):
    i = points.shape[1]
    dr = points[:, 1:] - points[:, :i - 1]
    ds = np.sqrt(np.sum(dr * dr, axis=0))
    return np.sum(ds) / (i - 1)


def i_finder(path, i=100):
    points = path.points(i)
    avg = avg_dist(points)
    if avg < 0.03 or avg > 0.09:
        i = int(i * (avg / 0.05))
        return i_finder(path, i)
    return i


class TrajectoryFollower(Node):

    def __init__(self, flight_controller):
        super().__init__('trajectory_follower')
        self.trajectory_covered = False
        self.vmax = self.declare_parameter('v_max', 2.0).value
        self.vmin = self.declare_parameter('v_min', 0.3).value
        self.vnet = self.declare_parameter('v_net', 1.0).value
        self.derror = 5.0
        self.closest_point_index = 0
        self.target_point_index = None
        self.error = None
        self.current_points = None
        self.number_of_points = None
        self.number_of_total_points = None
        self.current_pose = np.array([0.0, 0.0, 0.0])
        self.angular_v = [0.0, 0.0, 0.0]
        self.linear_pid_factor = self.declare_parameter('linear_pid_factor', 0.5).value
        self.correction_of_path = False

        self.ic = flight_controller
        self.linear_pid = pid_module.PID()

    def data(self, waypoints):
        waypoints = np.array(waypoints)
        self.ms = traj.min_snap(waypoints[0], waypoints[1], waypoints[2], self.vnet)
        self.number_of_points = i_finder(self.ms)
        self.current_points = self.ms.points(self.number_of_points)
        self.number_of_total_points = self.current_points.shape[1]
        self.get_logger().info(
            f'Trajectory generated with {self.number_of_total_points} sampled points')

    def closest_point(self):
        i = max(0, self.closest_point_index - 5)
        j = min(self.number_of_points - 1, self.closest_point_index + 5)
        dr = self.current_points[:, i:j] - np.reshape(self.current_pose, (3, 1))
        ds = np.sum(dr * dr, axis=0)
        index = np.argmin(ds)
        self.closest_point_index = i + index
        self.error = np.sqrt(ds[index])
        if self.closest_point_index < (self.number_of_total_points - 3):
            self.target_point_index = self.closest_point_index + 1
        else:
            self.trajectory_covered = True

    def ds_finder(self):
        if self.error < 0.5:
            self.correction_of_path = False
        if self.error < self.derror and not self.correction_of_path:
            i = 1
            ds = (self.current_points[:, self.closest_point_index + i] -
                  self.current_points[:, self.closest_point_index])
            temp_ds = np.sqrt(np.sum(ds * ds))
            while temp_ds * 20 < self.vmin:
                ds = (self.current_points[:, self.closest_point_index + i] -
                      self.current_points[:, self.closest_point_index])
                temp_ds = np.sqrt(np.sum(ds * ds))
                i += 1
            if temp_ds * 20 > self.vmax:
                new_derivative = (ds / temp_ds) * self.vmax / 20
            else:
                new_derivative = ds
        else:
            if not self.correction_of_path:
                self.correction_of_path = True
            temp = self.current_points[:, self.closest_point_index]
            dr = temp - self.current_pose
            ds = np.sqrt(np.sum(dr * dr))
            if ds * 20 > self.vmax:
                new_derivative = (dr / ds) * self.vmax / 20.0
            else:
                new_derivative = dr
        return new_derivative

    def send_velocity(self):
        self.current_pose = np.array(self.ic.current_position)
        self.closest_point()
        dr = self.ds_finder()
        v_out = dr * 20
        pid_output = self.linear_pid.update(
            self.current_points[:, self.closest_point_index], self.current_pose)
        v_out = v_out - self.linear_pid_factor * pid_output
        self.ic.update_target_vel(v_out, self.angular_v)

    def start_the_flight(self):
        self.current_pose = np.array(self.ic.current_position)
        self.closest_point()
        while not self.trajectory_covered and rclpy.ok():
            self.send_velocity()
            self.current_pose = np.array(self.ic.current_position)
            self.closest_point()
            time.sleep(0.05)
        self.get_logger().info('Trajectory covered')


def main(args=None):
    rclpy.init(args=args)
    flight_controller = FlightController()
    follower = TrajectoryFollower(flight_controller)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(flight_controller)
    executor.add_node(follower)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        x = [0, -3, -8, -9, -9, -8, -6]
        y = [0, 3, 0, -5, -7, -12, -15]
        z = [5, 5, 5, 5, 5, 5, 5]
        alt = 5.0

        flight_controller.toggle_arm(True)
        flight_controller.takeoff(alt)
        flight_controller.set_offboard_mode()

        pose = flight_controller.current_position
        x[0], y[0], z[0] = pose[0], pose[1], pose[2]
        follower.data([x, y, z])
        follower.start_the_flight()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        follower.destroy_node()
        flight_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
