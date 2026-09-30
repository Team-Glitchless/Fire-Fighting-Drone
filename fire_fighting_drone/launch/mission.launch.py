"""Bring up the full autonomous exploration + search-and-rescue mission.

Starts (all on ROS 2 Jazzy):
  * the point cloud frame-transform node (ORB-SLAM2 -> world/NEU),
  * the RRT-based Next-Best-View exploration planner,
  * the flight controller (consumes /next_goal, controls MAVROS/PX4), and
  * the YOLOv3 human-detection node (publishes /detected_humans).

``octomap_server`` itself is intentionally not started here: it is a
separate, third-party ROS 2 package (see README.md) that must be installed
and configured to subscribe to ``/pcl_out`` and publish ``/octomap_binary``.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ffd_share = get_package_share_directory('fire_fighting_drone')
    params_file = os.path.join(ffd_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='fire_fighting_drone_cpp',
            executable='pointcloud_transform_node',
            name='pcl_transform',
            output='screen',
        ),
        Node(
            package='fire_fighting_drone_cpp',
            executable='nbvp_planner_node',
            name='nbvp_planner',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='fire_fighting_drone',
            executable='flight_controller',
            name='flight_controller',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='fire_fighting_drone',
            executable='object_detection',
            name='object_detection',
            output='screen',
            parameters=[params_file],
        ),
    ])
