#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

apriltag_steering_wheel = get_package_share_directory('apriltag_steering_wheel')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            namespace="camera",
            parameters=[os.path.join(apriltag_steering_wheel, "config/params.yaml")],
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(apriltag_steering_wheel, "launch", "tag_16h5_9.launch.py")),
            launch_arguments={}.items(),
            ),
        ])
