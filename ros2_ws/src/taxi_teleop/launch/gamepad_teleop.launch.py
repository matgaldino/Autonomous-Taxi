#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('taxi_teleop'), 'config', '8bitdo_ultimate_2c.yaml')

    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node', output='screen', ),
        Node(package='taxi_teleop', executable='gamepad_teleop', name='gamepad_teleop', parameters=[config], output='screen', ),
    ])