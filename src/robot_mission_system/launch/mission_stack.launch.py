"""Teach + mission_manager (+ optional status)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_mission_system')
    params = os.path.join(pkg, 'config', 'mission_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_system_status', default_value='false'),
        Node(
            package='robot_mission_system',
            executable='teach_waypoint',
            name='teach_waypoint',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='robot_mission_system',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='robot_mission_system',
            executable='system_status',
            name='system_status',
            output='screen',
            parameters=[params],
            condition=IfCondition(LaunchConfiguration('use_system_status')),
        ),
    ])
