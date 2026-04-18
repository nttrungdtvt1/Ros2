"""
Real robot: hardware + LD06 + AMCL + Nav2 + optional mission nodes.

Map default: share/articubot_one/maps/my_room_map.yaml (install maps via CMakeLists).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    articubot = get_package_share_directory('articubot_one')
    default_map = os.path.join(articubot, 'maps', 'my_room_map.yaml')

    use_ldlidar = LaunchConfiguration('use_ldlidar')
    use_mission = LaunchConfiguration('use_mission')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(articubot, 'launch', 'launch_robot.launch.py')),
    )

    ldlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ldlidar'),
                'launch',
                'ldlidar.launch.py',
            )),
        launch_arguments={
            'lidar_frame': 'laser_frame',
            'topic_name': 'scan',
        }.items(),
    )

    loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(articubot, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': default_map,
            'use_sim_time': 'false',
        }.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(articubot, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_mission_system'),
                'launch',
                'mission_stack.launch.py',
            )),
        launch_arguments={
            'use_system_status': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_ldlidar', default_value='true'),
        DeclareLaunchArgument('use_mission', default_value='true'),
        robot,
        GroupAction(actions=[ldlidar], condition=IfCondition(use_ldlidar)),
        loc,
        nav,
        GroupAction(actions=[mission], condition=IfCondition(use_mission)),
    ])
