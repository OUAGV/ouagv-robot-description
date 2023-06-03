import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_dir = get_package_share_directory("ouagv_robot_description")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_path = os.path.join(package_dir, "map", 'map.yaml')
    nav_params_path = os.path.join(
        package_dir, "config", 'nav2_params.yaml')
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    # rviz = os.path.join(package_dir, "rviz" , "first_robot.rviz")
    rviz = os.path.join(package_dir, "rviz", "navigation.rviz")

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': "/home/tatsuki/colcon_ws/src/ouagv-robot-description/map/map.yaml",
                'use_sim_time': use_sim_time,
                'params_file': nav_params_path}.items(),
        ),
    ])
