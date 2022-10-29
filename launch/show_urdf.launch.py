import os

import launch_ros.actions
import xacro
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='both',
    )

    return launch.LaunchDescription([rviz])
