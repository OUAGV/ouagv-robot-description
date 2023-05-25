# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import xacro
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

gazebo_ros2_control_demos_path = os.path.join(
    get_package_share_directory("ouagv_robot_description")
)
xacro_file = os.path.join(
    gazebo_ros2_control_demos_path, "xacro", "diff_drive_robot.urdf.xacro"
)
urdf_path = os.path.join(
    gazebo_ros2_control_demos_path, "xacro", "diff_drive_robot.urdf"
)


def generate_launch_description():
    show_rviz = LaunchConfiguration('show_rviz', default=True)

    rviz_config_path = os.path.join(
        gazebo_ros2_control_demos_path, "rviz", "show_urdf.rviz"
    )

    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent="  ")
    f = open(urdf_path, "w")
    f.write(robot_desc)
    f.close()
    params = {"robot_description": robot_desc}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", [rviz_config_path]],
        condition=IfCondition(show_rviz)
    )

    joy_node = Node(package="joy", executable="joy_node", output="screen")

    joy_to_twist = Node(
        package="joy_to_twist",
        executable="joy_to_twist_node",
        parameters=[{"longitudal_input_ratio": 0.8},
                    {"lateral_input_ratio": 1.0}],
        remappings=[
            ("/target_twist", "/cmd_vel")],)

    return LaunchDescription(
        [
            joy_node,
            joy_to_twist,
            node_robot_state_publisher,
            rviz,
        ]
    )
