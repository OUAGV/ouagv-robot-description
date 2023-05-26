import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

share_dir = os.path.join(
    get_package_share_directory("ouagv_robot_description"))


def generate_launch_description():
    show_rviz = LaunchConfiguration('show_rviz', default=True)
    diff_drive_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(share_dir, "launch"),
             "/show_diff_drive_robot.launch.py"]
        ), launch_arguments={"show_rviz": "true"}.items()
    )

    joy_node = Node(package="joy", executable="joy_node", output="screen")

    joy_to_twist = Node(
        package="joy_to_twist",
        executable="joy_to_twist_node",
        parameters=[{"longitudal_input_ratio": 0.4},
                    {"lateral_input_ratio": 0.4}],
        remappings=[
            ("/target_twist", "/cmd_vel")],
    )

    return LaunchDescription([diff_drive_robot, joy_node, joy_to_twist])
