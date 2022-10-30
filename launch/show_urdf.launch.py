import os

import launch_ros.actions
import xacro
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

share_dir_path = os.path.join(
    get_package_share_directory('ouagv_robot_description'))
urdf_path = os.path.join(share_dir_path, 'urdf', 'urdf_test.urdf')
rviz_config_path = os.path.join(share_dir_path, 'rviz', 'show_urdf.rviz')


def generate_launch_description():

    doc = xacro.process_file(urdf_path)
    robot_desc = doc.toprettyxml(indent='  ')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  arguments=[urdf_path],
                                  parameters=[{'robot_description': robot_desc}])

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='both',
        arguments=['-d', [rviz_config_path]])

    return launch.LaunchDescription([rsp, rviz])
