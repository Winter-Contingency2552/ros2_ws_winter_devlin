import re
from struct import pack
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def launch_setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory('robot_bringup'))

    urdf = (pkg_share / 'urdf' / 'robot.urdf').read_text()

    rviz_path = str(pkg_share / 'rviz' / 'simulation.rviz')

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        ),
        Node(
            package='robot_simulator_cpp',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),
        Node(
            package='robot_simulator_py',
            executable='controller_node',
            name='controller_node',
            output='screen'
        )
    ]
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Full path to the RVIZ config file to use'
        ),
        OpaqueFunction(function=launch_setup)
    ])