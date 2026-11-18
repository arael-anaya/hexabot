import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hexabot_description')
    with open(os.path.join(share, 'urdf', 'hexabot.urdf')) as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Slider GUI for joint positions'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # Start pose: every joint 3 at its 0.162 lower limit (zero is out of range, so
        # without this the sliders would start at the middle of the range).
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'zeros': {f'a{leg}_joint3': 0.162 for leg in range(1, 7)}}],
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'hexabot.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
