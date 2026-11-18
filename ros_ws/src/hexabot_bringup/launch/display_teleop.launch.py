"""RViz display driven by the keyboard teleop (no joint sliders).

Reuses hexabot_description's display.launch.py with gui:=false, runs the control
chain, and feeds the bridge's joint commands straight into /joint_states so
robot_state_publisher animates the model.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('hexabot_description')

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'display.launch.py')),
        launch_arguments={
            'gui': 'false',
            'rviz': LaunchConfiguration('rviz'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gait', default_value='tripod',
                              description='Initial gait: tripod, ripple or wave'),
        DeclareLaunchArgument('stand_height', default_value='0.06'),
        DeclareLaunchArgument('stance_radius', default_value='0.11'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='Launch keyboard_teleop in a terminal window. Use false and '
                        'run `ros2 run hexabot_control keyboard_teleop` yourself.'),
        DeclareLaunchArgument(
            'teleop_prefix', default_value='xterm -e',
            description='Terminal wrapper for the keyboard (it needs a real tty).'),
        display,
        Node(package='hexabot_control', executable='leg_ik', name='leg_ik',
             output='screen'),
        Node(package='hexabot_control', executable='body_controller',
             name='body_controller', output='screen',
             parameters=[{
                 'gait': LaunchConfiguration('gait'),
                 'stand_height': LaunchConfiguration('stand_height'),
                 'stance_radius': LaunchConfiguration('stance_radius'),
             }]),
        # Isaac backend publishes sensor_msgs/JointState; point it at /joint_states
        # so robot_state_publisher consumes it directly.
        Node(package='hexabot_control', executable='joint_command_bridge',
             name='joint_command_bridge', output='screen',
             parameters=[{'backend': 'isaac', 'command_topic': '/joint_states'}]),
        Node(package='hexabot_control', executable='keyboard_teleop',
             name='keyboard_teleop', output='screen',
             prefix=LaunchConfiguration('teleop_prefix'),
             condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
