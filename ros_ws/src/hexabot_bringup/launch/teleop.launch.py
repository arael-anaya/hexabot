from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='isaac',
                              description='isaac or ros2_control'),
        DeclareLaunchArgument('gait', default_value='tripod',
                              description='tripod, ripple or wave'),
        DeclareLaunchArgument('stand_height', default_value='0.06'),
        DeclareLaunchArgument('stance_radius', default_value='0.11'),
        Node(package='hexabot_control', executable='leg_ik', name='leg_ik',
             output='screen'),
        Node(package='hexabot_control', executable='body_controller',
             name='body_controller', output='screen',
             parameters=[{
                 'gait': LaunchConfiguration('gait'),
                 'stand_height': LaunchConfiguration('stand_height'),
                 'stance_radius': LaunchConfiguration('stance_radius'),
             }]),
        Node(package='hexabot_control', executable='joint_command_bridge',
             name='joint_command_bridge', output='screen',
             parameters=[{'backend': LaunchConfiguration('backend')}]),
    ])
