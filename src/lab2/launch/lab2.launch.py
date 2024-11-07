from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab2',
            executable='publisher',
            output='screen'),
        Node(
            package='lab2',
            executable='subscriber',
            output='screen'),
    ])