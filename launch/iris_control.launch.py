from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='iris_control',
            executable='thrust_setpoints_pub',
            name='thrust_setpoints_pub'
        ),
        Node(
            package='iris_control',
            executable='acoustic_pilot',
            name='acoustic_pilot'
        )
    ])