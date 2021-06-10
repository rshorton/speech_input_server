from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_input_server',
            executable='speech_input_action_server'
        )
    ])