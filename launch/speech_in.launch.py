import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

default_wake_word="elsa:2bf1166a-0dbd-43ea-a08e-b3456a06a446.table"

def generate_launch_description():
    wake_word_dir = os.path.join(get_package_share_directory('speech_input_server'), 'wake_words')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='wake_word',
            default_value=os.path.join(wake_word_dir, default_wake_word)
        ),

        Node(
            package='speech_input_server',
            executable='speech_input_action_server',
            parameters=[
                {'wake_word': LaunchConfiguration('wake_word')},
            ],
        )
    ])