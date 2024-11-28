import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='node',  # Replace with your package name
            executable='stm32_interface',  # Replace with your node executable name
            name='stm32_interface',
            output='screen',
            parameters=[],
            remappings=[],
        ),
    ])